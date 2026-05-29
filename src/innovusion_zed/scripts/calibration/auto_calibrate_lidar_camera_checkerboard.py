#!/usr/bin/env python3

"""Automatic checkerboard-plane LiDAR-camera calibration evaluator.

This script detects a checkerboard in RGB images, fits the corresponding
checkerboard plane in each PCD, evaluates several plane-constrained extrinsic
updates against an existing trusted calibration, and writes ranked candidate
JSON files. The recommended conservative mode is usually
`normal_align_t_prior_5cm`: align plane normals while keeping translation close
to the trusted calibration.
"""

import argparse, json, math
from pathlib import Path
import numpy as np
import cv2

def parse_args():
    p=argparse.ArgumentParser()
    p.add_argument('--data-dir', required=True, nargs='+',
                   help='One or more true_data directories. Samples from all directories are calibrated together.')
    p.add_argument('--config', default='src/innovusion_zed/config/camera_left.json')
    p.add_argument('--out-dir', required=True)
    p.add_argument('--pattern-cols', type=int, default=9)
    p.add_argument('--pattern-rows', type=int, default=6)
    p.add_argument('--square-size', type=float, default=0.13)
    return p.parse_args()
args=parse_args()
DATA_DIRS=[Path(x) for x in args.data_dir]; CFG=Path(args.config); OUTDIR=Path(args.out_dir); OUTDIR.mkdir(parents=True,exist_ok=True)
PATTERN=(args.pattern_cols,args.pattern_rows); SQUARE=args.square_size
AX=np.array([[0,0,1],[0,-1,0],[1,0,0]], dtype=np.float64)
cfg=json.load(open(CFG)); K=np.array(cfg['intrinsic'],float).reshape(3,3); dist=np.array(cfg['distortion'],float).reshape(-1,1)
R_s=np.array(cfg['rotation'],float).reshape(3,3); t_s=np.array(cfg['translation'],float)
R0=np.linalg.inv(R_s)@AX; t0=-np.linalg.inv(R_s)@t_s
objp=np.zeros((PATTERN[0]*PATTERN[1],3),np.float32); objp[:,:2]=np.mgrid[0:PATTERN[0],0:PATTERN[1]].T.reshape(-1,2); objp*=SQUARE

def R_to_r(R): return cv2.Rodrigues(R)[0].ravel()
def r_to_R(r): return cv2.Rodrigues(np.asarray(r).reshape(3,1))[0]
def angleR(A,B):
    v=(np.trace(A@B.T)-1)/2; return math.degrees(math.acos(max(-1,min(1,v))))
def read_pcd(path):
    with open(path,'rb') as f:
        meta={}
        while True:
            line=f.readline().decode('ascii',errors='ignore').strip(); parts=line.split()
            if parts: meta[parts[0]]=parts[1:]
            if line.startswith('DATA'):
                data=parts[1]; break
        fields=meta['FIELDS']; sizes=list(map(int,meta['SIZE'])); types=meta['TYPE']; n=int(meta.get('POINTS',meta.get('WIDTH'))[0])
        if data=='ascii':
            arr=np.loadtxt(f,dtype=np.float32); cols={k:i for i,k in enumerate(fields)}; return arr[:,[cols['x'],cols['y'],cols['z']]].astype(float)
        ds=[]
        for name,sz,tp in zip(fields,sizes,types):
            if tp=='F' and sz==4: dt='<f4'
            elif tp=='F' and sz==8: dt='<f8'
            elif tp=='U' and sz==4: dt='<u4'
            elif tp=='U' and sz==2: dt='<u2'
            elif tp=='U' and sz==1: dt='u1'
            elif tp=='I' and sz==4: dt='<i4'
            else: dt='<f4'
            ds.append((name,dt))
        arr=np.frombuffer(f.read(n*np.dtype(ds).itemsize),dtype=np.dtype(ds),count=n)
    return np.vstack([arr['x'],arr['y'],arr['z']]).T.astype(float)
def plane(points):
    c=points.mean(0); _,_,vh=np.linalg.svd(points-c,full_matrices=False); n=vh[-1]; n=n/np.linalg.norm(n); return n,-n@c,c
def ransac(points,th=0.035,iters=300):
    if len(points)<80: return None
    rng=np.random.default_rng(4); best=-1; bestm=None
    for _ in range(iters):
        ids=rng.choice(len(points),3,replace=False); p=points[ids]
        n=np.cross(p[1]-p[0],p[2]-p[0]); nn=np.linalg.norm(n)
        if nn<1e-6: continue
        n=n/nn; d=-n@p[0]; m=np.abs(points@n+d)<th
        if m.sum()>best: best=int(m.sum()); bestm=m
    if bestm is None or best<80: return None
    n,d,c=plane(points[bestm]); return n,d,c,best,int(len(points))
def project(xyz,R,t):
    cam=(R@xyz.T+t[:,None]).T; uv=np.full((len(xyz),2),np.nan); valid=cam[:,2]>0.1
    if valid.any(): uv[valid]=cv2.projectPoints(cam[valid].reshape(-1,1,3),np.zeros(3),np.zeros(3),K,dist)[0].reshape(-1,2)
    return uv,cam
pairs=[]; rejected=[]
for DATA in DATA_DIRS:
    for imgp in sorted((DATA/'left').glob('*.png')):
        stem=imgp.stem; sample_id=f'{DATA.name}/{stem}'; pcdp=DATA/'pcd'/(stem+'.pcd')
        if not pcdp.exists(): continue
        img=cv2.imread(str(imgp),0)
        ok,corners=cv2.findChessboardCornersSB(img,PATTERN,flags=cv2.CALIB_CB_NORMALIZE_IMAGE) if hasattr(cv2,'findChessboardCornersSB') else (False,None)
        if not ok:
            ok,corners=cv2.findChessboardCorners(img,PATTERN,flags=cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_NORMALIZE_IMAGE)
            if ok: cv2.cornerSubPix(img,corners,(11,11),(-1,-1),(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,0.001))
        if not ok:
            rejected.append((sample_id,'no_corners')); continue
        ok,rvec,tvec=cv2.solvePnP(objp,corners,K,dist,flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            rejected.append((sample_id,'solvepnp_fail')); continue
        Rb=r_to_R(rvec.ravel()); tb=tvec.ravel(); nc=Rb[:,2]; nc/=np.linalg.norm(nc); dc=-nc@tb
        if dc<0: nc=-nc; dc=-dc
        board_center=Rb@np.array([SQUARE*(PATTERN[0]-1)/2,SQUARE*(PATTERN[1]-1)/2,0.0])+tb
        xyz=read_pcd(pcdp); xyz=xyz[np.isfinite(xyz).all(axis=1)]
        uv,cam=project(xyz,R0,t0); x0,y0=corners.reshape(-1,2).min(0); x1,y1=corners.reshape(-1,2).max(0)
        margin=120
        mask=(uv[:,0]>x0-margin)&(uv[:,0]<x1+margin)&(uv[:,1]>y0-margin)&(uv[:,1]<y1+margin)&(cam[:,2]>max(0.2,tb[2]-2.0))&(cam[:,2]<tb[2]+2.0)
        cand=xyz[mask]; fit=ransac(cand)
        if fit is None:
            rejected.append((sample_id,f'plane_fail cand={len(cand)}')); continue
        nl,dl,cl,inliers,total=fit
        if (R0@nl)@nc<0: nl=-nl; dl=-dl
        pairs.append(dict(stem=sample_id,nc=nc,dc=dc,nl=nl,dl=dl,cl=cl,center_cam=board_center,inliers=inliers,total=total,cam_z=tb[2]))
print('DATA_DIRS',[str(x) for x in DATA_DIRS],'pairs',len(pairs),'rejected',len(rejected))
for item in rejected[:20]: print('reject',item)
if not pairs:
    raise RuntimeError('No valid calibration pairs were found. Check data directories, checkerboard pattern size, and PCD files.')

def plane_metrics(R,t):
    ang=[]; dist=[]
    for p in pairs:
        npred=R@p['nl']; dpred=p['dl']-npred@t
        if npred@p['nc']<0: npred=-npred; dpred=-dpred
        ang.append(math.degrees(math.acos(max(-1,min(1,npred@p['nc']))))); dist.append(dpred-p['dc'])
    return np.array(ang),np.array(dist)
def center_metrics(R,t): return np.array([np.linalg.norm(R@p['cl']+t-p['center_cam']) for p in pairs])
def convert(R,t): Rst=AX@R.T; tst=-(Rst@t); return Rst,tst
def delta(R,t):
    Rst,tst=convert(R,t); return angleR(Rst,R_s),np.linalg.norm(tst-t_s),tst-t_s,Rst,tst
cands={'old_baseline':(R0,t0)}
A=np.zeros((3,3))
for p in pairs: A+=np.outer(p['nc'],p['nl'])
U,S,Vt=np.linalg.svd(A); Rn=U@np.diag([1,1,np.linalg.det(U@Vt)])@Vt
cands['normal_align_keep_old_t']=(Rn,t0.copy())
for Rbase,rname in [(R0,'old_R'),(Rn,'normal_align')]:
    Aeq=[]; beq=[]
    for p in pairs: Aeq.append(Rbase@p['nl']); beq.append(p['dl']-p['dc'])
    Aeq=np.vstack(Aeq); beq=np.array(beq)
    for sig in [0.02,0.05,0.10]:
        lam=1/(sig*sig); Aaug=np.vstack([Aeq,math.sqrt(lam)*np.eye(3)]); baug=np.r_[beq,math.sqrt(lam)*t0]
        cands[f'{rname}_t_prior_{int(sig*100)}cm']=(Rbase,np.linalg.lstsq(Aaug,baug,rcond=None)[0])
def residual(p,sigr,sigt):
    R=r_to_R(p[:3]); t=p[3:6]; res=[]
    for q in pairs:
        npred=R@q['nl']; dpred=q['dl']-npred@t
        if npred@q['nc']<0: npred=-npred; dpred=-dpred
        res.extend((2*(npred-q['nc'])).tolist()); res.append(dpred-q['dc'])
    res.extend(((p[:3]-R_to_r(R0))/math.radians(sigr)).tolist()); res.extend(((t-t0)/sigt).tolist())
    return np.array(res)
def lm(sigr,sigt):
    p=np.r_[R_to_r(R0),t0]; lmb=1e-3
    for _ in range(80):
        r=residual(p,sigr,sigt); J=np.zeros((len(r),6)); eps=1e-6
        for i in range(6):
            pp=p.copy(); pm=p.copy(); pp[i]+=eps; pm[i]-=eps; J[:,i]=(residual(pp,sigr,sigt)-residual(pm,sigr,sigt))/(2*eps)
        try: dp=np.linalg.solve(J.T@J+lmb*np.eye(6),-J.T@r)
        except np.linalg.LinAlgError: break
        if np.linalg.norm(dp)<1e-10: break
        pn=p+dp
        if np.mean(residual(pn,sigr,sigt)**2)<np.mean(r**2): p=pn; lmb=max(lmb/3,1e-9)
        else: lmb*=10
    return r_to_R(p[:3]),p[3:6]
for sigr,sigt in [(0.5,0.02),(1,0.03),(2,0.05),(5,0.10)]: cands[f'lm_plane_prior_R{sigr:g}deg_T{int(sigt*100)}cm']=lm(sigr,sigt)
rows=[]
for name,(R,t) in cands.items():
    a,d=plane_metrics(R,t); ce=center_metrics(R,t); drot,dtn,dtv,Rst,tst=delta(R,t)
    rows.append(dict(name=name,R=R,t=t,Rst=Rst,tst=tst,ang_mean=a.mean(),ang_max=a.max(),dist_abs=np.mean(np.abs(d)),dist_max=np.max(np.abs(d)),center_mean=ce.mean(),center_max=ce.max(),drot=drot,dt_norm=dtn,dt_vec=dtv))
rows.sort(key=lambda r:(r['dt_norm']>0.10, r['dist_abs']+0.02*r['ang_mean']+2*r['dt_norm']))
print('\nRESULTS')
for r in rows:
    print(f"{r['name']:<35} plane_dist_mean={r['dist_abs']:.4f}m plane_ang_mean={r['ang_mean']:.3f}deg center_mean={r['center_mean']:.3f}m delta_t={r['dt_norm']:.3f}m delta_R={r['drot']:.3f}deg dt_vec={r['dt_vec']}")
for i,r in enumerate(rows[:6]):
    out=json.loads(json.dumps(cfg)); out['rotation']=[float(x) for x in r['Rst'].reshape(-1)]; out['translation']=[float(x) for x in r['tst']]
    out['_auto_calib_method']=r['name']; out['_source_data']=[str(x) for x in DATA_DIRS]; out['_reference_config']=str(CFG); out['_used_samples']=[p['stem'] for p in pairs]
    out['_metrics']={k:float(r[k]) for k in ['ang_mean','ang_max','dist_abs','dist_max','center_mean','center_max','drot','dt_norm']}; out['_delta_translation']=[float(x) for x in r['dt_vec']]
    path=OUTDIR/f'candidate_{i+1}_{r["name"]}.json'.replace('/','_')
    json.dump(out,open(path,'w'),indent=4); print('wrote',path)
