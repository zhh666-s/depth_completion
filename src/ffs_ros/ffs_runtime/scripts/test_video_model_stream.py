import os
import sys
import time
import argparse
import logging
import yaml

import cv2
import torch
import numpy as np
from omegaconf import OmegaConf


def find_project_root(start_dir: str) -> str:
    """
    从当前脚本目录向上找，直到找到工程根目录。
    判定条件：同时存在 core 目录、Utils.py、weights 目录中的至少两个。
    """
    cur = os.path.abspath(start_dir)
    while True:
        score = 0
        if os.path.isdir(os.path.join(cur, "core")):
            score += 1
        if os.path.isfile(os.path.join(cur, "Utils.py")):
            score += 1
        if os.path.isdir(os.path.join(cur, "weights")):
            score += 1

        if score >= 2:
            return cur

        parent = os.path.dirname(cur)
        if parent == cur:
            raise RuntimeError(
                f"Cannot find project root from start_dir={start_dir}. "
                f"Please make sure this script is placed under your project tree."
            )
        cur = parent


# ===== 路径处理：确保能 import 到工程模块 =====
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
PROJECT_ROOT = find_project_root(SCRIPT_DIR)
sys.path.insert(0, PROJECT_ROOT)

from core.utils.utils import InputPadder
from Utils import AMP_DTYPE, set_logging_format, set_seed, vis_disparity


def configure_torch_runtime(enable_tf32: bool = True, cudnn_benchmark: bool = True):
    """
    Runtime knobs for faster GPU inference without changing model weights.
    """
    if torch.cuda.is_available():
        torch.backends.cuda.matmul.allow_tf32 = bool(enable_tf32)
        torch.backends.cudnn.allow_tf32 = bool(enable_tf32)
    torch.backends.cudnn.benchmark = bool(cudnn_benchmark)
    # PyTorch 2.x: prefer faster matmul kernels on Ampere+.
    try:
        torch.set_float32_matmul_precision("high")
    except Exception:
        pass


def load_model_and_cfg(
    model_dir: str,
    valid_iters: int,
    max_disp: int,
    use_torch_compile: bool = False,
    compile_mode: str = "reduce-overhead",
    use_channels_last: bool = False,
):
    """
    加载模型与配置。
    """
    cfg_file = os.path.join(os.path.dirname(model_dir), "cfg.yaml")
    if not os.path.isfile(cfg_file):
        raise FileNotFoundError(f"cfg.yaml not found next to model: {cfg_file}")

    with open(cfg_file, "r") as ff:
        cfg = yaml.safe_load(ff)

    cfg["model_dir"] = model_dir
    cfg["valid_iters"] = valid_iters
    cfg["max_disp"] = max_disp

    args = OmegaConf.create(cfg)
    logging.info(f"args:\n{args}")

    if not os.path.isfile(model_dir):
        raise FileNotFoundError(f"model file not found: {model_dir}")

    model = torch.load(model_dir, map_location="cpu", weights_only=False)
    model.args.valid_iters = valid_iters
    model.args.max_disp = max_disp
    model = model.cuda().eval()
    if use_channels_last:
        try:
            model = model.to(memory_format=torch.channels_last)
            logging.info("channels_last enabled for model")
        except Exception as e:
            logging.warning(f"channels_last fallback to contiguous due to: {e}")

    if use_torch_compile and hasattr(torch, "compile"):
        try:
            model = torch.compile(model, mode=compile_mode, fullgraph=False)
            logging.info(f"torch.compile enabled, mode={compile_mode}")
        except Exception as e:
            logging.warning(f"torch.compile fallback to eager due to: {e}")

    return model, args


def open_camera(device: str, width: int, height: int, fps: int):
    """
    打开 video0 这种 V4L2 摄像头。
    """
    if not os.path.exists(device):
        raise FileNotFoundError(f"Device not found: {device}")

    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open device: {device}")

    # 尽量减小缓冲
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # 设置采集参数
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    real_fps = cap.get(cv2.CAP_PROP_FPS)

    logging.info(f"Opened camera: {device}")
    logging.info(f"Actual camera size: {real_w} x {real_h}")
    logging.info(f"Actual camera fps : {real_fps:.2f}")
    return cap


def split_side_by_side(frame: np.ndarray):
    """
    把 /dev/video0 读出来的 side-by-side 大图切成左右图。
    """
    h, w = frame.shape[:2]
    mid = w // 2
    left = frame[:, :mid]
    right = frame[:, mid:]
    return left, right


def safe_rgb_to_bgr(img: np.ndarray) -> np.ndarray:
    """
    尽量稳妥地把 RGB 图转成 BGR 图供 cv2.imshow 用。
    """
    if img.ndim == 3 and img.shape[2] == 3:
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img


@torch.inference_mode()
def infer_one_frame(
    model,
    left_bgr: np.ndarray,
    right_bgr: np.ndarray,
    valid_iters: int,
    hiera: int = 0,
    use_channels_last: bool = False,
    build_vis: bool = True,
):
    """
    对一对左右图做一次推理。
    输入是 OpenCV 读出来的 BGR 图。
    返回：
      disp: (H, W) 的视差图
      vis_triplet: BGR 三联图 [left | right | disp]
    """
    # 模型原脚本是按 RGB 图来处理的，这里先转 RGB
    img0 = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2RGB)
    img1 = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2RGB)

    # 防御式处理，确保 3 通道
    if len(img0.shape) == 2:
        img0 = np.tile(img0[..., None], (1, 1, 3))
    if len(img1.shape) == 2:
        img1 = np.tile(img1[..., None], (1, 1, 3))
    img0 = img0[..., :3]
    img1 = img1[..., :3]

    h, w = img0.shape[:2]
    img0_ori = img0.copy()
    img1_ori = img1.copy()

    # HWC -> NCHW
    img0_t = torch.as_tensor(img0).cuda(non_blocking=True).float()[None].permute(0, 3, 1, 2)
    img1_t = torch.as_tensor(img1).cuda(non_blocking=True).float()[None].permute(0, 3, 1, 2)
    if use_channels_last:
        img0_t = img0_t.contiguous(memory_format=torch.channels_last)
        img1_t = img1_t.contiguous(memory_format=torch.channels_last)

    padder = InputPadder(img0_t.shape, divis_by=32, force_square=False)
    img0_t, img1_t = padder.pad(img0_t, img1_t)

    with torch.amp.autocast("cuda", enabled=True, dtype=AMP_DTYPE):
        if not hiera:
            disp = model.forward(
                img0_t,
                img1_t,
                iters=valid_iters,
                test_mode=True,
                optimize_build_volume="pytorch1",
            )
        else:
            disp = model.run_hierachical(
                img0_t,
                img1_t,
                iters=valid_iters,
                test_mode=True,
                small_ratio=0.5,
            )

    disp = padder.unpad(disp.float())
    disp = disp.data.cpu().numpy().reshape(h, w).clip(0, None)

    vis_triplet = None
    if build_vis:
        # 视差可视化，沿用你原来的工具函数
        disp_vis = vis_disparity(
            disp,
            min_val=None,
            max_val=None,
            cmap=None,
            color_map=cv2.COLORMAP_TURBO,
        )

        left_show = safe_rgb_to_bgr(img0_ori)
        right_show = safe_rgb_to_bgr(img1_ori)
        disp_show = safe_rgb_to_bgr(disp_vis)
        vis_triplet = np.concatenate([left_show, right_show, disp_show], axis=1)

    return disp, vis_triplet


def draw_info(img: np.ndarray, text: str, x: int, y: int, color=(0, 255, 255), scale=0.8):
    cv2.putText(
        img,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        2,
        cv2.LINE_AA,
    )


def main():
    parser = argparse.ArgumentParser()

    # ===== 模型相关 =====
    parser.add_argument(
        "--model_dir",
        default=os.path.join(PROJECT_ROOT, "weights", "23-36-37", "model_best_bp2_serialize.pth"),
        type=str,
        help="path to serialized model",
    )
    parser.add_argument("--valid_iters", type=int, default=8)
    parser.add_argument("--max_disp", type=int, default=192)
    parser.add_argument("--hiera", type=int, default=0)

    # ===== 摄像头相关 =====
    parser.add_argument("--device", type=str, default="/dev/video0")
    parser.add_argument("--width", type=int, default=1344)
    parser.add_argument("--height", type=int, default=376)
    parser.add_argument("--fps", type=int, default=30)

    # ===== 显示和推理节奏 =====
    parser.add_argument("--display_scale", type=float, default=1.0)
    parser.add_argument(
        "--skip",
        type=int,
        default=1,
        help="每处理一帧后跳过多少帧。0=每帧都推理，1=处理1帧跳1帧。",
    )
    parser.add_argument(
        "--warmup_grabs",
        type=int,
        default=2,
        help="每轮推理前先 grab 几帧，尽量丢旧帧降低延迟。",
    )
    parser.add_argument(
        "--save_dir",
        type=str,
        default="/tmp",
        help="按 s 保存结果图的目录",
    )

    args = parser.parse_args()

    os.makedirs(args.save_dir, exist_ok=True)

    set_logging_format()
    set_seed(0)
    torch.autograd.set_grad_enabled(False)

    logging.info(f"PROJECT_ROOT = {PROJECT_ROOT}")
    logging.info(f"Using model     = {args.model_dir}")

    model, _ = load_model_and_cfg(
        model_dir=args.model_dir,
        valid_iters=args.valid_iters,
        max_disp=args.max_disp,
    )

    cap = open_camera(
        device=args.device,
        width=args.width,
        height=args.height,
        fps=args.fps,
    )

    window_name = "Stereo Model Stream"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    last_vis = None
    infer_counter = 0
    save_id = 0
    t_last = time.time()

    print("[INFO] Press q to quit")
    print("[INFO] Press s to save current display image")

    while True:
        # 丢掉一些旧帧，降低延迟
        for _ in range(max(args.warmup_grabs, 0)):
            cap.grab()

        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Failed to read frame")
            break

        left, right = split_side_by_side(frame)

        need_infer = (infer_counter % (args.skip + 1) == 0) or (last_vis is None)

        if need_infer:
            try:
                _, vis_triplet = infer_one_frame(
                    model=model,
                    left_bgr=left,
                    right_bgr=right,
                    valid_iters=args.valid_iters,
                    hiera=args.hiera,
                )
                last_vis = vis_triplet
            except Exception as e:
                print(f"[ERROR] Inference failed: {e}")
                break

        infer_counter += 1

        show = last_vis.copy()

        now = time.time()
        disp_fps = 1.0 / max(now - t_last, 1e-6)
        t_last = now

        draw_info(show, f"Display FPS: {disp_fps:.1f}", 20, 35)
        draw_info(show, f"skip={args.skip}", 20, 70, color=(0, 255, 0), scale=0.7)

        if args.display_scale != 1.0:
            show = cv2.resize(
                show,
                None,
                fx=args.display_scale,
                fy=args.display_scale,
                interpolation=cv2.INTER_AREA,
            )

        cv2.imshow(window_name, show)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            out_name = os.path.join(args.save_dir, f"stereo_model_{save_id:04d}.png")
            cv2.imwrite(out_name, show)
            print(f"[INFO] Saved {out_name}")
            save_id += 1

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
