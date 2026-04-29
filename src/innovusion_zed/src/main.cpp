#include "innovusion_zed/ros_node.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "innovusion_zed_node");
    
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string lidar_topic;
    std::string cam_topic;
    std::string frame_id;
    std::string param_path;
    std::string publish_topic;

    private_nh.param("lidar_topic", lidar_topic, std::string("/iv_points"));
    private_nh.param("cam_topic", cam_topic, std::string("/camera/left/image_raw"));
    private_nh.param("frame_id", frame_id, std::string("innovusion"));
    private_nh.param("param_path", param_path,
                     std::string("src/innovusion_zed/config/camera_left.json"));
    private_nh.param("publish_topic", publish_topic, std::string("/processed_points"));

    RosNode node(lidar_topic, cam_topic, frame_id, param_path, publish_topic);
    node.run();

    return 0;
}
