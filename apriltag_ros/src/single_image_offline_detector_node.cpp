#include "apriltag_ros/single_image_offline_detector.h"

using namespace ros;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "apriltag_ros_single_offline_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    if (nh.hasParam("standalone_tags")) {
        ROS_INFO("nh has video_file parameter");
    } else if (nh_private.hasParam("standalone_tags")) {
        ROS_INFO("nh_p has video_file parameter");
    }
    apriltag_ros::SingleDetector single_offline_detector(nh, nh_private);
    return single_offline_detector.readImage();
}
