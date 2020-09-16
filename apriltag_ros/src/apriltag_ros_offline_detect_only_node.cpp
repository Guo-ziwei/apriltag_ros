#include "apriltag_ros/detector_offline_only.h"

using namespace ros;

int main(int argc, char** argv) {
  ros::init(argc, argv, "apriltag_ros_offline_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  if (nh.hasParam("standalone_tags")) {
    ROS_INFO("nh has video_file parameter");
  } else if (nh_private.hasParam("standalone_tags")) {
    ROS_INFO("nh_p has video_file parameter");
  }
  apriltag_ros::DetectorOfflineOnly offline_detector(nh, nh_private);
  offline_detector.ReadImage();
  // off_line_detector.t1.join();
  return 0;
}