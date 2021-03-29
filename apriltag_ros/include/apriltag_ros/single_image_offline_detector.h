#ifndef SINGLE_IMAGE_OFFLINE_DETECTOR_H
#define SINGLE_IMAGE_OFFLINE_DETECTOR_H
#include <chrono>
#include <memory>
#include <mutex>
#include <queue>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <string>
#include <thread>
#include <visualization_msgs/Marker.h>

#include "apriltag_ros/common_functions.h"
#include "camera.h"
namespace apriltag_ros {
class SingleDetector {
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    std::shared_ptr<TagDetector> tag_detector_;
    bool draw_tag_detections_image_;
    cv_bridge::CvImagePtr cv_image_;
    image_transport::Publisher tag_detections_image_publisher_;
    ros::Publisher tag_detections_publisher_;
    std_msgs::Header header;
    sensor_msgs::ImagePtr msg;
    ros::Time start_time;
    ros::Publisher pub_poses;
    std::string image_file;

  public:
    SingleDetector(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~SingleDetector() = default;
    int readImage();
};

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_SINGLE_IMAGE_DETECTOR_H