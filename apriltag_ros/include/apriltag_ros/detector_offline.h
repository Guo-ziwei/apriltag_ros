#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <queue>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Header.h>
#include <string>
#include <thread>
#include <visualization_msgs/Marker.h>

#include "apriltag_ros/common_functions.h"
#include "camera.h"
#include "sensor_msgs/CompressedImage.h"

namespace apriltag_ros {

struct Frame {
    cv::Mat frame;
    double time_stamp;
    int led_on;
};

class DetectorOffline {
  public:
    DetectorOffline(ros::NodeHandle nh, ros::NodeHandle nh_private);
    int readImage();
    int readCompressedImageFromBag();

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
    std::string video_file;
    std::string output_file;
    std::string bag_file;
    cv::VideoCapture cap;
    ros::Time start_time;
    ros::Publisher pub_poses;
    rosbag::Bag bag;
};

}  // namespace apriltag_ros