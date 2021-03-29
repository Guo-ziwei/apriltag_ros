#include "apriltag_ros/single_image_offline_detector.h"

#include <opencv2/opencv.hpp>
#include <ros/console.h>
using namespace std;

namespace apriltag_ros {
SingleDetector::SingleDetector(ros::NodeHandle nh, ros::NodeHandle nh_private) : nh_(nh), nh_private_(nh_private) {
    if (nh_private_.hasParam("image_file")) {
        ROS_INFO("nh_private_ has image file parameter");
    }
    nh_private_.param<std::string>("image_file", image_file, "/home/guoziwei/Documents/image_file.jpg");
    ROS_INFO("Got param: %s", image_file.c_str());
    pub_poses = nh.advertise<visualization_msgs::Marker>("position_xy", 1000);
    tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(nh_private_));
    draw_tag_detections_image_ = getAprilTagOption<bool>(nh_, "publish_tag_detections_image", false);
    it_ = std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh_private_));
    tag_detections_publisher_ = nh_.advertise<AprilTagDetectionArray>("tag_detections", 1);
    if (draw_tag_detections_image_) {
        tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
        // tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
    }
    start_time = ros::Time::now();
    header.frame_id = "/camera_link";
    header.seq = 0;
}

int SingleDetector::readImage() {
    start_time = ros::Time(0.0);
    header.stamp = start_time;
    visualization_msgs::Marker poses;
    Camera_Info camera_;
    camera_.CreatefromYaml("/home/guoziwei/catkin_ws/src/apriltag_ros/apriltag_ros/config/camera_info.yaml");
    cv::Mat camera_matrix =
        (cv::Mat_<double>(3, 3) << camera_.fx, 0.0, camera_.cx, 0.0, camera_.fy, camera_.cy, 0.0, 0.0, 1.0);
    cv::Mat distcoeffs = (cv::Mat_<double>(5, 1) << camera_.k1, camera_.k2, 0.0, 0.0, camera_.k3);
    std::vector<cv::Point2d> imagePoints;
    cv::Mat distortion_image = cv::imread(image_file);
    cv::Mat undistortion_image;
    if (distortion_image.empty())
        return -1;
    cv::undistort(distortion_image, undistortion_image, camera_matrix, distcoeffs);
    cv::cvtColor(undistortion_image, undistortion_image, CV_BGR2GRAY);
    cv_image_ = boost::make_shared<cv_bridge::CvImage>(header, "bgr8", undistortion_image);
    AprilTagDetectionArray tag_pose_array = tag_detector_->detectTags(cv_image_, imagePoints);
    for (unsigned int i = 0; i < tag_pose_array.detections.size(); i++) {
        geometry_msgs::PoseStamped pose;
        geometry_msgs::Point position_marker;
        int id = tag_pose_array.detections[i].id[0];
        pose.pose = tag_pose_array.detections[i].pose.pose.pose;
        pose.header = tag_pose_array.detections[i].pose.header;
        poses.header = tag_pose_array.detections[i].pose.header;
        poses.header.stamp = ros::Time::now();
        position_marker.x = pose.pose.position.x;
        position_marker.y = pose.pose.position.y;
        position_marker.z = pose.pose.position.z;
        poses.points.push_back(position_marker);
        ROS_INFO(
            "%d %lf %lf %lf %lf %lf %lf %lf\n", id, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    }
    pub_poses.publish(poses);
    // ROS_INFO("seq:%d",header.seq);
    if (draw_tag_detections_image_) {
        tag_detector_->drawDetections(cv_image_);
        tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
    }
    return 0;
}
}  // namespace apriltag_ros