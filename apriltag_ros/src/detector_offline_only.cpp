#include "apriltag_ros/detector_offline_only.h"

#include <opencv2/opencv.hpp>

#include <sys/time.h>

using namespace std;

namespace apriltag_ros {

DetectorOfflineOnly::DetectorOfflineOnly(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh_private) {
  if (nh_private_.hasParam("video_file")) {
    ROS_INFO("nh_private_ has video_file parameter");
  }
  nh_private_.param<std::string>("video_file", video_file, "/home/guoziwei/Documents/output.avi");
  nh_private_.param<std::string>(
      "output_file", output_file, "/home/guoziwei/Documents/outputpose.txt");
  ROS_INFO("Got param: %s", video_file.c_str());
  ROS_INFO("Got param: %s", output_file.c_str());
  tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(nh_private_));
  draw_tag_detections_image_ =
      getAprilTagOption<bool>(nh_private_, "publish_tag_detections_image", false);
  it_ = std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh_));
  tag_detections_publisher_ = nh_.advertise<AprilTagDetectionArray>("tag_detections", 1);
  if (draw_tag_detections_image_) {
    tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
    // tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }
  cap = cv::VideoCapture(video_file);
  start_time = ros::Time::now();
  header.frame_id = "/camera_link";
  header.seq = 0;
}

int DetectorOfflineOnly::ReadImage() {
  if (!cap.isOpened()) {
    std::cerr << "Error opening video file" << std::endl;
    return -1;
  }
  int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
  int frames = cap.get(CV_CAP_PROP_FRAME_COUNT);
  ROS_INFO("frames:%d %d %d", frame_width, frame_height, frames);
  // struct timeval start, end;
  Frame frame;
  start_time = ros::Time(0.0);
  header.stamp = start_time;

  FILE* outfile;
  outfile = std::fopen(output_file.c_str(), "w");
  Camera_Info camera_;
  camera_.CreatefromYaml(
      "/home/guoziwei/catkin_ws/src/apriltag_ros/apriltag_ros/config/camera_info.yaml");
  cv::Mat distortion_image;
  cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << camera_.fx, 0.0, camera_.cx, 0.0, camera_.fy, camera_.cy, 0.0, 0.0,
       1.0);
  cv::Mat distcoeffs = (cv::Mat_<double>(5, 1) << camera_.k1, camera_.k2, 0.0, 0.0, camera_.k3);
  cv::Point2d imagePoints;
  while (1) {
    cap >> distortion_image;
    if (distortion_image.empty())
      break;
    cv::undistort(distortion_image, frame.frame, camera_matrix, distcoeffs);
    cv::cvtColor(frame.frame, frame.frame, CV_BGR2GRAY);
    // cv::imshow("Undistortion", frame.frame);
    // cv::waitKey(3);
    // std::cout<<"get frame"<<std::endl;
    frame.led_on = 0;
    frame.time_stamp = cap.get(CV_CAP_PROP_POS_MSEC) / 1e3;
    // gettimeofday(&start, nullptr);

    try {
      header.stamp = start_time + ros::Duration(frame.time_stamp);
      cv_image_ = boost::make_shared<cv_bridge::CvImage>(header, "bgr8", frame.frame);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return -2;
    }
    if (tag_detector_->detectTagsonly(cv_image_, imagePoints)) {
      std::fprintf(outfile, "%lf %lf %lf\n", frame.time_stamp, imagePoints.x, imagePoints.y);
    }

    // cv::imshow("Red Detections", crroped);
    // cv::waitKey(3);
    // ROS_INFO("seq:%d",header.seq);
    if (draw_tag_detections_image_) {
      tag_detector_->drawDetections(cv_image_);
      tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
    }
    header.seq += 1;
    // gettimeofday(&end, nullptr);
    // double elapsed_seconds = (end.tv_sec - start.tv_sec) * 1e6;
    // elapsed_seconds += (end.tv_usec - start.tv_usec) * 1e-6;
    // std::cout<<"elapsed time: "<<std::setprecision(8)<<elapsed_seconds<<endl;
  }
  cap.release();
  std::fclose(outfile);
  return 0;
}
}  // namespace apriltag_ros