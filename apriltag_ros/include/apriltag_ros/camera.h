#pragma once

#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
namespace apriltag_ros
{
class Camera_Info
{
public:
  double fx, fy, cx, cy, k1, k2, k3;
  Camera_Info() = default;
//   ~Camera_Info();
  bool CreatefromYaml(std::string file_name)
  {
    cv::FileStorage fs(file_name, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        ROS_WARN("open camera yaml failed");
        return false;
    }
    cv::FileNode n = fs["projection_parameters"];
    fx = static_cast<double> (n["fx"]);
    fy = static_cast<double> (n["fy"]);
    cx = static_cast<double> (n["cx"]);
    cy = static_cast<double> (n["cy"]);
    cv::FileNode m = fs["distortion_parameters"];
    k1 = static_cast<double> (m["k1"]);
    k2 = static_cast<double> (m["k2"]);
    k3 = static_cast<double> (m["k3"]);
    return true;
  }
  
};
} // namespace apriltag_ros