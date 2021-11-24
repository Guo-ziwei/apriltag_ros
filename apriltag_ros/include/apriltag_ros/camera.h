#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <string>
namespace apriltag_ros {
enum CameraModel { PinHole, DS };
class Camera_Info {
  public:
    double fx, fy, cx, cy, k1, k2, k3, xi, alpha;
    CameraModel camera_model;
    Camera_Info() = default;
    //   ~Camera_Info();
    bool CreatefromYaml(std::string file_name) {
        cv::FileStorage fs(file_name, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            ROS_WARN("open camera yaml failed");
            return false;
        }
        cv::FileNode n = fs["projection_parameters"];
        fx = static_cast<double>(n["fx"]);
        fy = static_cast<double>(n["fy"]);
        cx = static_cast<double>(n["cx"]);
        cy = static_cast<double>(n["cy"]);
        cv::FileNode m = fs["distortion_parameters"];
        auto model = static_cast<int>(m["model"]);
        camera_model = static_cast<CameraModel>(model);
        switch (camera_model) {
            case 0: {
                k1 = static_cast<double>(m["k1"]);
                k2 = static_cast<double>(m["k2"]);
                k3 = static_cast<double>(m["k3"]);
                break;
            }
            case 1: {
                xi = static_cast<double>(m["xi"]);
                alpha = static_cast<double>(m["alpha"]);
                break;
            }
            default:
                ROS_WARN("camera mode error, mode is %d", camera_model);
                break;
        }
        return true;
    }

    bool doubleSphereUndistort(const std::vector<cv::Point2d>& input_points, std::vector<cv::Point2d>& output_points) {
        for (size_t i = 0; i < input_points.size(); i++) {
            const double x = input_points[i].x;
            const double y = input_points[i].y;
            const double mx = (1 / fx) * (x - cx);
            const double my = (1 / fy) * (y - cy);
            const double r2 = mx * mx + my * my;
            if (alpha > 0.5 && r2 > 1.0 / (2 * alpha - 1)) {
                return false;
            }  // double sphere max projection area

            const double mz = (1 - alpha * alpha * r2) / (alpha * std::sqrt(1 - (2 * alpha - 1) * r2) + 1 - alpha);
            const double mz2 = mz * mz;
            const double k = (mz * xi + std::sqrt(mz2 + (1 - xi * xi) * r2)) / (mz2 + r2);

            const double out_x = k * mx;
            const double out_y = k * my;
            const double z = k * mz - xi;
            if (z < 0) {
                return false;
            }  // 180 deg//TODO sometimes can use too
            double u = fx * (out_x / z) + cx;
            double v = fy * (out_y / z) + cy;
            cv::Point2d uv(u, v);
            output_points.push_back(uv);
        }
        return true;
    }
};
}  // namespace apriltag_ros