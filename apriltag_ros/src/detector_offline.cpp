#include "apriltag_ros/detector_offline.h"

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <sys/time.h>

using namespace std;

namespace apriltag_ros {
/*
 * Z-Y-X Eular angles
 */
template <typename T>
Eigen::Matrix<T, 3, 1> Quaternion2Euler(const Eigen::Quaternion<T>& q) {
    const double Epsilon = 0.0009765625f;
    const double threshold = 0.5 - Epsilon;
    Eigen::Matrix<T, 3, 1> eular(0, 0, 0);
    double singular = static_cast<double>(q.w() * q.y() - q.x() * q.z());
    if (singular < -threshold || singular > threshold) {
        if (singular < -0.00001) {
            eular(2) = static_cast<T>(2 * atan2(q.x(), q.w()));
            eular(1) = static_cast<T>(-M_PI / 2.0);
        } else if (singular > 0.000001) {
            eular(2) = static_cast<T>(-2 * atan2(q.x(), q.w()));
            eular(1) = static_cast<T>(M_PI / 2.0);
        }
        eular(0) = 0.0;
    } else {
        eular(0) = static_cast<T>(
            atan2(2 * (q.y() * q.z() + q.w() * q.x()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z()));
        eular(1) = static_cast<T>(std::asin(-2 * (q.x() * q.z() - q.w() * q.y())));
        eular(2) = static_cast<T>(
            atan2(2 * (q.x() * q.y() + q.w() * q.z()), q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z()));
    }
    return eular;
}

DetectorOffline::DetectorOffline(ros::NodeHandle nh, ros::NodeHandle nh_private) : nh_(nh), nh_private_(nh_private) {
    if (nh_private_.hasParam("video_file")) {
        ROS_INFO("nh_private_ has video_file parameter");
    }
    nh_private_.param<std::string>("video_file", video_file, "");
    nh_private_.param<std::string>("bag_file", bag_file, "");
    nh_private_.param<std::string>("output_file", output_file, "/home/guoziwei/Documents/outputpose.txt");
    if (video_file.empty()) {
        ROS_WARN("Video name is not passed\n");
    } else {
        ROS_INFO("Got param: %s", video_file.c_str());
        cap = cv::VideoCapture(video_file);
    }
    if (bag_file.empty()) {
        ROS_WARN("bagfile name is not passed\n");
    } else {
        ROS_INFO("Got param: %s", bag_file.c_str());
    }
    ROS_INFO("Got param: %s", output_file.c_str());
    pub_poses = nh.advertise<visualization_msgs::Marker>("position_xy", 1000);
    tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(nh_private_));
    draw_tag_detections_image_ = getAprilTagOption<bool>(nh_private_, "publish_tag_detections_image", false);
    it_ = std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh_));
    tag_detections_publisher_ = nh_.advertise<AprilTagDetectionArray>("tag_detections", 1);
    if (draw_tag_detections_image_) {
        tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
        // tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
    }
    start_time = ros::Time::now();
    header.frame_id = "/camera_link";
    header.seq = 0;
}

int DetectorOffline::readImage() {
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

    visualization_msgs::Marker poses;
    poses.header = header;
    poses.ns = "tag_position";
    poses.type = visualization_msgs::Marker::SPHERE_LIST;
    poses.action = visualization_msgs::Marker::ADD;
    poses.pose.orientation.w = 1.0;
    poses.lifetime = ros::Duration();
    poses.id = 0;
    poses.scale.x = 0.05;
    poses.scale.y = 0.05;
    poses.scale.z = 0.05;
    poses.color.r = 1.0;
    poses.color.a = 1.0;
    FILE* outfile;
    outfile = std::fopen(output_file.c_str(), "w");
    Camera_Info camera_;
    camera_.CreatefromYaml("/home/guoziwei/catkin_ws/src/apriltag_ros/apriltag_ros/config/camera_info.yaml");
    cv::Mat distortion_image;
    cv::Mat camera_matrix =
        (cv::Mat_<double>(3, 3) << camera_.fx, 0.0, camera_.cx, 0.0, camera_.fy, camera_.cy, 0.0, 0.0, 1.0);
    cv::Mat distcoeffs = (cv::Mat_<double>(5, 1) << camera_.k1, camera_.k2, 0.0, 0.0, camera_.k3);
    std::vector<cv::Point2d> imagePoints;
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
        AprilTagDetectionArray tag_pose_array = tag_detector_->detectTags(cv_image_, imagePoints);
        // detect led
        // cv::Rect roi(imagePoints[2].x - 60, imagePoints[2].y - 60, 120, 120);
        // cv::Mat crroped = distortion_image(roi);
        // cv::Mat1b mask1, mask2;
        // cv::cvtColor(crroped, crroped, cv::COLOR_BGR2HSV);
        // cv::inRange(crroped, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), mask1);
        // cv::inRange(crroped, cv::Scalar(170, 70, 50), cv::Scalar(180, 250, 255), mask2);
        // cv::Mat1b mask = mask1 | mask2;
        // cv::Mat_<uchar>::iterator it = mask.begin();
        // cv::Mat_<uchar>::iterator it_end = mask.end();
        // for (; it != it_end; it++) {
        //     if (*it == 255)
        //         frame.led_on = 1;
        // }

        // cv::imshow("Red Detections", crroped);
        // cv::waitKey(3);
        for (unsigned int i = 0; i < tag_pose_array.detections.size(); i++) {
            geometry_msgs::PoseStamped pose;
            geometry_msgs::Point position_marker;
            pose.pose = tag_pose_array.detections[i].pose.pose.pose;
            pose.header = tag_pose_array.detections[i].pose.header;
            poses.header = tag_pose_array.detections[i].pose.header;
            poses.header.stamp = ros::Time::now();
            position_marker.x = pose.pose.position.x;
            position_marker.y = pose.pose.position.y;
            position_marker.z = pose.pose.position.z;
            poses.points.push_back(position_marker);
            std::fprintf(
                outfile, "%lf %lf %lf %lf %lf %lf %lf %lf %d\n", pose.header.stamp.toSec(), pose.pose.position.x,
                pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y,
                pose.pose.orientation.z, pose.pose.orientation.w, frame.led_on);
        }
        pub_poses.publish(poses);
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

int DetectorOffline::readCompressedImageFromBag() {
    FILE* outfile;
    outfile = std::fopen(output_file.c_str(), "w");
    std::vector<cv::Point2d> imagePoints;
    bag.open(bag_file);
    std::vector<std::string> topics;
    topics.push_back(std::string("/ninebot/fisheye_left/compressed"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    for (rosbag::MessageInstance const compress_image : view) {
        sensor_msgs::CompressedImageConstPtr i = compress_image.instantiate<sensor_msgs::CompressedImage>();
        header = i->header;
        cv_bridge::CvImagePtr compressed_ptr = cv_bridge::toCvCopy(i, sensor_msgs::image_encodings::RGB8);
        try {
            cv_image_ = boost::make_shared<cv_bridge::CvImage>(header, "rgb8", compressed_ptr->image);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return -2;
        }
        AprilTagDetectionArray tag_pose_array = tag_detector_->detectTags(cv_image_, imagePoints);
        for (unsigned int i = 0; i < tag_pose_array.detections.size(); i++) {
            geometry_msgs::PoseStamped pose;
            geometry_msgs::Point position_marker;
            pose.pose = tag_pose_array.detections[i].pose.pose.pose;
            pose.header = tag_pose_array.detections[i].pose.header;
            // poses.header = tag_pose_array.detections[i].pose.header;
            // poses.header.stamp = ros::Time::now();
            position_marker.x = pose.pose.position.x;
            position_marker.y = pose.pose.position.y;
            position_marker.z = pose.pose.position.z;
            // poses.points.push_back(position_marker);
            Eigen::Vector3d position(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            Eigen::Quaterniond attitude(
                pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
            attitude = attitude.conjugate();
            position = attitude * position;
            const auto eular_vec = Quaternion2Euler(attitude);
            std::fprintf(
                outfile, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", pose.header.stamp.toSec(), position[0],
                position[1], position[2], attitude.w(), attitude.x(), attitude.y(), attitude.z(), eular_vec[0], eular_vec[1],
                eular_vec[2]);
        }
        if (draw_tag_detections_image_) {
            tag_detector_->drawDetections(cv_image_);
            tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
        }
        // cv::imshow("image", compressed_ptr->image);
        // cv::waitKey(1);
    }
    std::fclose(outfile);
    return 0;
}
}  // namespace apriltag_ros