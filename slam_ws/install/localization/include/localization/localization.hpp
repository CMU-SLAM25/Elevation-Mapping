#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

class Localization : public rclcpp::Node
{
public:
    Localization();

private:
    void setupCommunications();
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void publishTransform(const Eigen::Matrix4f& T);

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Camera intrinsics
    cv::Mat camera_intrinsics_;
    bool has_camera_info_ = false;

    // ICP
    pcl::PointCloud<pcl::PointNormal>::Ptr map_;
    bool map_initialized_ = false;
    Eigen::Matrix4f T_cam_to_world_ = Eigen::Matrix4f::Identity();
};
