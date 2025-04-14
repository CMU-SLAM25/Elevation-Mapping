#include "localization/localization.hpp"
#include <thread>

Localization::Localization() : Node("localization_node") {
    setupCommunications();
    map_.reset(new pcl::PointCloud<pcl::PointNormal>());
    RCLCPP_INFO(this->get_logger(), "Localization node initialized");
}

void Localization::setupCommunications() {
    cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/camera/depth/color/points", 10,
        std::bind(&Localization::pointCloudCallback, this, std::placeholders::_1));

    camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera/depth/camera_info", 10,
        std::bind(&Localization::cameraInfoCallback, this, std::placeholders::_1));

    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapping/global_point_cloud_map", 10);


    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void Localization::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (has_camera_info_) return;

    camera_intrinsics_ = (cv::Mat_<float>(3, 3) <<
        msg->k[0], msg->k[1], msg->k[2],
        msg->k[3], msg->k[4], msg->k[5],
        msg->k[6], msg->k[7], msg->k[8]);

    has_camera_info_ = true;
    RCLCPP_INFO(this->get_logger(), "Received camera intrinsics");
}

void Localization::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!has_camera_info_) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    // Estimate normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    ne.setKSearch(10);
    ne.compute(*normals);

    pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud(new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*cloud, *normals, *input_cloud);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud, indices);

    if (!map_initialized_) {
        *map_ = *input_cloud;
        map_initialized_ = true;
        return;
    }

    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
    icp.setInputSource(input_cloud);
    icp.setInputTarget(map_);
    icp.setMaximumIterations(20);
    icp.setMaxCorrespondenceDistance(0.1);
    pcl::PointCloud<pcl::PointNormal> aligned;
    icp.align(aligned);

    if (icp.hasConverged()) {
        Eigen::Matrix4f T = icp.getFinalTransformation();
        publishTransform(T);
        Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols);
        std::stringstream ss;
        ss << T.format(fmt);
        RCLCPP_INFO(this->get_logger(), "Estimated T_cam_to_world:\n%s", ss.str().c_str());

        pcl::PointCloud<pcl::PointNormal>::Ptr transformed(new pcl::PointCloud<pcl::PointNormal>());
        pcl::transformPointCloudWithNormals(aligned, *transformed, T);

        pcl::VoxelGrid<pcl::PointNormal> voxel;
        voxel.setInputCloud(transformed);
        voxel.setLeafSize(0.05f, 0.05f, 0.05f);
        voxel.filter(*transformed);

        *map_ += *transformed;

        // DEBUG: Publish global point cloud map
        sensor_msgs::msg::PointCloud2 result_msg;
        pcl::toROSMsg(*map_, result_msg);   
        result_msg.header.frame_id = "map"; 
        pointcloud_publisher_->publish(result_msg);
    } else {
        RCLCPP_WARN(this->get_logger(), "ICP did not converge");
    }
}

void Localization::publishTransform(const Eigen::Matrix4f& T) {
    Eigen::Matrix3f R = T.block<3, 3>(0, 0);
    Eigen::Vector3f t = T.block<3, 1>(0, 3);
    Eigen::Quaternionf q(R);
    q.normalize();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "base_link";

    tf_msg.transform.translation.x = t.x();
    tf_msg.transform.translation.y = t.y();
    tf_msg.transform.translation.z = t.z();
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
} 
