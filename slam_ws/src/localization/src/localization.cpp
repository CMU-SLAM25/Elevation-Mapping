#include "localization/localization.hpp"

Localization::Localization() : Node("localization_node")
{   

    // Setup Communications
    setupCommunications();

    RCLCPP_INFO(this->get_logger(), "ICP localization initialized");
}

void Localization::setupCommunications(){
    // QoS profile
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::SensorDataQoS()).reliability(rclcpp::ReliabilityPolicy::BestEffort);

    // Subscribers
    camera_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/camera/color/image_raw",
        qos_profile,
        std::bind(&Localization::imageCallback, this, std::placeholders::_1));

    // Broadcasters
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void Localization::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg){
    image_thread_ = std::thread(std::bind(&Localization::processImage, this, msg));

    image_thread_.detach();
}

void Localization::publishTransform(const Eigen::Matrix4f& T_cam_to_world)
{
    // Extract rotation and translation
    Eigen::Matrix3f rotation = T_cam_to_world.block<3, 3>(0, 0);
    Eigen::Vector3f translation = T_cam_to_world.block<3, 1>(0, 3);

    // Convert rotation to quaternion
    Eigen::Quaternionf quat(rotation);
    quat.normalize();

    // Create the transform message
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "base_link";

    tf_msg.transform.translation.x = translation.x();
    tf_msg.transform.translation.y = translation.y();
    tf_msg.transform.translation.z = translation.z();

    tf_msg.transform.rotation.x = quat.x();
    tf_msg.transform.rotation.y = quat.y();
    tf_msg.transform.rotation.z = quat.z();
    tf_msg.transform.rotation.w = quat.w();

    // Publish the transform
    tf_broadcaster_->sendTransform(tf_msg);
}

void Localization::processImage(const sensor_msgs::msg::Image::SharedPtr msg){

    // Convert ROS image to OpenCV Mat
    try {
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // DEBUG
        // RCLCPP_INFO(this->get_logger(), "Image converted to OpenCV format. Size: %dx%d",
        //             image.cols, image.rows);

    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    Eigen::Matrix4f T_cam_to_world = Eigen::Matrix4f::Identity();

    // TODO: Use the cv::Mat image and perform ICP. Obtain the camera to world transformation using ICP (T_cam_to_world in HW4) for each timestep
    // Populate the 4 x 4 extrinsic matrix of the transformation

    // // TESTING
    // T_cam_to_world(0, 3) = 4.0f;  // X
    // T_cam_to_world(1, 3) = 4.0f;  // Y
    
    // DEBUG
    // Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols);
    // std::stringstream ss;
    // ss << T_cam_to_world.format(fmt);
    // RCLCPP_INFO(this->get_logger(), "Estimated T_cam_to_world:\n%s", ss.str().c_str());

    // Publish estimated ICP transform to transform publisher
    publishTransform(T_cam_to_world);
}