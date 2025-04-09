#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <Eigen/Dense>

class Localization : public rclcpp::Node
{
    private:
        // Variables & pointers

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber_;

        // Broadcasters
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // Threads
        std::thread image_thread_;

        // Functions
        void setupCommunications();
        void imageCallback(const sensor_msgs::msg::Image::SharedPtr );
        void processImage(const sensor_msgs::msg::Image::SharedPtr );
        void publishTransform(const Eigen::Matrix4f& );

    public:
        // Constructor and destructor
        Localization();
        ~Localization(){};
};  

#endif