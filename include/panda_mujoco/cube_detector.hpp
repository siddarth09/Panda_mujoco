#pragma once //Header guard 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "panda_mujoco/msg/cube_detector.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv4/opencv2/opencv.hpp>


class CubeDetector : public rclcpp::Node {

    public: 
        CubeDetector(); //Constructor 
    
    private: 

        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);


        void detectCubes(const cv::Mat &rgb);
        double getDepthMedian(int u,int v,int window = 7);

        //RCL interfaces 

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

        rclcpp::Publisher<panda_mujoco::msg::CubeDetector>::SharedPtr cube_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;

        //Camera info

        double fx_{0},fy_{0},cx_{0},cy_{0};
        bool cam_ready_{false};

        cv::Mat depth_image_;

        std::string camera_frame_{"top_rgb_optical_frame"};


};