#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "panda_mujoco/msg/cube_detector.hpp"


class CubeTF : public rclcpp::Node 
{
    public:
        CubeTF();

    private:
        void cube_callback(
            const panda_mujoco::msg::CubeDetector::SharedPtr msg);
        
            
        rclcpp::Subscription<panda_mujoco::msg::CubeDetector>::SharedPtr sub_;
        rclcpp::Publisher<panda_mujoco::msg::CubeDetector>::SharedPtr pub_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;


};

