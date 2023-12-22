#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Regulator : public rclcpp::Node
{
    public:
    Regulator()
    : Node("regulator") // инициалзация полей
    {
        std::string defualt_robot_ns = "";
        this->declare_parameter("robot_ns", defualt_robot_ns);
        this->get_parameter_or("robot_ns", robot_ns, defualt_robot_ns);

        std::string pose_topic = robot_ns + "/pose";
        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 10, std::bind(&Regulator::pose_cb, this, _1)); 
    }

    private:
    std::string robot_ns;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;


    virtual void logic(geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) = 0;


    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
    {
        logic(pose_msg);
    }
};
