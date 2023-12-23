#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "augv_navigation_msgs/msg/position.hpp"
#include "euler_angles_lib/euler_angles.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


class Regulator : public rclcpp::Node
{
    public:
    Regulator()
    : Node("regulator") // инициалзация полей
    {
        std::string defualt_robot_ns = "/robot1";
        this->declare_parameter("robot_ns", defualt_robot_ns);
        this->get_parameter_or("robot_ns", robot_ns, defualt_robot_ns);

        std::string pose_topic = robot_ns + "/pose";
        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 10, std::bind(&Regulator::pose_cb, this, _1)); 
        std::string goal_topic = robot_ns + "/goal";
        goal_sub = this->create_subscription<augv_navigation_msgs::msg::Position>(goal_topic, 10, std::bind(&Regulator::goal_cb, this, _1)); 
        std::string cmd_vel_topic = robot_ns + "/cmd_vel";
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic, 10);
    }

    private:
    rclcpp::Time last_time;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::Subscription<augv_navigation_msgs::msg::Position>::SharedPtr goal_sub;


    virtual void logic(float rotation_diff, float dx, float dy, float dz, float dt, float curr_orientation) = 0;


    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
    {
        rclcpp::Time now = this->get_clock()->now();
        EulerAngles ea;
        curr_orientation = ea.yaw();
        if (got_goal_at_least_once)
        {
            ea.get_RPY_from_msg_quaternion(pose_msg->pose.orientation);
            float rotation_diff = current_goal.course - ea.yaw();
            float dx = current_goal.position.x - pose_msg->pose.position.x;
            float dy = current_goal.position.y - pose_msg->pose.position.y;
            float dz = current_goal.position.z - pose_msg->pose.position.z;

            // auto xy = ea.rotate_vector_by_angle(dx, dy, curr_orientation);
            // dx = xy[0];
            // dy = xy[1];

            float dt = (now - last_time).nanoseconds() / 1e9;
            logic(rotation_diff, dx, dy, dz, dt, ea.yaw());
        }
        last_time = now;
    }


    void goal_cb(const augv_navigation_msgs::msg::Position::SharedPtr goal_msg)
    {
        current_goal = *goal_msg;
        EulerAngles eu;
        // auto xy = eu.rotate_vector_by_angle(current_goal.position.x, current_goal.position.y, -curr_orientation);
        // current_goal.position.x = xy.at(0);
        // current_goal.position.y = xy.at(1);

        got_goal_at_least_once = true;
        rot_integr = 0;
        x_integr = 0;
        y_integr = 0;

        rot_diff = 0;
        x_diff = 0;
        y_diff = 0;

        rot_errs.clear();
        x_errs.clear();
        y_errs.clear();
    }

    protected:
    std::string robot_ns;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub;
    augv_navigation_msgs::msg::Position current_goal;
    bool got_goal_at_least_once = false;
    std::vector<float> rot_errs;
    std::vector<float> x_errs;
    std::vector<float> y_errs;

    float rot_integr;
    float x_integr;
    float y_integr;

    float rot_diff;
    float x_diff;
    float y_diff;

    float kp;
    float kd;
    float ki;

    float curr_orientation;
};
