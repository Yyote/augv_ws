#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "augv_navigation_msgs/msg/position.hpp"
#include "euler_angles_lib/euler_angles.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


class GroundRegulator;
class PID;




class PID
{
    public:
    PID(float kp, float ki, float kd)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }


    /**
    * @brief Main logic function. Use it to use regulation
    * @param dx the regulated signal error (goal - real)
    * @param dt time difference between current and previous measurements
    */
    float pid(float dx, float dt)
    {
        float signal = kp * dx;

        if (error_vector.size() > 1)
        {
            integral += dx * dt;
            derivative = -(dx - error_vector.at(error_vector.size() - 1)) / dt;
            signal = kp * dx + ki * integral + kd * derivative;

            if (error_vector.size() > 2) error_vector.erase(error_vector.begin()); // as there doesn't need to be more than two elements, delete the old ones
        }

        // _status(dx);
        error_vector.push_back(dx);

        return signal;
    }


    /**
    * @brief Equalizes the integral and derivative calculated before to zero and clears the error vector
    */
    float clear()
    {
        integral = 0;
        derivative = 0;
        error_vector.clear();
    }


    private:
    float kp;
    float ki;
    float kd;

    float integral = 0;
    float derivative = 0;

    std::vector<float> error_vector;


    void _status(float signal)
    {
        std::cout   << "ki = " << ki << "\n"
                    << "kp = " << kp << "\n"
                    << "kd = " << kd << "\n"
                    << "integral = " << integral << "\n"
                    << "derivative = " << derivative << "\n"
                    << "signal = " << signal << "\n"
                    << "error_vector.size() = " << error_vector.size() << "\n";
    }
};

/**
* @brief Базовый класс для реализации регуляторов
* @param id Идентификационный номер ноды, показывающий, какому роботу регулятор принадлежит. По-умолчанию - 1
*/
class GroundRegulator : public rclcpp::Node
{
    public:
    GroundRegulator()
    : Node("regulator"), x_regulator(0.8, 0.015, 0.5), y_regulator(0.8, 0.015, 0.5), z_regulator(1, 0.02, 0.5), yaw_regulator(0.45, 0.002, 0.2) // инициалзация полей
    {
        int default_id = 1;
        int id = default_id;
        this->declare_parameter("id", default_id);
        this->get_parameter_or("id", id, default_id);

        id_ = id;

        std::string defualt_robot_ns = "/robot" + std::to_string(id);
        this->declare_parameter("robot_ns", defualt_robot_ns);
        this->get_parameter_or("robot_ns", robot_ns, defualt_robot_ns);

        std::string pose_topic = robot_ns + "/pose";
        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 10, std::bind(&GroundRegulator::pose_cb, this, _1)); 
        std::string goal_topic = robot_ns + "/goal";
        goal_sub = this->create_subscription<augv_navigation_msgs::msg::Position>(goal_topic, 10, std::bind(&GroundRegulator::goal_cb, this, _1)); 
        std::string cmd_vel_topic = robot_ns + "/cmd_vel";
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic, 10);
        arrow_pub = this->create_publisher<visualization_msgs::msg::Marker>("/robot" + std::to_string(this->id_) + "/goal_arrow", 10);
    }

    private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub; // regulators.hpp::GroundRegulator
    rclcpp::Subscription<augv_navigation_msgs::msg::Position>::SharedPtr goal_sub; // regulators.hpp::GroundRegulator


    virtual void logic(float yaw_singal, float x_signal, float y_signal, float z_signal) = 0;


    virtual void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
    {
        rclcpp::Time now = this->get_clock()->now();
        EulerAngles ea;
        // curr_orientation = ea.yaw();
        if (got_goal_at_least_once)
        {
            ea.get_RPY_from_msg_quaternion(pose_msg->pose.orientation);
            curr_orientation = ea.yaw();
            float dyaw = current_goal.course - ea.yaw();
            float dx = current_goal.position.x - pose_msg->pose.position.x;
            float dy = current_goal.position.y - pose_msg->pose.position.y;
            float dz = current_goal.position.z - pose_msg->pose.position.z;

            auto xy = ea.rotate_vector_by_angle(dx, dy, -curr_orientation);
            dx = xy.at(0);
            dy = xy.at(1);

            // float dyaw = atan2(dy, dx);
            dyaw = atan2f(dy, dx);
            // dyaw = ea.normalize_angle(dyaw);
            RCLCPP_WARN_STREAM(this->get_logger(), "DYAW = " << dyaw);

            // auto xy = ea.rotate_vector_by_angle(dx, dy, curr_orientation);
            // dx = xy[0];
            // dy = xy[1];

            float dt = (now - last_time).nanoseconds() / 1e9;

            ea.setRPY(0, 0, dyaw);

            visualization_msgs::msg::Marker dmarker;
            dmarker.header.frame_id = "robot" + std::to_string(this->id_) + "/base_link";
            dmarker.header.stamp = this->get_clock()->now();
            dmarker.id = 0;
            dmarker.type = visualization_msgs::msg::Marker::ARROW;
            dmarker.action = visualization_msgs::msg::Marker::ADD;
            dmarker.pose.position.x = 0;
            dmarker.pose.position.y = 0;
            dmarker.pose.position.z = 0;
            dmarker.pose.orientation = ea.get_current_msg_quaternion();
            dmarker.scale.x = 1;
            dmarker.scale.y = 0.1;
            dmarker.scale.z = 0.1;
            dmarker.color.a = 1.0; // Don't forget to set the alpha!
            dmarker.color.r = 0.0;
            dmarker.color.g = 1.0;
            dmarker.color.b = 0.0;
            arrow_pub->publish(dmarker);

            // std::cout   << "dx = " << dx << "\n"
            //             << "dy = " << dy << "\n"
            //             << "dz = " << dz << "\n"
            //             << "dyaw = " << dyaw << "\n";

            float x_sig = x_regulator.pid(dx, dt);
            float y_sig = y_regulator.pid(dy, dt);
            float z_sig = z_regulator.pid(dz, dt);
            float yaw_sig = yaw_regulator.pid(dyaw, dt);
            logic(yaw_sig, x_sig, y_sig, z_sig);
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

        x_regulator.clear();
        y_regulator.clear();
        z_regulator.clear();
        yaw_regulator.clear();
    }

    protected:
    rclcpp::Time last_time; // regulators.hpp::GroundRegulator
    int  id_; // regulators.hpp::GroundRegulator
    std::string robot_ns; // regulators.hpp::GroundRegulator
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub; // regulators.hpp::GroundRegulator
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arrow_pub;
    augv_navigation_msgs::msg::Position current_goal; // regulators.hpp::GroundRegulator
    bool got_goal_at_least_once = false; // regulators.hpp::GroundRegulator
    std::vector<float> rot_errs; // regulators.hpp::GroundRegulator
    std::vector<float> x_errs; // regulators.hpp::GroundRegulator
    std::vector<float> y_errs; // regulators.hpp::GroundRegulator

    PID x_regulator;
    PID y_regulator;
    PID z_regulator;
    PID yaw_regulator;

    float curr_orientation;
};

