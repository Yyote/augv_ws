#include "rclcpp/rclcpp.hpp"
#include <cstdlib>
#include <cmath>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using std::placeholders::_1;


nav_msgs::msg::Odometry global_odom;

bool trajectory_is_sent = 0;
bool timer_set = 0;
geometry_msgs::msg::PoseStamped bufferized_goal;
geometry_msgs::msg::PoseStamped curr_pose;




// Класс для работы с Эйлеровыми углами и кватернионами
class EulerAngles {
    public:
    EulerAngles()
    {
        roll = 0;
        pitch = 0;
        yaw = 0;
    }


    double roll;
    double pitch;
    double yaw;


    // Функция для установки Эйлеровых углов по переданным параметрам
    void setRPY(float new_roll, float new_pitch, float new_yaw)
    {
        roll = new_roll;
        pitch = new_pitch;
        yaw = new_yaw;
    }


    // Функция для передачи Эйлеровых углов в вектор вращения
    void setRPY_of_quaternion(tf2::Quaternion &q)
    {
        q.setRPY(roll, pitch, yaw);
    }


    // Функция для получения Эйлеровых углов из вектора вращения 
    void get_RPY_from_quaternion(tf2::Quaternion q)
    {
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
    }


    // Функция для получения Эйлеровых углов из вектора вращения заданного geometry_msgs::Quaternion
    void get_RPY_from_msg_quaternion(geometry_msgs::msg::Quaternion q)
    {
        tf2::Quaternion q_tf;
        tf2::convert(q, q_tf);

        get_RPY_from_quaternion(q_tf);
    }
};



class FieldsNode : public rclcpp::Node
{
    public:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr field_vel_pub;


    FieldsNode() : Node("potential_fields_node")
    {

    }


    void init(int id, double rmax1, double rmax2, double rmax3, double k1, double k2, double k3)
    {
        this->k1 = k1;
        this->k2 = k2;
        this->k3 = k3;
        this->rmax1 = rmax1;
        this->rmax2 = rmax2;
        this->rmax3 = rmax3;
        
        std::string local_namespace = "/robot" + std::to_string(id);
        std::string topic = local_namespace + "/scan";
        RCLCPP_INFO_STREAM(this->get_logger(), "Topic = " << topic);
        laser_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(topic, 100, std::bind(&FieldsNode::laser_scan_cb, this, _1));
        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(local_namespace + "/pose", 100, std::bind(&FieldsNode::pose_sub_cb, this, _1));
        field_vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(local_namespace + "/potential_fields/force", 10);
    }


    private:
    double k1 = 0.004 * 2.5;
    double k2 = 0.007 * 3;
    double k3 = 0.010 * 3.8;
    double rmax1 = 3.8;
    double rmax2 = 3;
    double rmax3 = 1.5;



    void laser_scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        sensor_msgs::msg::PointCloud pc;

        geometry_msgs::msg::TwistStamped twist;
        twist.header.frame_id = scan->header.frame_id;
        twist.header.stamp = this->get_clock()->now();

        twist.twist.linear.x = 0;
        twist.twist.linear.y = 0;

        double local_koeff = 360.0 / scan->ranges.size();

        for (int i = 0; i < scan->ranges.size(); ++i)
        {
            if (scan->ranges.at(i) > 0 and !isnan(scan->ranges.at(i)))
            {
                geometry_msgs::msg::Point32 point;

                double alpha_0 = 0;

                if (std::isfinite(scan->ranges.at(i)))
                {
                    if (scan->ranges.at(i) < rmax1 and scan->ranges.at(i) >= rmax2)
                    {
                        double alpha_scan = scan->angle_increment * i + alpha_0;
                        point.x = scan->ranges.at(i) * cos(alpha_scan);
                        point.y = scan->ranges.at(i) * sin(alpha_scan);

                        double dr = scan->ranges.at(i);

                        twist.twist.linear.x += (1.0 / 2.0) * k1 * local_koeff * pow(1.0 / dr - 1.0 / rmax1, 2) * cos(alpha_scan);
                        twist.twist.linear.y += (1.0 / 2.0) * k1 * local_koeff * pow(1.0 / dr - 1.0 / rmax1, 2) * sin(alpha_scan);
                    }
                    else if (scan->ranges.at(i) < rmax2 and scan->ranges.at(i) >= rmax3)
                    {
                        double alpha_scan = scan->angle_increment * i + alpha_0;
                        point.x = scan->ranges.at(i) * cos(alpha_scan);
                        point.y = scan->ranges.at(i) * sin(alpha_scan);

                        double dr = scan->ranges.at(i);

                        twist.twist.linear.x += (1.0 / 2.0) * k2 * local_koeff * pow(1.0 / dr - 1.0 / rmax1, 2) * cos(alpha_scan);
                        twist.twist.linear.y += (1.0 / 2.0) * k2 * local_koeff * pow(1.0 / dr - 1.0 / rmax1, 2) * sin(alpha_scan);
                    }
                    else if (scan->ranges.at(i) < rmax3)
                    {
                        double alpha_scan = scan->angle_increment * i + alpha_0;
                        point.x = scan->ranges.at(i) * cos(alpha_scan);
                        point.y = scan->ranges.at(i) * sin(alpha_scan);
                        
                        double dr = scan->ranges.at(i);

                        twist.twist.linear.x += (1.0 / 2.0) * k3 * local_koeff * pow(1.0 / dr - 1.0 / rmax1, 2) * cos(alpha_scan);
                        twist.twist.linear.y += (1.0 / 2.0) * k3 * local_koeff * pow(1.0 / dr - 1.0 / rmax1, 2) * sin(alpha_scan);
                    }
                }
                else
                {
                    double alpha = scan->angle_increment * i + alpha_0;
                    point.x = 10 * cos(alpha);;
                    point.y = 10 * sin(alpha);;
                }
                // ROS_INFO_STREAM("Point x = " << point.x);
                // ROS_INFO_STREAM("Point y = " << point.y);
                pc.points.push_back(point);
            }
        }

        EulerAngles angles;
        angles.get_RPY_from_msg_quaternion(curr_pose.pose.orientation);

        double tmpx = twist.twist.linear.x;
        double tmpy = twist.twist.linear.y;

        // twist.twist.linear.x = tmpx * cos(angles.yaw) - tmpy * sin(angles.yaw);
        // twist.twist.linear.y = 0;
        twist.twist.angular.z = tmpy;

        // RCLCPP_INFO_STREAM(this->get_logger(), "twist.twist.linear.x: " << twist.twist.linear.x << "twist.twist.linear.y: " << twist.twist.linear.y << "");

        field_vel_pub->publish(twist);
    }



    void pose_sub_cb(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
    {
        curr_pose = *pose;
    }
};
















int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FieldsNode>();

    node->declare_parameter("id", 1);

    node->declare_parameter("rmax1", 1.0);
    node->declare_parameter("rmax2", 1.0);
    node->declare_parameter("rmax3", 1.0);
    node->declare_parameter("k1", 1.0);
    node->declare_parameter("k2", 1.0);
    node->declare_parameter("k3", 1.0);

    double k1 = 0.01;
    double k2 = 0.021;
    double k3 = 0.038;
    double rmax1 = 3.0;
    double rmax2 = 1.0;
    double rmax3 = 0.5;

    int id = 0;
    int default_id = 1;

    node->get_parameter_or("id", id, default_id);
    node->get_parameter_or("rmax1", rmax1, rmax1);
    node->get_parameter_or("rmax2", rmax2, rmax2);
    node->get_parameter_or("rmax3", rmax3, rmax3);
    node->get_parameter_or("k1", k1, k1);
    node->get_parameter_or("k2", k2, k2);
    node->get_parameter_or("k3", k3, k3);

    node->init(id, rmax1, rmax2, rmax3, k1, k2, k3);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}