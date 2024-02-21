#include "regulators_lib/regulators.hpp"
#include "euler_angles_lib/euler_angles.hpp"
#include "augv_navigation_msgs/msg/position.hpp"


class TrackedRegulator : public GroundRegulator
{
    public:
    TrackedRegulator()
    {
        rviz_goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(this->robot_ns + "/goal_pose", 10, std::bind(&TrackedRegulator::goal_sub_cb, this, std::placeholders::_1));
        potential_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(this->robot_ns + "/potential_fields/force", 10, std::bind(&TrackedRegulator::potential_cb, this, std::placeholders::_1));
        goal_pub = this->create_publisher<augv_navigation_msgs::msg::Position>("/robot" + std::to_string(this->id_) + "/goal", 10);
    }

    private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_goal_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr potential_sub;
    rclcpp::Publisher<augv_navigation_msgs::msg::Position>::SharedPtr goal_pub;
    geometry_msgs::msg::TwistStamped field_vel;

    void logic(float yaw_singal, float x_signal, float y_signal, float z_signal) override
    {
            geometry_msgs::msg::TwistStamped twist;
            twist.header.stamp = this->get_clock()->now();
            // twist.twist.linear.x = x_signal + field_vel.twist.linear.x / 1;
            twist.twist.linear.x = x_signal + field_vel.twist.linear.x / 10;
            // twist.twist.linear.x = 0;
            // twist.twist.angular.z = yaw_singal + field_vel.twist.angular.z / 2.5;
            twist.twist.angular.z = yaw_singal + field_vel.twist.angular.z / 14.5;
            // twist.twist.angular.z = 0;
            // if (twist.twist.linear.x < 0) twist.twist.angular.z = twist.twist.angular.z * -1;
            cmd_vel_pub->publish(twist);
    }


    void goal_sub_cb(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
    {
        EulerAngles eu;
        eu.get_RPY_from_msg_quaternion(goal->pose.orientation);
        augv_navigation_msgs::msg::Position goal_pose;
        goal_pose.course = eu.yaw();
        goal_pose.position.x = goal->pose.position.x;
        goal_pose.position.y = goal->pose.position.y;
        goal_pose.position.z = goal->pose.position.z;

        goal_pub->publish(goal_pose);
    }


    void potential_cb(const geometry_msgs::msg::TwistStamped::SharedPtr pf_msg)
    {
        field_vel = *pf_msg;
    }
};




int main (int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackedRegulator>());
    return 0;
}