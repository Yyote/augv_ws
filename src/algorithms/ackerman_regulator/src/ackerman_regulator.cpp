#include "regulators_lib/regulators.hpp"
#include "euler_angles_lib/euler_angles.hpp"
#include "augv_navigation_msgs/msg/position.hpp"


class AckermanRegulator : public GroundRegulator
{
    public:
    AckermanRegulator()
    {
        rviz_goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&AckermanRegulator::goal_sub_cb, this, std::placeholders::_1));
        potential_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(this->robot_ns + "/potential_fields/force", 10, std::bind(&AckermanRegulator::potential_cb, this, std::placeholders::_1));
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
            twist.twist.linear.x = x_signal + field_vel.twist.linear.x;
            twist.twist.angular.z = yaw_singal + field_vel.twist.angular.z / 2;
            if (twist.twist.linear.x < 0) twist.twist.angular.z = twist.twist.angular.z * -1;
            cmd_vel_pub->publish(twist);
            // RCLCPP_INFO_STREAM(this->get_logger(), "twist.twist.linear.x = " << twist.twist.linear.x << "\n" << "twist.twist.angular.z = " << twist.twist.angular.z);
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
    rclcpp::spin(std::make_shared<AckermanRegulator>());
    return 0;
}