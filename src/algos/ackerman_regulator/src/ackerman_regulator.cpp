#include "regulators_lib/regulators.hpp"
#include "euler_angles_lib/euler_angles.hpp"
#include "augv_navigation_msgs/msg/position.hpp"
#include "visualization_msgs/msg/marker.hpp"

class AckermanRegulator : public Regulator
{
    public:
    AckermanRegulator()
    {
        this->kp = 1;
        this->ki = 0.02;
        this->kd = 0.5;

        rviz_goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&AckermanRegulator::goal_sub_cb, this, std::placeholders::_1));
        potential_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(this->robot_ns + "/potential_fields/force", 10, std::bind(&AckermanRegulator::potential_cb, this, std::placeholders::_1));
        goal_pub = this->create_publisher<augv_navigation_msgs::msg::Position>("/robot1/goal", 10);
        arrow_pub = this->create_publisher<visualization_msgs::msg::Marker>("/robot1/goal_arrow", 10);
    }

    private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_goal_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr potential_sub;
    rclcpp::Publisher<augv_navigation_msgs::msg::Position>::SharedPtr goal_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arrow_pub;
    geometry_msgs::msg::TwistStamped field_vel;

    void logic(float rotation_diff, float dx, float dy, float dz, float dt, float curr_orientation) override
    {
        EulerAngles ea;
        auto xy = ea.rotate_vector_by_angle(dx, dy, -curr_orientation);
        dx = xy.at(0);
        dy = xy.at(1);
        rotation_diff = atan2(dy, dx);
        ea.setRPY(0, 0, rotation_diff);
        visualization_msgs::msg::Marker dmarker;
        dmarker.header.frame_id = "robot1/base_link";
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
        // rotation_diff -= curr_orientation;
        // ea.normalize_angle(rotation_diff);
        RCLCPP_INFO_STREAM(this->get_logger(), "rotation_diff = " << rotation_diff);
        RCLCPP_INFO_STREAM(this->get_logger(), "dx = " << dx);
        if (!rot_errs.empty())
        {
            if (rot_errs.size() > 10)
            {
                rot_errs.erase(rot_errs.begin());
                x_errs.erase(x_errs.begin());
            }

            rot_integr += rotation_diff * dt;
            x_integr += dx * dt;

            rot_diff = (rotation_diff - rot_errs.at(rot_errs.size() - 1)) / dt;
            x_diff = (dx - x_errs.at(x_errs.size() - 1)) / dt;

            float rot_sig = kp * rotation_diff + kd * rot_diff + ki * rot_integr; 
            float x_sig = kp * dx + kd * x_diff + ki * x_integr; 
            geometry_msgs::msg::TwistStamped twist;
            twist.header.stamp = this->get_clock()->now();
            twist.twist.linear.x = x_sig + field_vel.twist.linear.x;
            twist.twist.angular.z = rot_sig + field_vel.twist.angular.z / 2;
            if (twist.twist.linear.x < 0) twist.twist.angular.z = twist.twist.angular.z * -1;
            cmd_vel_pub->publish(twist);
            RCLCPP_INFO_STREAM(this->get_logger(), "twist.twist.linear.x = " << twist.twist.linear.x << "\n" << "twist.twist.angular.z = " << twist.twist.angular.z);
        }
        rot_errs.push_back(rotation_diff);
        x_errs.push_back(dx);
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