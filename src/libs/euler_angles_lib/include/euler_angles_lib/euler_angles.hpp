#pragma once

#include <iostream>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

// Класс для работы с Эйлеровыми углами и кватернионами
class EulerAngles {
    public:
    EulerAngles()
    {
        roll_ = 0;
        pitch_ = 0;
        yaw_ = 0;
    }


    double roll()
    {
        return roll_;
        // return normalize_angle(roll_);tf2_geometry_msgs/tf2_geometry_msgs.hpp
    }


    double pitch()
    {
        // return normalize_angle(pitch_);radians
        return pitch_;
    }


    double yaw()
    {
        return yaw_;
        // return normalize_angle(yaw_);
    }


    /**
     * @brief Returns 2-element vector with rotated x and y
     * 
     * @param x meters - coords.at(0)
     * @param y meters - coords.at(1)
     * @param angle radians 
     * @return std::vector<double>  
     */
    std::vector<double> rotate_vector_by_angle(double x, double y, double angle)
    {
        std::vector<double> coords;
        coords.resize(2);
        coords.at(0) = x * cos(angle) - y * sin(angle);
        coords.at(1) = x * sin(angle) + y * cos(angle);

        return coords;

        // finvec_x = tmpvec_x * cos(angles.yaw) - tmpvec_y * sin(angles.yaw);
        // finvec_y = tmpvec_x * sin(angles.yaw) + tmpvec_y * cos(angles.yaw);
    }


    geometry_msgs::msg::Quaternion get_current_msg_quaternion()
    {
        geometry_msgs::msg::Quaternion q;
        setRPY_of_quaternion(q);
        return q;
    }


    tf2::Quaternion get_current_tf2_quaternion()
    {
        tf2::Quaternion q;
        setRPY_of_quaternion(q);
        return q;
    }


    // Функция для установки Эйлеровых углов по переданным параметрам
    void setRPY(float new_roll, float new_pitch, float new_yaw)
    {
        roll_ = new_roll;
        pitch_ = new_pitch;
        yaw_ = new_yaw;
    }


    /**
    * @brief Функция для передачи Эйлеровых углов в вектор вращения.
    * 
    * Нормализация кватернионов есть
    */
    void setRPY_of_quaternion(tf2::Quaternion &q)
    {
        q.setRPY(roll_, pitch_, yaw_);
        q.normalize();
    }


    /**
    * @brief Функция для передачи Эйлеровых углов в вектор вращения.
    * 
    * Нормализация кватернионов есть
    */
    void setRPY_of_quaternion(geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion q_tf;
        q_tf.setRPY(roll_, pitch_, yaw_);
        q_tf.normalize();

        q.w = q_tf.getW();
        q.x = q_tf.getX();
        q.y = q_tf.getY();
        q.z = q_tf.getZ();
    }


    // Функция для получения Эйлеровых углов из вектора вращения 
    void get_RPY_from_quaternion(tf2::Quaternion q)
    {
        tf2::Matrix3x3 m(q);
        m.getRPY(roll_, pitch_, yaw_);
    }


    /**
    * @brief Функция для получения Эйлеровых углов из вектора вращения заданного geometry_msgs::msg::Quaternion.
    * 
    * Нормализация кватернионов есть
    */
    void get_RPY_from_msg_quaternion(geometry_msgs::msg::Quaternion q)
    {
        tf2::Quaternion q_tf;
        tf2::convert(q, q_tf);

        q_tf.normalize();

        get_RPY_from_quaternion(q_tf);
    }


    /**
    * @brief Returns quaternion with applied rotation
    */
    geometry_msgs::msg::Quaternion rotate_position_yaw_by_angle(geometry_msgs::msg::Quaternion orientation, double angle)
    {
        get_RPY_from_msg_quaternion(orientation);
        yaw_ = yaw_ + angle;

        geometry_msgs::msg::Quaternion q;
        setRPY_of_quaternion(q);
        return q;
    }


    /**
    * @brief Нормализует угол в пределах от -pi до pi
    * @warning НЕ РАБОТАЕТ
    */
    double normalize_angle(double angle)
    {
        double new_angle = angle;
        if (angle > M_PI)
        {
            new_angle = angle - 2 * M_PI;
        }
        else if (angle < - M_PI)
        {
            new_angle = angle + 2 * M_PI;
        }

        if (new_angle > M_PI)
        {
            std::cerr << "EulerAngles: returning unnormalized angle";
        }

        return new_angle;
    }


    double quantize_angle_by(double radians, double angle)
    {
        double division_res = angle / radians;
        // ROS_INFO_STREAM("\nangle: " << angle << "\nradians: " << radians << "\ndivision_res * radians: " << round(division_res) * radians << "\ndivision_res raw: " << division_res << "\ndivision_res rounded: " << round(division_res));
        division_res = round(division_res);
        return division_res * radians;
    }


    geometry_msgs::msg::Quaternion quantize_quaternion_yaw_by(double radians, geometry_msgs::msg::Quaternion q)
    {
        get_RPY_from_msg_quaternion(q);
        yaw_ = quantize_angle_by(radians, yaw_);
        
        setRPY_of_quaternion(q);
        return q;
    }


    private:
    double roll_;
    double pitch_;
    double yaw_;
};


