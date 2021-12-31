#ifndef __ROOMBA_TELEOP_HPP
#define __ROOMBA_TELEOP_HPP
 
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

using namespace std::chrono_literals;

class RoombaTeleop : public rclcpp::Node
{
public:
    RoombaTeleop(void);

    void timer_callback(void);
    void command_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

private:
    // Parameters
    double HZ;
    double MAX_SPEED;
    double MAX_YAWRATE;
    double VEL_RATIO;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr dock_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr undock_pub;

    // ROS message
    geometry_msgs::msg::Twist cmd_vel;
    geometry_msgs::msg::Twist joy_vel;
    std_msgs::msg::Empty empty_msgs;

    // flag
    bool auto_flag;
    bool move_flag;
    bool dock_flag;
    bool pre_dock_flag;

};

#endif// __ROOMBA_TELEOP_HPP