#include "roomba_teleop/roomba_teleop.hpp"

RoombaTeleop::RoombaTeleop(void)
    :Node("roomba_teleop"),
     auto_flag(false), move_flag(false), dock_flag(false), pre_dock_flag(false)
{
    // Parameters
    this->declare_parameter<double>("HZ", 20);
    this->declare_parameter<double>("MAX_SPEED", 0.5);
    this->declare_parameter<double>("MAX_YAWRATE", 1.0);
    this->declare_parameter<double>("VEL_RATIO", 0.5);
    this->get_parameter("HZ", HZ);
    this->get_parameter("MAX_SPEED", MAX_SPEED);
    this->get_parameter("MAX_YAWRATE", MAX_YAWRATE);
    this->get_parameter("VEL_RATIO", VEL_RATIO);

    // Timer
    timer_ = this->create_wall_timer(50ms, std::bind(&RoombaTeleop::timer_callback, this));

    // Subscribers
    cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>("/planner/cmd_vel", 10, std::bind(&RoombaTeleop::command_callback, this, std::placeholders::_1));
    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&RoombaTeleop::joy_callback, this, std::placeholders::_1));

    // Publishers
    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    dock_pub = this->create_publisher<std_msgs::msg::Empty>("dock", 1);
    undock_pub = this->create_publisher<std_msgs::msg::Empty>("undock", 1);
}

void RoombaTeleop::command_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    cmd_vel = *msg;
}

void RoombaTeleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    sensor_msgs::msg::Joy joy = *msg;
    if(joy.buttons[2]){
        auto_flag = false;
    }else if(joy.buttons[3]){
        auto_flag = true;
    }
    if(joy.buttons[0]){
        move_flag = false;
    }else if(joy.buttons[1]){
        move_flag = true;
    }
    if(joy.buttons[6]){
        dock_flag = false;
    }else if(joy.buttons[7]){
        dock_flag = true;
    }

    joy_vel.linear.x = joy.axes[1]*MAX_SPEED;
    joy_vel.angular.z = joy.axes[0]*MAX_YAWRATE;

    if(joy.axes[7]==1.0){
        joy_vel.linear.x = VEL_RATIO*MAX_SPEED;
        joy_vel.angular.z = 0.0;
    }else if(joy.axes[7]==-1.0){
        joy_vel.linear.x = -VEL_RATIO*MAX_SPEED;
        joy_vel.angular.z = 0.0;
    }else if(joy.axes[6]==1.0){
        joy_vel.linear.x = 0.0;
        joy_vel.angular.z = VEL_RATIO*MAX_YAWRATE;
    }else if(joy.axes[6]==-1.0){
        joy_vel.linear.x = 0.0;
        joy_vel.angular.z = -VEL_RATIO*MAX_YAWRATE;
    }
    if(!joy.buttons[4]){
        joy_vel.linear.x = 0.0;
        joy_vel.angular.z = 0.0;
    }
}

void RoombaTeleop::timer_callback(void)
{
    RCLCPP_INFO(this->get_logger(), "==== roomba teleop ====");
    geometry_msgs::msg::Twist vel;
    if(dock_flag){
        if(!pre_dock_flag) dock_pub->publish(empty_msgs);
        RCLCPP_INFO(this->get_logger(), "docking");
    }else{
        if(pre_dock_flag) undock_pub->publish(empty_msgs);
        RCLCPP_INFO_STREAM(this->get_logger(), (move_flag ? "move" : "stop") << " : (" << (auto_flag ? "auto" : "manual") << ")");
        if(move_flag){
            if(auto_flag) vel = cmd_vel;
            else vel = joy_vel;
            vel.linear.x = std::min(std::max(vel.linear.x, -MAX_SPEED), MAX_SPEED);
            vel.angular.z = std::min(std::max(vel.angular.z, -MAX_YAWRATE), MAX_YAWRATE);
            RCLCPP_INFO_STREAM(this->get_logger(), "vel: (" << vel.linear.x << "[m/s], " << vel.angular.z << "[rad/s])");
        }else{
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;
        }
        vel_pub->publish(vel);
    }
    pre_dock_flag = dock_flag;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoombaTeleop>());
    rclcpp::shutdown();
    return 0;
}