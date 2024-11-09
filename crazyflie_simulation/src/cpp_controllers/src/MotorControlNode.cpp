#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "actuator_msgs/msg/actuators.hpp"
#include "pid_controller.h"

class MotorControlNode : public rclcpp::Node
{
public:
  MotorControlNode() : Node("motor_control_node")
  {
    // Create a publisher for the motor control topic
    motor_control_pub = this->create_publisher<actuator_msgs::msg::Actuators>("/crazyflie/motor_commands", 10);

    this ->declare_parameter("motor_speed", 2000.0);

    // Create a subscriber for the drone state
    // drone_state_sub = this->create_subscription<nav_msgs::msg::Odometry>("crazyflie/odom", 10, std::bind(&MotorControlNode::droneStateCallback, this, std::placeholders::_1));


    // desired_state_s = {
    //   .roll = 0.0,
    //   .pitch = 0.0,
    //   .yaw_rate = 0.0,
    //   .altitude = 2.0, // Target altitude in meters
    //   .vx = 0.0,
    //   .vy = 0.0
    // };

    // gains_pid = {
    //   .kp_att_rp = 1.0,
    //   .kd_att_rp = 0.1,
    //   .kp_att_y = 1.2,
    //   .kd_att_y = 0.15,
    //   .kp_vel_xy = 1.5,
    //   .kd_vel_xy = 0.2,
    //   .kp_z = 2.0,
    //   .kd_z = 0.5,
    //   .ki_z = 0.1
    // };

    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MotorControlNode::timerCallback, this)
        );
  }

private:
    void droneStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        
    }

    void timerCallback()
    {
      // Purpose of thsi function is to publish motor speed to the gazebo drone at every time step
      // TODO: Implement the PID controller here
      // The PID controller should calculate the motor speeds based on the current state of the drone
        double speed = this->get_parameter("motor_speed").as_double();
        std::vector<double> motor_speeds = {speed, speed, speed, speed};

        auto msg = actuator_msgs::msg::Actuators();
        msg.velocity = motor_speeds;
        motor_control_pub->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Publishing motor speeds: %f", speed);
    }
    
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_control_pub;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControlNode>());
  rclcpp::shutdown();
  return 0;
};


