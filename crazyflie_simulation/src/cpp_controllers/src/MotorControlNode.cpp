#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <memory>
#include "actuator_msgs/msg/actuators.hpp"
#include "pid_controller.h"
#include <Eigen/Dense>

class MotorControlNode : public rclcpp::Node
{
public:
  desired_state_t goal;
  actual_state_t state;
  gains_pid_t gain;
  control_commands_t control_commands;
  motor_power_t motorCommands;
  

  MotorControlNode() : Node("motor_control_node")
  {
    // Create a publisher for the motor control topic
    motor_control_pub = this->create_publisher<actuator_msgs::msg::Actuators>("/crazyflie/motor_commands", 10);

    this ->declare_parameter("motor_speed", 2000.0);

    // Create a subscriber for the drone state
    //motor_control_sub = this->create_subscription<actual_msgs::msg::ActualState>("crazyflie/actual_state", 10, std::bind(&MotorControlNode::droneStateCallback, this, std::placeholders::_1));
    drone_state_sub = this->create_subscription<nav_msgs::msg::Odometry>("crazyflie/odom", 10, std::bind(&MotorControlNode::droneStateCallback, this, std::placeholders::_1));

    goal= {
      .roll = 0.0,
      .pitch = 0.0,
      .yaw_rate = 0.0,
      .altitude = 2.0, // Target altitude in meters
      .vx = 0.0,
      .vy = 0.0
    };

    state = {
      .roll = 0.0,
      .pitch = 0.0,
      .yaw_rate = 0.0,
      .altitude = 0.0,
      .vx = 0.0,
      .vy = 0.0
    };

    gain = {
      .kp_att_rp = 70.0,
      .kd_att_rp = 50.50,

      .kp_att_y = 70.0,
      .kd_att_y = 50.50,

      .kp_vel_xy = 0.0,
      .kd_vel_xy = 0.0,

      .kp_z = 2650.0,
      .kd_z = 1300.0,
      .ki_z = 0.0
    };

    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&MotorControlNode::timerCallback, this)
        );

    init_pid_attitude_fixed_height_controller();

  }

private:
    void droneStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      // Purpose of this function is to receive the state of the drone at every time step
      // float x = msg->pose.pose.position.x;
      // float y = msg->pose.pose.position.y;
      float z = msg->pose.pose.position.z;
      float quat_x = msg->pose.pose.orientation.x;
      float quat_y = msg->pose.pose.orientation.y;
      float quat_z = msg->pose.pose.orientation.z;
      float quat_w = msg->pose.pose.orientation.w;

      // Convert quaternion to euler angles
      double roll, pitch, yaw;
      Eigen::Quaternionf quaternion(quat_w, quat_x, quat_y, quat_z);
      Eigen::Matrix3f rotationMatrix = quaternion.toRotationMatrix();
      Eigen::Vector3f euler = rotationMatrix.eulerAngles(2, 1, 0);
      roll = euler[2];
      pitch = euler[1];
      yaw = euler[0];

      std::cout << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << std::endl;

      float xd = msg->twist.twist.linear.x;
      float yd = msg->twist.twist.linear.y;
      // float zd = msg->twist.twist.linear.z;

      // float roll_d = msg->twist.twist.angular.x;
      // float pitch_d = msg->twist.twist.angular.y;
      float yaw_d = msg->twist.twist.angular.z;

      state.roll = static_cast<double>(roll);
      state.pitch = pitch;
      state.yaw_rate = yaw_d;
      state.altitude = z;
      state.vx = xd;
      state.vy = yd;
    }


    void timerCallback()
    {
      // Purpose of thsi function is to publish motor speed to the gazebo drone at every time step
      // TODO: Implement the PID controller here
      // The PID controller should calculate the motor speeds based on the current state of the drone
        // Call the PID fixed height controller to compute control commands

        double dt = 0.001; // 1ms timer interval
        
        pid_velocity_fixed_height_controller(state, &goal, gain, dt, &motorCommands);

        // Create and populate the actuator message
        auto msg = actuator_msgs::msg::Actuators();
        msg.velocity = {motorCommands.m1, motorCommands.m2, motorCommands.m3, motorCommands.m4};
        motor_control_pub->publish(msg);

        // RCLCPP_INFO(this->get_logger(), "Publishing motor speeds: %f", speed);
    }
    
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_control_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_state_sub;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControlNode>());
  rclcpp::shutdown();
  return 0;
};


