#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <memory>
#include "actuator_msgs/msg/actuators.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "pid_controller.h"
#include <Eigen/Dense>
#include <cmath>

class MotorControlNode : public rclcpp::Node
{
public:
    desired_state_t goal;
    actual_state_t state;
    gains_pid_t gain;
    std::vector<double> gains;
    motor_power_t motor_commands;


    bool simulation_running;
    double current_simulation_time;
    double past_simulation_time;
    double gamma = 0.005964552; // rotor drag to thrust ratio
    double thrust_coefficient = 1.28192e-08;

    double L = 0.046 * cos(M_PI/4.0); // meters

    Eigen::Matrix4d Thrust_torque_to_motor_forces;
    Eigen::Matrix4d motor_forces_to_thrust_torque;
    
    MotorControlNode() : Node("motor_control_node")
    {
        // Create a publisher for the motor control topic
        motor_control_pub = this->create_publisher<actuator_msgs::msg::Actuators>("/crazyflie/motor_commands", 10);

        

        // Create a subscriber for the drone state
        drone_state_sub = this->create_subscription<nav_msgs::msg::Odometry>("crazyflie/odom", 10, std::bind(&MotorControlNode::droneStateCallback, this, std::placeholders::_1));

        // create a subscriber for the simluation time
        simulation_time_sub = this->create_subscription<rosgraph_msgs::msg::Clock>("clock", 10, std::bind(&MotorControlNode::simulationTimeCallback, this, std::placeholders::_1));

        // create subscriber for the goal state
        goal_state_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("crazyflie/goal_state_vector", 10, std::bind(&MotorControlNode::goalStateCallback, this, std::placeholders::_1));

    goal= {
        .roll = 0.0,
        .pitch = 0.0,
        .yaw_rate = 0.0,
        .altitude = 0.0, // Target altitude in meters
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

    this->declare_parameter<std::vector<double>>("gains", {1e-3, 5e-4, 0, 0, 0, 0, 0.14, 0.07, 0});
    // this->declare_parameter<std::vector<double>>("gains", {200.0, 100.50, 200.0, 100.50, 0.0, 0.0, 2350.0, 1300.0, 0.0});



    // gain = {
    //   .kp_att_rp = 200.0,
    //   .kd_att_rp = 100.50,

    //   .kp_att_y = 200.0,
    //   .kd_att_y = 100.50,

    //   .kp_vel_xy = 0.0,
    //   .kd_vel_xy = 0.0,

    //   .kp_z = 2350.0,
    //   .kd_z = 1300.0,
    //   .ki_z = 0.0
    // };

    Eigen::Matrix4d scale_matrix;
    scale_matrix << 0.25, 0, 0, 0,
                    0, 0.25/L, 0, 0,
                    0, 0, 0.25/L, 0,
                    0, 0, 0, 0.25/gamma;

    Thrust_torque_to_motor_forces << 1, -1, 1, 1,
                                     1, 1, -1, 1,
                                     1, 1, 1, -1,
                                     1, -1, -1,-1;

    // std::cout << "Thrust_torque_to_motor_forces: " << Thrust_torque_to_motor_forces << std::endl;               

    Thrust_torque_to_motor_forces = Thrust_torque_to_motor_forces * scale_matrix;

    motor_forces_to_thrust_torque << 1.0, 1.0, 1.0, 1.0,
                                      -L, L, L, -L,
                                      L, -L, L, -L,
                                      gamma, gamma, -gamma, -gamma;

    std::cout << "motor_forces_to_thrust_torque: " << motor_forces_to_thrust_torque << std::endl;
    std::cout << "Thrust_torque_to_motor_forces: " << Thrust_torque_to_motor_forces.inverse() << std::endl;


    // std::cout << "Thrust_torque_to_motor_forces: " << Thrust_torque_to_motor_forces << std::endl;
    // std::cout << scale_matrix << std::endl;


    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&MotorControlNode::timerCallback, this)
        );

    init_pid_attitude_fixed_height_controller();

  }
private:
    void goalStateCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Purpose of this function is to receive the goal state at every time step
        goal.roll = msg->data[0];
        goal.pitch = msg->data[1];
        goal.yaw_rate = msg->data[2];
        goal.altitude = msg->data[3];
    }
    void simulationTimeCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
      // Purpose of this function is to receive the simulation time at every time step
      double nanos = msg->clock.nanosec;
      int secs = msg->clock.sec;

      current_simulation_time = secs * 1e9 + nanos;
      // conver time to milliseconds
      current_simulation_time = current_simulation_time / 1e6;

      // Check if the simulation is running
      if (current_simulation_time > past_simulation_time){
            simulation_running = true;
      } // Check if the simulation is reset
        else if (current_simulation_time < past_simulation_time || current_simulation_time == 0.0){
            simulation_running = false;
            init_pid_attitude_fixed_height_controller();
      } // Check if the simulation is paused
        else if (current_simulation_time == past_simulation_time){
            simulation_running = false;
      }
        past_simulation_time = current_simulation_time;
    }

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

        roll = wrapAngle(roll); // wrap angle to [-pi, pi]
        pitch = wrapAngle(pitch); 
        yaw = wrapAngle(yaw);
        // std::cout << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << std::endl;


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

        // if the simulation is not running, do not publish motor commands
        if (!simulation_running){
          return;
        }

        this->get_parameter("gains", gains);

        gain = {
            .kp_att_rp = gains[0],
            .kd_att_rp = gains[1],

            .kp_att_y = gains[2],
            .kd_att_y = gains[3],

            .kp_vel_xy = gains[4],
            .kd_vel_xy = gains[5],

            .kp_z = gains[6],
            .kd_z = gains[7],
            .ki_z = gains[8]
        };

        double dt = 0.001; // 1ms timer interval
        
        control_commands_t control_commands = {0};
        pid_attitude_fixed_height_controller(state, &goal, gain, dt, &control_commands);

        // Create and populate the actuator message
        Eigen::Vector4d Thrust_torque;
        Thrust_torque << control_commands.altitude, control_commands.roll, control_commands.pitch, control_commands.yaw;

        std::cout << "Thrust_torque: " << Thrust_torque << std::endl;
        Eigen::Vector4d motor_forces = Thrust_torque_to_motor_forces * Thrust_torque;
        // std::cout << "Motor forces: " << motor_forces << std::endl;

        for (int i = 0; i < 4; i++){
          if (motor_forces[i] < 0){
            motor_forces[i] = 0;
          }
        }
        Eigen::Vector4d motor_speeds = sqrt(motor_forces.array() / thrust_coefficient);

        auto msg = actuator_msgs::msg::Actuators();
        msg.velocity = {motor_speeds[0], motor_speeds[1], motor_speeds[2], motor_speeds[3]};
        motor_control_pub->publish(msg);

        

        // pid_attitude_fixed_height_controller2(state, &goal, gain, dt, &motor_commands);
        // Eigen::Vector4d motor_speeds = {motor_commands.m1, motor_commands.m2, motor_commands.m3, motor_commands.m4};
        // Eigen::Vector4d motor_forces = thrust_coefficient * motor_speeds.array() * motor_speeds.array();
        // Eigen::Vector4d Thrust_torque = motor_forces_to_thrust_torque * motor_forces;
        // std::cout << "Thrust_torque: " << Thrust_torque << std::endl;
        // auto msg = actuator_msgs::msg::Actuators();
        // msg.velocity = {motor_commands.m1, motor_commands.m2, motor_commands.m3, motor_commands.m4};
        // motor_control_pub->publish(msg);


        // RCLCPP_INFO(this->get_logger(), "Publishing motor speeds: %f", speed);
    }

    double wrapAngle(double angle) {
        // Wrap angle to [-pi, pi]
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle <= -M_PI) angle += 2 * M_PI;
        return angle;
    }
    
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_control_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_state_sub;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr simulation_time_sub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr goal_state_sub;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControlNode>());
  rclcpp::shutdown();
  return 0;
};


