#include "rclcpp/rclcpp.hpp"
#include <termios.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <unistd.h>
#include <vector>

class KeyboardControlNode : public rclcpp::Node
{
public:
    KeyboardControlNode() : Node("keyboard_control_node")
    {
        // Create a publisher for a vector
        vector_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("crazyflie/goal_state_vector", 10);

        // Initialize the vector with default values
        goal_state_vector_ = {0.0, 0.0, 0.0, 0.0}; // [roll, pitch, yaw, altitude]
    }

    void run()
    {
        struct termios oldt, newt;
        char ch;

        // Set terminal to raw mode
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        std::cout << "Keyboard control enabled. Use keys to change goal state:\n"
                  << "w/s: Increase/Decrease altitude\n"
                  << "a/d: Increase/Decrease roll\n"
                  << "q/e: Increase/Decrease pitch\n"
                  << "z/x: Increase/Decrease yaw rate\n"
                  << "Press ESC to exit.\n";

        while (rclcpp::ok())
        {
            ch = getchar();
            switch (ch)
            {
            case 'w':
                goal_state_vector_[3] += 0.1; // Increase altitude
                break;
            case 's':
                goal_state_vector_[3] -= 0.1; // Decrease altitude
                break;
            case 'a':
                goal_state_vector_[0] -= 0.1; // Decrease y
                break;
            case 'd':
                goal_state_vector_[0] += 0.1; // Increase y
                break;
            case 'q':
                goal_state_vector_[1] += 0.1; // Increase x
                break;
            case 'e':
                goal_state_vector_[1] -= 0.1; // Decrease x
                break;
            case 'z':
                goal_state_vector_[2] += 0.1; // Increase yaw 
                break;
            case 'x':
                goal_state_vector_[2] -= 0.1; // Decrease yaw 
                break;
            case 27: // ESC
                tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Restore terminal
                return;
            default:
                break;
            }

            std::cout << "Roll: " << goal_state_vector_[0] << std::endl;
            std::cout << "Pitch: " << goal_state_vector_[1] << std::endl;
            std::cout << "Yaw rate: " << goal_state_vector_[2] << std::endl;
            std::cout << "Altitude: " << goal_state_vector_[3] << std::endl;
            

            // Publish the vector
            std_msgs::msg::Float32MultiArray msg;
            msg.data = goal_state_vector_;
            vector_pub_->publish(msg);
        }
    }

private:
    std::vector<float> goal_state_vector_; // [roll, pitch, yaw, altitude]
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr vector_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardControlNode>();

    std::thread keyboard_thread(&KeyboardControlNode::run, node);
    rclcpp::spin(node);

    if (keyboard_thread.joinable())
        keyboard_thread.join();
    rclcpp::shutdown();
    return 0;
}
