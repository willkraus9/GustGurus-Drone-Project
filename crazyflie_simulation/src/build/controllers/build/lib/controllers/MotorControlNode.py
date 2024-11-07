import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from actuator_msgs.msg import Actuators  


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('rotor_control')

        # Create individual publishers for each rotor
        self.publisher_ = self.create_publisher(Actuators, '/crazyflie/motor_commands', 10)
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
 
    def timer_callback(self):
        # Define the motor speeds for each rotor
        motor_speeds = [2000.0, 2000.0, 2000.0, 2000.0]  

        # Create and populate the Float32MultiArray message
        msg = Actuators()
        msg.velocity = motor_speeds

        # Publish the motor speeds
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    motor_control_node = MotorControlNode()

    rclpy.spin(motor_control_node)

    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
