import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class RotorControlNode(Node):
    def __init__(self):
        super().__init__('rotor_control')

        # Create individual publishers for each rotor
        self.publisher_ = self.create_publisher(Float32MultiArray, 'crazyflie/motor_commands', 10)
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
 
    def timer_callback(self):
        # Define the motor speeds for each rotor
        motor_speeds = [700.0, 700.0, 700.0, 700.0]  # Example speeds in arbitrary units

        # Create and populate the Float32MultiArray message
        msg = Float32MultiArray()
        msg.data = motor_speeds

        # Publish the motor speeds
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published motor speeds: {motor_speeds}')

def main(args=None):
    rclpy.init(args=args)

    rotor_control = RotorControlNode()

    rclpy.spin(rotor_control)

    rotor_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
