import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('rotor_control')

        # Create publisher for motor commands
        self.publisher_ = self.create_publisher(Actuators, '/crazyflie/motor_commands', 10)
        
        # Declare and initialize motor speed parameter
        self.declare_parameter("motor_speed", 2000.0)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Get motor speed parameter value
        speed = self.get_parameter("motor_speed").get_parameter_value().double_value
        motor_speeds = [speed, speed, speed, speed]

        # Publish motor speeds
        msg = Actuators()
        msg.velocity = motor_speeds
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published motor speeds: {motor_speeds}')

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
