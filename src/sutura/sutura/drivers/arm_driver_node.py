import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from sutura.drivers.mock_driver import MockArmDriver
from sutura.drivers.makiina_driver import MakiinaArmDriver


class ArmDriverNode(Node):
    def __init__(self):
        super().__init__('arm_driver_node')

        # Parameters for selecting the driver type and CAN interface
        self.declare_parameter('arm_type', 'makiina')  # Options: 'makiina' or 'mock'
        self.declare_parameter('can_interface', 'can0')

        arm_type = self.get_parameter('arm_type').get_parameter_value().string_value
        can_interface = self.get_parameter('can_interface').get_parameter_value().string_value

        # Instantiate the appropriate driver based on the parameter
        if arm_type == 'makiina':
            self.get_logger().info(f"Instanciando Driver MAKIINA na interface '{can_interface}'...")
            self.driver = MakiinaArmDriver(interface=can_interface)
        elif arm_type == 'mock':
            self.get_logger().info("Instanciando Driver SIMULADO (Mock)...")
            self.driver = MockArmDriver()
        else:
            self.get_logger().error(f"Tipo de arm_type '{arm_type}' não reconhecido! Fechando nó...")
            return

        if not self.driver.connect():
            self.get_logger().error("Falha na inicialização do driver!")
            return

        # Publisher for joint states
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Subscriber for joint commands
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/joint_commands',
            self._command_callback,
            10
        )

        # Timer to periodically publish joint states
        self.timer = self.create_timer(0.1, self._publish_joint_states)

    def _command_callback(self, msg: Float32MultiArray):
        self.driver.set_joint_positions(msg.data)

    def _publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'joint_{i+1}' for i in range(6)]
        msg.position = self.driver.get_joint_positions()
        msg.effort = self.driver.get_joint_torques()
        self.joint_state_pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'driver'):
            self.driver.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()