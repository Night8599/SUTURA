import queue
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger

from sutura.drivers.base_driver import BaseArmDriver
from sutura.drivers.makiina_driver import MakiinaArmDriver
from sutura.drivers.mock_driver import MockArmDriver
from sutura.drivers.ned1_driver import Ned1ArmDriver

JOINT_NAMES = [f'joint_{i + 1}' for i in range(6)]


class ArmDriverNode(Node):
    """
    Bridges ROS 2 and a BaseArmDriver.

    Topics:
        sub /joint_commands            Float32MultiArray, target joint positions (rad)
        pub /joint_states              JointState
        pub /sutura/hardware_status    String: READY | MOVING | ESTOP | ERROR | DISCONNECTED
    Services (std_srvs/Trigger):
        /sutura/arm/emergency_stop
        /sutura/arm/reset_emergency_stop
        /sutura/arm/home               (Ned1 only)

    Drivers such as Ned1 block during motion, so commands are executed by a worker
    thread and never inside a ROS callback.
    """

    def __init__(self):
        super().__init__('arm_driver_node')

        # ----------------------------------------------------------------------
        # 1. Parameters
        # ----------------------------------------------------------------------
        self.declare_parameter('arm.type', 'mock')  # 'ned1', 'mock' or 'makiina'
        self.declare_parameter('arm.state_rate_hz', 10.0)
        self.declare_parameter('arm.command_queue_size', 1)
        self.declare_parameter('arm.can_interface', 'can0')
        self.declare_parameter('arm.ned1.ip_address', '10.10.10.10')
        self.declare_parameter('arm.ned1.velocity_percent', 20)
        self.declare_parameter('arm.ned1.auto_calibrate', True)
        self.declare_parameter('arm.ned1.safety_margin_rad', 0.02)
        self.declare_parameter('arm.ned1.max_step_rad', 0.5)

        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.status_pub = self.create_publisher(String, '/sutura/hardware_status', status_qos)
        self._last_status = None

        # ----------------------------------------------------------------------
        # 2. Driver
        # ----------------------------------------------------------------------
        self.driver = self._build_driver()
        self._connected = self.driver is not None and self.driver.connect()
        if not self._connected:
            self.get_logger().error("Driver initialization failed. Commands will be refused.")
            self._publish_status('ERROR')

        # ----------------------------------------------------------------------
        # 3. Command worker
        # ----------------------------------------------------------------------
        queue_size = max(1, self.get_parameter('arm.command_queue_size').value)
        self._commands: queue.Queue = queue.Queue(maxsize=queue_size)
        self._shutdown = threading.Event()
        self._worker = threading.Thread(target=self._command_worker, daemon=True)
        self._worker.start()

        # ----------------------------------------------------------------------
        # 4. ROS interfaces
        # ----------------------------------------------------------------------
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(Float32MultiArray, '/joint_commands', self._command_callback, 10)
        self.create_service(Trigger, '/sutura/arm/emergency_stop', self._estop_callback)
        self.create_service(Trigger, '/sutura/arm/reset_emergency_stop', self._reset_estop_callback)
        self.create_service(Trigger, '/sutura/arm/home', self._home_callback)

        rate = self.get_parameter('arm.state_rate_hz').value
        self.create_timer(1.0 / rate, self._publish_state)

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------
    def _build_driver(self) -> BaseArmDriver | None:
        arm_type = self.get_parameter('arm.type').value
        log = self.get_logger()

        if arm_type == 'ned1':
            ip = self.get_parameter('arm.ned1.ip_address').value
            max_step = self.get_parameter('arm.ned1.max_step_rad').value
            log.info(f"Creating Ned1 driver for {ip}...")
            return Ned1ArmDriver(
                ip_address=ip,
                velocity_percent=self.get_parameter('arm.ned1.velocity_percent').value,
                auto_calibrate=self.get_parameter('arm.ned1.auto_calibrate').value,
                safety_margin_rad=self.get_parameter('arm.ned1.safety_margin_rad').value,
                max_step_rad=max_step if max_step > 0.0 else None,
            )
        if arm_type == 'mock':
            log.info("Creating simulated (mock) driver...")
            return MockArmDriver()
        if arm_type == 'makiina':
            interface = self.get_parameter('arm.can_interface').value
            log.info(f"Creating MAKIINA driver on '{interface}'...")
            return MakiinaArmDriver(interface=interface)

        log.error(f"Unknown arm.type '{arm_type}'.")
        return None

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------
    def _command_callback(self, msg: Float32MultiArray) -> None:
        if not self._connected:
            self.get_logger().warn("Command ignored: driver not connected.")
            return
        if getattr(self.driver, 'estop_latched', False):
            self.get_logger().warn("Command ignored: emergency stop latched.")
            return
        try:
            self._commands.put_nowait(('move', list(msg.data)))
        except queue.Full:
            self.get_logger().warn("Command ignored: arm busy and command queue full.")

    def _command_worker(self) -> None:
        while not self._shutdown.is_set():
            try:
                kind, target = self._commands.get(timeout=0.1)
            except queue.Empty:
                continue
            ok = self.driver.move_to_home() if kind == 'home' else self.driver.set_joint_positions(target)
            if not ok:
                reason = getattr(self.driver, 'last_error', None) or 'unknown error'
                self.get_logger().error(f"Command failed: {reason}")

    def _drain_commands(self) -> None:
        while True:
            try:
                self._commands.get_nowait()
            except queue.Empty:
                return

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------
    def _estop_callback(self, _request, response):
        self._drain_commands()
        if self.driver is not None:
            self.driver.emergency_stop()
        response.success = True
        response.message = "Emergency stop latched."
        if getattr(self.driver, 'is_moving', False):
            response.message += " WARNING: current motion cannot be interrupted by software."
        return response

    def _reset_estop_callback(self, _request, response):
        reset = getattr(self.driver, 'reset_emergency_stop', None)
        if reset is None:
            response.success = False
            response.message = "Driver has no emergency stop latch."
            return response
        reset()
        response.success = True
        response.message = "Emergency stop reset."
        return response

    def _home_callback(self, _request, response):
        home = getattr(self.driver, 'move_to_home', None)
        if home is None or not self._connected:
            response.success = False
            response.message = "Home not available for this driver."
            return response
        if getattr(self.driver, 'estop_latched', False):
            response.success = False
            response.message = "Emergency stop latched."
            return response
        try:
            self._commands.put_nowait(('home', None))
        except queue.Full:
            response.success = False
            response.message = "Arm busy and command queue full."
            return response
        response.success = True
        response.message = "Home motion queued. Watch /sutura/hardware_status."
        return response

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------
    def _publish_state(self) -> None:
        if self.driver is None or not self._connected:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(v) for v in self.driver.get_joint_positions()]
        msg.effort = [float(v) for v in self.driver.get_joint_torques()]
        self.joint_state_pub.publish(msg)
        self._publish_status(self._current_status())

    def _current_status(self) -> str:
        if getattr(self.driver, 'estop_latched', False):
            return 'ESTOP'
        if getattr(self.driver, 'is_moving', False):
            return 'MOVING'
        if not getattr(self.driver, 'is_connected', True):
            return 'DISCONNECTED'
        return 'READY'

    def _publish_status(self, status: str) -> None:
        if status == self._last_status:
            return
        self._last_status = status
        self.status_pub.publish(String(data=status))
        self.get_logger().info(f"Hardware status: {status}")

    def destroy_node(self):
        self._shutdown.set()
        self._drain_commands()
        if self._worker.is_alive():
            self._worker.join(timeout=1.0)
        if self.driver is not None:
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
