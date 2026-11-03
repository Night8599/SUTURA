import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray

from sutura.planning.patterns.base_pattern import BasePatternGenerator
from sutura.planning.patterns.linear_pattern import LinearPatternGenerator
from sutura.utils.data_models import GridMap2D, StitchTarget
from sutura.utils.marker_utils import stitch_targets_to_marker_array


class PlanningNode(Node):
    """SUTURA Planning Node (Pattern Generator Phase with Debug Timer)."""

    def __init__(self) -> None:
        super().__init__('planning_node')

        # ----------------------------------------------------------------------
        # 1. Parameter Declaration & Loading
        # ----------------------------------------------------------------------
        self.declare_parameter('pattern.type', 'linear')
        self.declare_parameter('pattern.margin_mm', 3.0)
        self.declare_parameter('pattern.spacing_mm', 5.0)
        self.declare_parameter('pattern.entry_offset_mm', 4.0)
        self.declare_parameter('pattern.default_entry_angle_deg', 90.0)

        margin_mm = self.get_parameter('pattern.margin_mm').get_parameter_value().double_value
        spacing_mm = self.get_parameter('pattern.spacing_mm').get_parameter_value().double_value
        entry_offset_mm = self.get_parameter('pattern.entry_offset_mm').get_parameter_value().double_value
        entry_angle = self.get_parameter('pattern.default_entry_angle_deg').get_parameter_value().double_value

        self.pattern_generator: BasePatternGenerator = LinearPatternGenerator(
            margin_mm=margin_mm,
            spacing_mm=spacing_mm,
            entry_offset_mm=entry_offset_mm,
            entry_angle_deg=entry_angle,
        )

        # ----------------------------------------------------------------------
        # 2. Publisher com Transient Local QoS (Garante entrega ao RViz 2)
        # ----------------------------------------------------------------------
        marker_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/sutura/visualization/pattern', marker_qos
        )

        # ----------------------------------------------------------------------
        # 3. Timer Temporário para Teste/Visualização no RViz 2 (2.0 segundos)
        # ----------------------------------------------------------------------
        self.timer = self.create_timer(2.0, self._debug_timer_callback)
        self.get_logger().info("SUTURA Planning Node rodando em modo de teste contínuo...")

    def _debug_timer_callback(self) -> None:
        """Gera um GridMap2D simulado e publica os marcadores no RViz 2."""
        grid_map = GridMap2D(resolution_mm=0.5, width_cells=100, height_cells=100, frame_id="camera_frame")

        # Injeta uma ferida simulada no centro da grade
        grid_map.wound_mask[45:55, 20:80] = 1

        # Processa e publica
        self.process_grid_map(grid_map)

    def process_grid_map(self, grid_map: GridMap2D) -> list[StitchTarget]:
        stitch_targets = self.pattern_generator.generate_pattern(grid_map)
        if not stitch_targets:
            self.get_logger().warn("Pattern generator não retornou alvos válidos.")
            return []

        # Passa o timestamp do nó para os marcadores
        markers = stitch_targets_to_marker_array(
            stitch_targets,
            frame_id=grid_map.frame_id,
            stamp=self.get_clock().now().to_msg()
        )
        self.marker_pub.publish(markers)
        self.get_logger().info(f"Publicados {len(markers.markers)} marcadores no RViz 2.")
        return stitch_targets


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()