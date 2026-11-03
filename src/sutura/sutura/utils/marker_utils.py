from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from sutura.utils.data_models import StitchTarget


def stitch_targets_to_marker_array(
    targets: list[StitchTarget],
    frame_id: str = "camera_frame",
    stamp: Time | None = None,
) -> MarkerArray:
    """Converts StitchTarget objects (in mm) into ROS 2 MarkerArray (in meters)."""
    marker_array = MarkerArray()

    for i, stitch in enumerate(targets):
        en_pt = stitch.entry_pose.position
        ex_pt = stitch.exit_pose.position

        # Convert mm to meters for RViz 2
        en_x_m, en_y_m, en_z_m = en_pt.x / 1000.0, en_pt.y / 1000.0, en_pt.z / 1000.0
        ex_x_m, ex_y_m, ex_z_m = ex_pt.x / 1000.0, ex_pt.y / 1000.0, ex_pt.z / 1000.0

        # --- Entry Point (Red Sphere: 2mm diameter = 0.002m) ---
        m_entry = Marker()
        m_entry.header.frame_id = frame_id
        if stamp:
            m_entry.header.stamp = stamp
        m_entry.ns = "stitch_entry_points"
        m_entry.id = i * 3 + 1
        m_entry.type = Marker.SPHERE
        m_entry.action = Marker.ADD
        m_entry.pose.position.x = en_x_m
        m_entry.pose.position.y = en_y_m
        m_entry.pose.position.z = en_z_m
        m_entry.pose.orientation = stitch.entry_pose.orientation.to_ros_quaternion()
        m_entry.scale.x, m_entry.scale.y, m_entry.scale.z = 0.002, 0.002, 0.002
        m_entry.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        marker_array.markers.append(m_entry)

        # --- Exit Point (Green Sphere: 2mm diameter = 0.002m) ---
        m_exit = Marker()
        m_exit.header.frame_id = frame_id
        if stamp:
            m_exit.header.stamp = stamp
        m_exit.ns = "stitch_exit_points"
        m_exit.id = i * 3 + 2
        m_exit.type = Marker.SPHERE
        m_exit.action = Marker.ADD
        m_exit.pose.position.x = ex_x_m
        m_exit.pose.position.y = ex_y_m
        m_exit.pose.position.z = ex_z_m
        m_exit.pose.orientation = stitch.exit_pose.orientation.to_ros_quaternion()
        m_exit.scale.x, m_exit.scale.y, m_exit.scale.z = 0.002, 0.002, 0.002
        m_exit.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        marker_array.markers.append(m_exit)

        # --- Connection Line (Yellow Line: 0.5mm thickness = 0.0005m) ---
        m_line = Marker()
        m_line.header.frame_id = frame_id
        if stamp:
            m_line.header.stamp = stamp
        m_line.ns = "stitch_lines"
        m_line.id = i * 3 + 3
        m_line.type = Marker.LINE_STRIP
        m_line.action = Marker.ADD
        m_line.scale.x = 0.0005
        m_line.color = ColorRGBA(r=1.0, g=0.9, b=0.0, a=1.0)
        m_line.points = [
            Point(x=en_x_m, y=en_y_m, z=en_z_m),
            Point(x=ex_x_m, y=ex_y_m, z=ex_z_m),
        ]
        marker_array.markers.append(m_line)

    return marker_array