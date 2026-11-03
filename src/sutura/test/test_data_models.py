import numpy as np
import pytest
from sutura.utils.data_models import (
    GridMap2D,
    Orientation3D,
    Point3D,
    Pose3D,
    StitchTarget,
)


def test_point_3d():
    p1 = Point3D(x=10.5, y=-5.0, z=2.3)
    
    # NumPy conversion
    np_arr = p1.to_numpy()
    p2 = Point3D.from_numpy(np_arr)
    assert p1 == p2

    # ROS Point conversion
    ros_msg = p1.to_ros_point()
    p3 = Point3D.from_ros_point(ros_msg)
    assert p1 == p3


def test_orientation_3d():
    o1 = Orientation3D(x=0.0, y=0.0, z=0.7071, w=0.7071)
    
    # Surface normal conversion
    normal = np.array([0.0, 1.0, 1.0], dtype=np.float64)
    o_normal = Orientation3D.from_surface_normal(normal)
    assert isinstance(o_normal, Orientation3D)

    # ROS Quaternion conversion
    ros_q = o1.to_ros_quaternion()
    o2 = Orientation3D.from_ros_quaternion(ros_q)
    assert o1 == o2


def test_pose_3d():
    pos = Point3D(x=15.0, y=20.0, z=-3.5)
    orient = Orientation3D(x=0.0, y=0.0, z=0.0, w=1.0)
    pose1 = Pose3D(position=pos, orientation=orient)

    # ROS Pose conversion
    ros_pose = pose1.to_ros_pose()
    pose2 = Pose3D.from_ros_pose(ros_pose)
    assert pose1 == pose2


def test_stitch_target():
    entry_p = Pose3D(position=Point3D(x=10.0, y=4.0, z=0.0))
    exit_p = Pose3D(position=Point3D(x=10.0, y=-4.0, z=0.0))

    stitch = StitchTarget(stitch_index=1, entry_pose=entry_p, exit_pose=exit_p)
    assert stitch.stitch_index == 1
    assert stitch.entry_pose.position.x == 10.0
    assert stitch.exit_pose.position.y == -4.0


def test_grid_map_2d_precision():
    # 1. Instanciação com resolução de 0.5 mm/célula
    grid = GridMap2D(resolution_mm=0.5, width_cells=100, height_cells=100)

    # 2. Teste de conversão Mundo -> Grade -> Mundo (Centro da Célula)
    x_target, y_target = 12.3, 8.7
    coords = grid.world_to_grid(x_target, y_target)
    assert coords is not None, "Coordenada mundial fora dos limites do grid!"

    row, col = coords
    x_center, y_center, _ = grid.grid_to_world(row, col)

    # O erro entre o ponto real e o centro da célula deve ser menor que meia célula (0.25 mm)
    assert abs(x_target - x_center) <= (grid.resolution_mm / 2.0)
    assert abs(y_target - y_center) <= (grid.resolution_mm / 2.0)

    # 3. Teste de Superfície com Plano Inclinado Contínuo (z = 1.0 * x)
    # Para z = x, dz/dx = 1.0 e dz/dy = 0.0.
    # O vetor normal teórico é (-1, 0, 1) / sqrt(2) ~= (-0.7071, 0.0, 0.7071)
    for r in range(grid.height_cells):
        for c in range(grid.width_cells):
            x_world, _, _ = grid.grid_to_world(r, c)
            grid.elevation[r, c] = x_world  # Inclinação de 45 graus ao longo de X

    # Avalia a normal em um ponto central da malha (row=50, col=50)
    normal: Orientation3D = grid.compute_surface_normal(50, 50)
    
    # Converte a orientação calculada de volta para o vetor normal no espaço 3D
    q = normal.to_numpy()  # [x, y, z, w]
    # Matriz de rotação aplicada ao vetor Z local [0, 0, 1]
    nz = 1.0 - 2.0 * (q[0]**2 + q[1]**2)
    nx = 2.0 * (q[0]*q[2] + q[3]*q[1])

    # Tolerância estrita para validação matemática da derivada
    assert pytest.approx(nx, abs=1e-2) == -0.7071
    assert pytest.approx(nz, abs=1e-2) == 0.7071


def test_grid_map_roi_and_elevation():
    grid = GridMap2D(resolution_mm=0.5, width_cells=50, height_cells=50)

    # Simula uma ferida retangular no centro do pad
    grid.elevation[20:30, 20:30] = -2.0  # Depressão de 2mm da ferida
    grid.wound_mask[20:30, 20:30] = 1    # Máscara ROI

    x_wound, y_wound, _ = grid.grid_to_world(25, 25)
    assert grid.is_wound_at_world(x_wound, y_wound) is True
    assert grid.get_elevation_at_world(x_wound, y_wound) == -2.0

    x_healthy, y_healthy, _ = grid.grid_to_world(5, 5)
    assert grid.is_wound_at_world(x_healthy, y_healthy) is False
    assert grid.get_elevation_at_world(x_healthy, y_healthy) == 0.0