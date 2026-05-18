import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def _materialize_world(pkg_share_path: str) -> str:
    """Substitute @MODELS_DIR@ in the world template with the absolute
    package-share models directory, then write the result to a temp file
    that Gazebo can load. Keeps the SDF in the repo machine-agnostic."""
    models_dir = os.path.join(pkg_share_path, "models")
    template_path = os.path.join(models_dir, "crazyflie_world.sdf")
    with open(template_path, "r") as f:
        world_xml = f.read().replace("@MODELS_DIR@", models_dir)
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".sdf", delete=False, prefix="crazyflie_world_"
    )
    tmp.write(world_xml)
    tmp.close()
    return tmp.name


def generate_launch_description():
    pkg_share_path = get_package_share_directory('crazyflie_gazebo')
    world_file_path = _materialize_world(pkg_share_path)

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_file_path}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_share_path, 'config', 'ros_gz_crazyflie_bridge.yaml'),
        }],
        output='screen'
    )

    motor_control = Node(
        package='cpp_controllers',
        executable='motor_control_node',
        output='screen'
    )

    keyboard_control = Node(
        package = "crazyflie_gazebo",
        executable = "control_services",
        output = "screen"
    )


    return LaunchDescription([
        gz_sim,
        motor_control,
        bridge,
    ])

