import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess, SetEnvironmentVariable

from launch_ros.actions import Node

def generate_launch_description():
    # Use the absolute path to the world file
    pkg_path = get_package_share_directory('crazyflie_gazebo')
    
    world_file_path = pkg_path + "/models/crazyflie/crazyflie_world.sdf"

    # Get path to the ros gazebo package
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_model_path = os.getenv('GZ_SIM_RESOURCE_PATH')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_file_path}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project, 'config', 'ros_gz_crazyflie_bridge.yaml'),
        }],
        output='screen'
    )

    control = Node(
        package='crazyflie_gazebo',
        executable='control_services',
        output='screen',
        parameters=[
            {'hover_height': 0.5},
            {'robot_prefix': '/crazyflie'},
            {'incoming_twist_topic': '/cmd_vel'},
            {'max_ang_z_rate': 0.4},
        ]
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        control
    ])

