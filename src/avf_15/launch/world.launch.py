import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ros_gz_sim.actions import GzServer
from launch.actions import AppendEnvironmentVariable

def generate_launch_description():
    pkg_share = get_package_share_directory('avf_15')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    world_path = os.path.join(pkg_share, 'world', 'warehouse.sdf')
    bridge_clock_global_path = os.path.join(pkg_share, 'config', 'bridge_clock.yaml')
    # map_path = os.path.join(pkg_share, 'maps', 'wearhouse_full.yaml')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    gz_server = GzServer(
        world_sdf_file=world_path,
        container_name='ros_gz_container',
        create_own_container=False,
        use_composition=False,
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        parameters=[{'config_file': bridge_clock_global_path}],
    )

    return LaunchDescription([
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.dirname(pkg_share),
        ),

        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file',
        ),
        DeclareLaunchArgument(
            name='use_rviz',
            default_value='false',
        ),
        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value='true',
        #     description='Use Gazebo simulation clock',
        # ),
        ExecuteProcess(
            cmd=['gz', 'sim', '-g'],
            output='screen',
        ),

        gz_server,
        clock_bridge,

        # map_server_node,
        # map_lifecycle_manager_node,

        # rviz_node,
    ])
