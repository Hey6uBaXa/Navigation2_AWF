import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('avf_15')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_spawn_model_launch_source = os.path.join(ros_gz_sim_share, "launch", "gz_spawn_model.launch.py")
    default_model_path = os.path.join(pkg_share, 'src', 'description', 'avf_15_description_sim.sdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    world_path = os.path.join(pkg_share, 'world', 'warehouse.sdf')
    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge_config.yaml')

    # Путь к пакету навигации Nav2
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    #Определение папки для СЛАМ
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    #Путь к вашему файлу параметров nav2_params
    default_nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    #Livox condif
    livox_laserscan_config = os.path.join(pkg_share, 'config', 'livox_laserscan.yaml')
    #Save map
    default_map_path = os.path.join(pkg_share,'maps','wearhouse_full.yaml')
    #Graph
    graph_filepath = os.path.join(pkg_share,'graphs','route.geojson')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': ParameterValue(
                    Command(['xacro ', LaunchConfiguration('model')]), 
                    value_type=str
                )
            }, 
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
    #     condition=UnlessCondition(LaunchConfiguration('gui'))
    # )

    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     condition=IfCondition(LaunchConfiguration('gui'))
    # )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    gz_server = GzServer(
        world_sdf_file=world_path,
        container_name='ros_gz_container',
        create_own_container='True',
        use_composition='True',
    )
    
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        container_name='ros_gz_container',
        create_own_container='False',
        use_composition='True',
    )

    front_camera_bridge_image = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='front_bridge_gz_ros_camera_image',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['/front_depth_camera/image'],
    )

    front_camera_bridge_depth = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='front_bridge_gz_ros_camera_depth',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['/front_depth_camera/depth_image'],
    )

    rear_camera_bridge_image = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='rear_bridge_gz_ros_camera_image',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['/rear_depth_camera/image'],
    )

    rear_camera_bridge_depth = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='rear_bridge_gz_ros_camera_depth',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['/rear_depth_camera/depth_image'],
    )
    
    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            'world': 'warehouse',
            'topic': '/robot_description',
            'entity_name': 'avf_15',
            'x': '1.0',
            'y': '1.0',
            'z': '0.65',
            # 'yaw': '3.14159',
        }.items(),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    tricycle_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tricycle_controller"],
        output="screen",
    )

    fork_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fork_controller"],
        output="screen",
    )
    
    livox_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='livox_and_sensors_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    livox_scan_low = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='livox_scan_low',
        output='screen',
        parameters=[livox_laserscan_config,{'use_sim_time': LaunchConfiguration('use_sim_time')},],
        remappings=[
            ('cloud_in', '/livox/points'),
            ('scan', '/livox/scan_low'),
        ],
    )

    livox_scan_middle = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='livox_scan_middle',
        output='screen',
        parameters=[livox_laserscan_config, 
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},],
        remappings=[
            ('cloud_in', '/livox/points'),
            ('scan', '/livox/scan_middle'),
        ],
    )

    livox_scan_high = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='livox_scan_high',
        output='screen',
        parameters=[livox_laserscan_config,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},],
        remappings=[
            ('cloud_in', '/livox/points'),
            ('scan', '/livox/scan_high'),
        ],
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_toolbox_share,
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': LaunchConfiguration('nav2_params'),
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
        }.items(),
    )

    initial_pose = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/initialpose',
            'geometry_msgs/msg/PoseWithCovarianceStamped',
            '{header: {frame_id: map}, pose: {pose: {position: {x: 0, y: 0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}'
        ],
        shell=False
    )

    #Включение официального запуска навигации Nav2 с вашими параметрами
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('nav2_params'),
            'autostart': 'true'
        }.items()
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': default_map_path,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('nav2_params'),
            'graph' : graph_filepath,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='True', description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        
        # ИСПРАВЛЕНО: Аргумент пути для файла параметров навигации
        DeclareLaunchArgument(name='nav2_params', default_value=default_nav2_params_path, description='Absolute path to nav2 params file'),
        
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'),
        robot_state_publisher_node,
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        rviz_node,
        gz_server,
        ros_gz_bridge,
        front_camera_bridge_image,
        front_camera_bridge_depth,
        rear_camera_bridge_image,
        rear_camera_bridge_depth,
        spawn_entity,
        #livox_bridge,
        robot_localization_node,

        #ROS2 Controller 
        joint_state_broadcaster_spawner,
        tricycle_controller_spawner,
        fork_controller_spawner,

        #Livox
        livox_scan_high,
        livox_scan_low,
        livox_scan_middle,

        # SLAM NAV2
        slam_toolbox_launch,
        nav2_navigation_launch,


        # # NAV2 MAP
        # initial_pose,
        # nav2_bringup_launch,
        
    ])
