import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import LogInfo
import tempfile
from launch.substitutions import PythonExpression
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
import yaml
"""
Loads a base Gazebo-ROS bridge YAML file and rewrites the topic names to include a robot-specific namespace.
Also updates message types from TwistStamped to Twist, if needed.
This utility adjusts TurtleBot3’s default bridge YAML config to support namespaces for ROS2 and gz.
The provided YAML file is reused but modified at runtime to prefix each topic with a namespace.
This is essential in multi-robot simulation, where each robot must publish and subscribe to its
own isolated set of topics (e.g., /tb0_1/cmd_vel) to avoid collisions.
It also normalizes message types (e.g., TwistStamped → Twist) where needed for compatibility.


Args:
    base_yaml_path (str): Path to the base YAML bridge configuration.
    namespace (str): Namespace prefix to apply to all ROS and Gazebo topic names.

Returns:
    str: Path to the modified, namespaced YAML file saved in /tmp.
"""

def create_namespaced_bridge_yaml(base_yaml_path, namespace):
    """Create a temporary namespaced bridge YAML for ros_gz_bridge."""
    with open(base_yaml_path, 'r') as f:
        bridges = yaml.safe_load(f)

    if namespace and not namespace.endswith('/'):
        namespace_with_slash = namespace + '/'
    else:
        namespace_with_slash = namespace

    namespaced_bridges = []
    for bridge in bridges:
        if bridge['ros_topic_name'] not in ['clock']:
            bridge['ros_topic_name'] = f"{namespace_with_slash}{bridge['ros_topic_name']}"
        if bridge['gz_topic_name'] not in ['clock']:
            bridge['gz_topic_name'] = f"{namespace_with_slash}{bridge['gz_topic_name']}"
        namespaced_bridges.append(bridge)

    output_path = f"/tmp/{namespace.strip('/')}_bridge.yaml"
    with open(output_path, 'w') as f:
        yaml.dump(namespaced_bridges, f)

    return output_path

def generate_rviz_config(robot_name, base_config_path):
    # Read the base RViz config
    with open(base_config_path, 'r') as f:
        config = f.read()

    # Replace placeholders
    config = config.replace('<ROBOT_NAME>', robot_name)

    # Use system temp directory
    temp_dir = tempfile.gettempdir()
    output_config_path = os.path.join(temp_dir, f'{robot_name}_rviz_config.rviz')

    with open(output_config_path, 'w') as f:
        f.write(config)

    return output_config_path

def generate_launch_description():
    pkg_share = get_package_share_directory('avf_15')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    gz_spawn_model_launch_source = os.path.join(
        ros_gz_sim_share, "launch", "gz_spawn_model.launch.py"
    )
    default_model_path = os.path.join(pkg_share, 'src', 'description', 'avf_15_description_sim.sdf')
    default_nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    livox_laserscan_config = os.path.join(pkg_share, 'config', 'livox_laserscan.yaml')
    default_map_path = os.path.join(pkg_share, 'maps', 'wearhouse_full.yaml')
    robot_bridge_config_path = os.path.join(pkg_share, 'config', 'bridge_robot.yaml')
    robot_name = LaunchConfiguration('robot_name')
    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    rviz_template_path = os.path.join(pkg_share, 'rviz', 'config.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': ParameterValue(
                    Command([
                        'xacro ',
                        LaunchConfiguration('model'),
                        ' robot_name:=',
                        robot_name,
                    ]),
                    value_type=str
                ),
                #'frame_prefix': [robot_name, '/'],
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            # ('joint_states', 'joint_states'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ],
    )

    pallet_detector_node = Node(
        package='avf_15_pallet_perception',
        executable='pallet_detector_node',
        name='pallet_detector_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'cloud_topic': 'rear_depth_camera/points',
                # 'base_frame': [robot_name, '/base_link'],
                'pose_topic': 'detected_dock_pose',
                'found_topic': 'pallet_detector/pallet_found',
                'debug_cloud_topic': 'pallet_detector/debug_band_cloud',
            }
        ],
        remappings=tf_remappings
    )
    rviz_config = generate_rviz_config(f'{robot_name}', rviz_template_path)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        #namespace=robot_name,
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        remappings=tf_remappings
    )
    # def launch_rviz(context, rviz_template_path):
    #     robot_name = LaunchConfiguration('robot_name').perform(context)
    #     rviz_config = generate_rviz_config(robot_name, rviz_template_path)

    #     return [
    #         Node(
    #             package='rviz2',
    #             executable='rviz2',
    #             name='rviz2',
    #             output='screen',
    #             arguments=['-d', rviz_config],
    #             condition=IfCondition(LaunchConfiguration('use_rviz')),
    #             remappings=[
    #                 ('/tf', 'tf'),
    #                 ('/tf_static', 'tf_static'),
    #             ],
    #         )
    #     ]
    

    robot_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[
            {
                'config_file': robot_bridge_config_path,
                'expand_gz_topic_names': True,
                'use_sim_time': True,
            }
        ],
    )

    front_camera_bridge_image = Node(
        package='ros_gz_image', executable='image_bridge',
        name='front_bridge_gz_ros_camera_image', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['front_depth_camera/image'],
    )
    front_camera_bridge_depth = Node(
        package='ros_gz_image', executable='image_bridge',
        name='front_bridge_gz_ros_camera_depth', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['front_depth_camera/depth_image'],
    )
    rear_camera_bridge_image = Node(
        package='ros_gz_image', executable='image_bridge',
        name='rear_bridge_gz_ros_camera_image', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['rear_depth_camera/image'],
    )
    rear_camera_bridge_depth = Node(
        package='ros_gz_image', executable='image_bridge',
        name='rear_bridge_gz_ros_camera_depth', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['rear_depth_camera/depth_image'],
    )

    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            'world': 'warehouse',
            'topic': 'robot_description',
            'entity_name': robot_name,
            'x': LaunchConfiguration('spawn_x'),
            'y': LaunchConfiguration('spawn_y'),
            'z': '0.65',
        }.items(),
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", ["/", robot_name, "/controller_manager"],
            ],
    )

    # Пример для tricycle_controller_spawner
    tricycle_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "tricycle_controller",
            # Явно указываем менеджер относительно корня, используя переменную robot_name
            "--controller-manager", ["/", robot_name, "/controller_manager"],
        ],
        output="screen",
        remappings=tf_remappings
    )

    # Пример для fork_controller_spawner
    fork_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "fork_controller",
            "--controller-manager", ["/", robot_name, "/controller_manager"],
        ],
        remappings=tf_remappings
        # Если эта нода ВНУТРИ GroupAction с PushRosNamespace, 
        # убедитесь, что вы не добавляете лишних ремаппингов здесь.
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'ekf.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=tf_remappings
    )
    livox_scan_low = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='livox_scan_low',
        output='screen',
        parameters=[
            livox_laserscan_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        remappings=[
            ('cloud_in', 'livox/points'),
            ('scan', 'livox/scan_low'),
            *tf_remappings,
        ],
    )
    livox_scan_middle = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='livox_scan_middle',
        output='screen',
        parameters=[
            livox_laserscan_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        remappings=[
            ('cloud_in', 'livox/points'),
            ('scan', 'livox/scan_middle'),
            *tf_remappings,
        ],
    )
    livox_scan_high = Node(
        package='pointcloud_to_laserscan', 
        executable='pointcloud_to_laserscan_node',
        name='livox_scan_high', 
        output='screen',
        parameters=[livox_laserscan_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('cloud_in', 'livox/points'), ('scan', 'livox/scan_high'), *tf_remappings],
    )

    topic_name = PythonExpression([
    "'/' + '",
    robot_name,
    "' + '/initialpose'"
    ])
    initial_pose = ExecuteProcess(
        cmd=[
            'ros2',
            'topic',
            'pub',
            '--once',
            topic_name,
            'geometry_msgs/msg/PoseWithCovarianceStamped',
            (
                '{header: {frame_id: map}, '
                'pose: {pose: {'
                'position: {x: 0.0, y: 0.0, z: 0.0}, '
                'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}'
                '}, '
                'covariance: ['
                '0.25, 0, 0, 0, 0, 0, '
                '0, 0.25, 0, 0, 0, 0, '
                '0, 0, 0, 0, 0, 0, '
                '0, 0, 0, 0, 0, 0, '
                '0, 0, 0, 0, 0, 0, '
                '0, 0, 0, 0, 0, 0.0685'
                ']}}'
            ),
        ],
        shell=False,
        output='screen',
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': default_map_path,
            'use_namespace': 'true',
            'namespace': robot_name,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('nav2_params'),
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'False',
            'slam': 'False',
            'use_localization': 'True',
        }.items(),
    )

    robot_group = GroupAction([
        PushRosNamespace(robot_name),

        robot_state_publisher_node,
        robot_localization_node,
        spawn_entity,
        robot_ros_gz_bridge,
        livox_scan_high,
        livox_scan_low,
        livox_scan_middle,
        
        # pallet_detector_node,

        joint_state_broadcaster_spawner,
        tricycle_controller_spawner,
        fork_controller_spawner,
        
        front_camera_bridge_image,
        front_camera_bridge_depth,
        rear_camera_bridge_image,
        rear_camera_bridge_depth,
        rviz_node,

    ])

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig',default_value=rviz_template_path, description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True'),
        DeclareLaunchArgument(name='robot_name', default_value='robot01'),
        DeclareLaunchArgument(name='spawn_x', default_value='1.0'),
        DeclareLaunchArgument(name='spawn_y', default_value='1.0'),
        DeclareLaunchArgument(name='use_rviz', default_value='false'),
        DeclareLaunchArgument(name='nav2_params', default_value=default_nav2_params_path, description='Absolute path to nav2 params file'),
        LogInfo(msg=["NAV2 PARAMS FILE: ", LaunchConfiguration('nav2_params')]),
        
        
        robot_group,
        nav2_bringup_launch,
        initial_pose,
        
    ])