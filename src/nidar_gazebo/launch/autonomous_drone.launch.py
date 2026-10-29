import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nidar_gazebo')
    models_dir = os.path.join(pkg_share, 'models')
    world_path = os.path.join(pkg_share, 'worlds', 'nidar_arena.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'nidar_gcs.rviz')

    current_display = os.environ.get('DISPLAY', '')
    if not current_display and os.path.exists('/tmp/.X11-unix/X0'):
        current_display = ':0'

    current_xauth = os.environ.get('XAUTHORITY', '')
    if not current_xauth:
        import glob
        m = glob.glob('/run/user/1000/.mutter-Xwaylandauth.*')
        current_xauth = m[0] if m else '/home/shubham/.Xauthority'

    default_headless = 'true'

    env_display = SetEnvironmentVariable(name='DISPLAY', value=current_display)
    env_xauth = SetEnvironmentVariable(name='XAUTHORITY', value=current_xauth)
    env_egl = SetEnvironmentVariable(
        name='__EGL_VENDOR_LIBRARY_FILENAMES',
        value='/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
    )
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_dir, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value=default_headless,
        description='Set to true for headless server mode, or false for 3D GUI'
    )
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Set to true for drone to auto-takeoff and begin mission on startup'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Set to true to launch RViz2 3D visualization'
    )

    # 1. Gazebo Sim with 3D Arena & Multicopter
    gz_sim_gui = ExecuteProcess(
        condition=UnlessCondition(LaunchConfiguration('headless')),
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    gz_sim_headless = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('headless')),
        cmd=['gz', 'sim', '-r', '-s', world_path],
        output='screen'
    )

    # 2. ROS-Gazebo Transport Bridge (LiDAR 1, LiDAR 2 ToF, Camera, IMU, Odometry, Cmd_vel, Clock)
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen'
    )

    # 3. Static TF Transforms
    # Base link -> LiDAR 1 (Horizontal 360 LaserScan)
    tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0.08', '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'],
        output='screen'
    )

    # Base link -> LiDAR 2 (Downward ToF Altimeter)
    tf_tf02 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '-0.05', '--yaw', '0', '--pitch', '1.5708', '--roll', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'tf02_link'],
        output='screen'
    )

    # Base link -> Forward RGB Camera
    tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.18', '--y', '0', '--z', '0.02', '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
        output='screen'
    )

    # 4. Real-time 2D SLAM Mapping Node (slam_toolbox)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_config,
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # 5. Dual-LiDAR Drone Autonomy Controller Node
    autonomy_node = Node(
        package='nidar_gazebo',
        executable='drone_autonomy_controller.py',
        parameters=[{
            'auto_start': LaunchConfiguration('auto_start'),
            'target_altitude': 1.20,
            'max_horizontal_speed': 0.45,
            'wall_safety_dist': 0.55,
            'emergency_dist': 0.25,
            'use_sim_time': True
        }],
        output='screen'
    )

    # 6. Survivor Detection & Localization Node
    survivor_node = Node(
        package='nidar_gazebo',
        executable='survivor_detector.py',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 7. Ground Control Station (GCS) & Mission Planner Web Server Node (http://localhost:8080)
    gcs_node = Node(
        package='nidar_gazebo',
        executable='mission_planner_gcs.py',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 8. RViz2 (Optional)
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        env_display,
        env_xauth,
        env_egl,
        gz_resource_path,
        headless_arg,
        auto_start_arg,
        rviz_arg,
        gz_sim_gui,
        gz_sim_headless,
        ros_gz_bridge_node,
        tf_laser,
        tf_tf02,
        tf_camera,
        slam_node,
        autonomy_node,
        survivor_node,
        gcs_node,
        rviz_node
    ])
