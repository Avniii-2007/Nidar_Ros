import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('nidar_gazebo')
    models_dir = os.path.join(pkg_share, 'models')
    world_path = os.path.join(pkg_share, 'worlds', 'nidar_arena.sdf')

    # Ensure DISPLAY and XAUTHORITY are set for SSH sessions if opening GUI
    current_display = os.environ.get('DISPLAY', ':0')
    current_xauth = os.environ.get('XAUTHORITY', '/run/user/1000/gdm/Xauthority')

    env_display = SetEnvironmentVariable(name='DISPLAY', value=current_display)
    env_xauth = SetEnvironmentVariable(name='XAUTHORITY', value=current_xauth)

    # Environment variable so Gazebo locates the drone and custom models
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_dir, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Set to true for headless (server only), or false to display the full 3D GUI'
    )

    # 1. Full 3D GUI Mode (Default)
    gz_sim_gui = ExecuteProcess(
        condition=UnlessCondition(LaunchConfiguration('headless')),
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    # 2. Headless Server Mode (When headless:=true)
    gz_sim_headless = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('headless')),
        cmd=['gz', 'sim', '-r', '-s', world_path],
        output='screen'
    )

    return LaunchDescription([
        env_display,
        env_xauth,
        gz_resource_path,
        headless_arg,
        gz_sim_gui,
        gz_sim_headless
    ])
