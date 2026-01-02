import os
import sys
import subprocess
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Get package share directories
    tb3_desc_share = get_package_share_directory('turtlebot3_description')
    world_share = get_package_share_directory('turtlebot3_football')
    gazebo_share = get_package_share_directory('gazebo_ros')

    # World file
    world_file = os.path.join(world_share, 'worlds', 'football_field.world')

    # URDF/Xacro file
    urdf_xacro = os.path.join(tb3_desc_share, 'urdf', 'turtlebot3_waffle_pi.urdf.xacro')
    urdf_file = os.path.join(tb3_desc_share, 'urdf', 'turtlebot3_waffle_pi.urdf')

    # Prefer xacro if present
    if os.path.exists(urdf_xacro):
        urdf_path = urdf_xacro
        use_xacro = True
    elif os.path.exists(urdf_file):
        urdf_path = urdf_file
        use_xacro = False
    else:
        print(f'❌ CRITICAL ERROR: URDF not found at {urdf_xacro} or {urdf_file}. Please install turtlebot3 packages.')
        sys.exit(1)

    print(f'✅ SUCCESS: Found URDF at {urdf_path}')

    # Process xacro if needed
    if use_xacro:
        try:
            robot_desc = subprocess.check_output(['xacro', urdf_path]).decode()
        except Exception as e:
            print(f'❌ CRITICAL ERROR: Failed to process xacro: {e}')
            sys.exit(1)
    else:
        with open(urdf_path, 'r') as f:
            robot_desc = f.read()

    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}]
    )

    # Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'turtlebot3_waffle_pi',
            '-x', '-1.5', '-y', '0.0', '-z', '0.15'
        ],
        output='screen'
    )

    return [gazebo_launch, robot_state_publisher, spawn_entity]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])