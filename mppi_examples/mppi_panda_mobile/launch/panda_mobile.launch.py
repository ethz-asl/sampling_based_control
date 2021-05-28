import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument as decl_arg

def generate_launch_description():
    arguments = []
    nodes = []

    pkg_root = get_package_share_directory("mppi_panda_mobile")
    robot_description_path = os.path.join(
        pkg_root,
        "resources", "panda",
        "panda_mobile.urdf.xacro",
    )
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {"robot_description": robot_description_config.toxml()}

    rviz_config_dir = os.path.join(pkg_root, 'config', 'config.rviz')

    visualization_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen')
    nodes.append(visualization_node)

    state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description]
    )
    nodes.append(state_publisher_node)

    controller_config = os.path.join(pkg_root, 'config', 'config.yaml')
    ros_params = os.path.join(pkg_root, 'config', 'params.yaml')
    controller_node = Node(
        package='mppi_panda_mobile',
        executable="panda_mobile_control",
        name="panda_mobile_control",
        output="screen",
        parameters=[ros_params,
                    robot_description,
                    {"config_file": controller_config}],
        prefix=['xterm -e gdb -ex run --args']
    )
    nodes.append(controller_node)

    interactive_reference_node = Node(
        package="mppi_panda",
        executable="interactive_marker.py",
        name="interactive_reference",
        parameters=[{'marker_server_name': 'reference_marker_server',
                     'subscribe_initial_pose': True,
                     'initial_pose_topic': '/end_effector',
                     'frame_id': 'world',
                     'target_pose_topic': '/end_effector_pose_desired'}]

    )
    nodes.append(interactive_reference_node)

    return LaunchDescription(nodes)
