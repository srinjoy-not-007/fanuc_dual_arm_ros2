import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Load URDF
    urdf_file = os.path.join(get_package_share_directory('dexsent_description'), 'urdf', 'fanuc_with_gripper.xacro')
    doc = xacro.process_file(urdf_file)
    robot_desc = {'robot_description': doc.toxml()}
    
    # Load SRDF
    srdf_file = os.path.join(get_package_share_directory('dexsent_moveit_config'), 'config', 'fanuc_crx10ial_gripper.srdf')
    with open(srdf_file, 'r') as f:
        semantic_content = f.read()
    robot_desc_sem = {'robot_description_semantic': semantic_content}
    
    # Load Controllers
    ctrl_file = os.path.join(get_package_share_directory('dexsent_moveit_config'), 'config', 'ros2_controllers.yaml')

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[robot_desc]),
        Node(package='controller_manager', executable='ros2_control_node', parameters=[robot_desc, ctrl_file], output='screen'),
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'], output='screen'),
        Node(package='controller_manager', executable='spawner', arguments=['manipulator_controller'], output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', parameters=[robot_desc, robot_desc_sem])
    ])
