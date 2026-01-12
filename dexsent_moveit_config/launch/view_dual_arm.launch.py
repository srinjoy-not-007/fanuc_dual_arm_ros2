import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return {}

def generate_launch_description():
    # 1. Load Robot Description
    urdf_file = os.path.join(get_package_share_directory('dexsent_description'), 'urdf', 'dual_arm.xacro')
    doc = xacro.process_file(urdf_file)
    robot_description = {'robot_description': doc.toxml()}

    # 2. Load SRDF
    srdf_file = os.path.join(get_package_share_directory('dexsent_moveit_config'), 'config', 'dual_arm.srdf')
    with open(srdf_file, 'r') as f:
        semantic_content = f.read()
    robot_description_semantic = {'robot_description_semantic': semantic_content}

    # 3. Load Configs
    kinematics_yaml = load_yaml('dexsent_moveit_config', 'config/kinematics.yaml')
    ompl_planning_yaml = load_yaml('dexsent_moveit_config', 'config/ompl_planning.yaml')
    fake_controllers_yaml = load_yaml('dexsent_moveit_config', 'config/fake_controllers.yaml')

    # *** FORCE THE CONTROLLER MANAGER PARAMETER ***
    # We add this directly to the dictionary to ensure it cannot be ignored
    if fake_controllers_yaml is None:
        fake_controllers_yaml = {}
    fake_controllers_yaml['moveit_controller_manager'] = 'moveit_fake_controller_manager/MoveItFakeControllerManager'

    # 4. Move Group Parameters
    move_group_params = [
        robot_description,
        robot_description_semantic,
        {'robot_description_kinematics': kinematics_yaml},
        {'planning_pipelines': ['ompl']},
        {'ompl': ompl_planning_yaml},
        fake_controllers_yaml, # Now contains the manager setting inside it
        {'moveit_manage_controllers': True},
        {'use_sim_time': True},
    ]

    return LaunchDescription([
        # A. Publish Robot State
        Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            output='screen', 
            parameters=[robot_description]
        ),

        # B. Fake Joint Driver
        # Listens to the fake controller output and publishes joint states
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'source_list': ['/move_group/fake_controller_joint_states']}, {'use_sim_time': True}]
        ),

        # C. Start MoveGroup
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=move_group_params,
        ),

        # D. Start RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                {'robot_description_kinematics': kinematics_yaml},
                {'ompl': ompl_planning_yaml}
            ]
        )
    ])
