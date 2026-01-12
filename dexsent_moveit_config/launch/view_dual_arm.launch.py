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
        return None

def generate_launch_description():
    # 1. Load Robot Description (URDF)
    urdf_file = os.path.join(get_package_share_directory('dexsent_description'), 'urdf', 'dual_arm.xacro')
    doc = xacro.process_file(urdf_file)
    robot_description = {'robot_description': doc.toxml()}

    # 2. Load Semantic Description (SRDF)
    srdf_file = os.path.join(get_package_share_directory('dexsent_moveit_config'), 'config', 'dual_arm.srdf')
    with open(srdf_file, 'r') as f:
        semantic_content = f.read()
    robot_description_semantic = {'robot_description_semantic': semantic_content}

    # 3. Load Kinematics
    kinematics_yaml = load_yaml('dexsent_moveit_config', 'config/kinematics.yaml')
    
    # 4. Load OMPL Planning (The missing piece!)
    ompl_planning_yaml = load_yaml('dexsent_moveit_config', 'config/ompl_planning.yaml')
    
    # 5. Common Parameters for Nodes
    move_group_params = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics := {'robot_description_kinematics': kinematics_yaml},
        {'planning_pipelines': ['ompl']},
        {'ompl': ompl_planning_yaml},
        {'use_sim_time': True},
        {'moveit_manage_controllers': False}, # Mock execution only
        {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
    ]

    return LaunchDescription([
        # A. Publish Robot State
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[robot_description]),
        
        # B. Publish Fake Joint States (since we have no real controllers)
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', name='joint_state_publisher_gui'),

        # C. THE BRAIN: Start MoveGroup Node
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=move_group_params,
        ),

        # D. The Visualizer (RViz)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                {'ompl': ompl_planning_yaml} 
            ]
        )
    ])
