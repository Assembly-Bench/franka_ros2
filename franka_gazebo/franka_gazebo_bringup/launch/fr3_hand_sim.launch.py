"""Launch a simulated FR3 and Franka Hand for GELLO teleoperation."""

from pathlib import Path
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def hand_robot_description(bringup_dir):
    """Expose independent effort control for both simulated fingers."""
    model = xacro.process_file(
        str(bringup_dir / 'urdf/franka_arm.gazebo.xacro'),
        mappings={
            'robot_type': 'fr3',
            'hand': 'true',
            'ee_id': 'franka_hand',
            'gazebo': 'true',
            'gazebo_effort': 'true',
        },
    )
    robot = ET.fromstring(model.toxml())
    hardware = robot.find('ros2_control')
    # Fortress drops the URDF mimic constraint during SDF conversion. Command both
    # Gazebo sliders and keep the URDF mimic intact for robot_state_publisher.
    second_finger = ET.SubElement(hardware, 'joint', name='fr3_finger_joint2')
    ET.SubElement(second_finger, 'command_interface', name='effort')
    ET.SubElement(second_finger, 'state_interface', name='position')
    ET.SubElement(second_finger, 'state_interface', name='velocity')
    return ET.tostring(robot, encoding='unicode')


def generate_launch_description():
    """Start the simulation, controllers, GELLO hand bridge and optional RViz."""
    bringup_dir = Path(get_package_share_directory('franka_gazebo_bringup'))
    sim_dir = Path(get_package_share_directory('ros_gz_sim'))
    description_dir = Path(get_package_share_directory('franka_description'))
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_dir / 'launch/gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-s -r /usr/share/ignition/ignition-gazebo6/worlds/empty.sdf',
        }.items(),
    )
    arm_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'gravity_compensation_example_controller',
            '--controller-manager-timeout',
            '60',
        ],
        output='screen',
    )
    finger_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gello_finger_controller',
            '-t',
            'joint_trajectory_controller/JointTrajectoryController',
            '-p',
            str(bringup_dir / 'config/finger_controller.yaml'),
            '--controller-manager-timeout',
            '60',
        ],
        output='screen',
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
            SetParameter(name='use_sim_time', value=True),
            gazebo,
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
                output='screen',
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[
                    {
                        'robot_description': ParameterValue(
                            hand_robot_description(bringup_dir), value_type=str
                        ),
                    }
                ],
                output='screen',
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-world', 'empty', '-topic', '/robot_description', '-name', 'fr3'],
                output='screen',
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=arm_controllers, on_exit=[finger_controller])
            ),
            arm_controllers,
            Node(
                package='franka_gazebo_bringup', executable='gello_hand_bridge.py', output='screen'
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                condition=IfCondition(LaunchConfiguration('rviz')),
                arguments=[
                    '-d',
                    str(description_dir / 'rviz/visualize_franka.rviz'),
                    '-f',
                    'world',
                ],
                output='screen',
            ),
        ]
    )
