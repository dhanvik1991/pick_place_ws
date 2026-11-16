"""Full cell bringup: Gazebo world + robot + controllers + MoveIt + vision + pick-and-place.

    ros2 launch pick_place_sim factory.launch.py            # everything
    ros2 launch pick_place_sim factory.launch.py rviz:=true # + RViz
    ros2 launch pick_place_sim factory.launch.py auto:=false  # no pick node (manual)
    ros2 launch pick_place_sim factory.launch.py headless:=true  # Gazebo server only
    ros2 launch pick_place_sim factory.launch.py mode:=stop      # stop the belt for each pick
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, SetEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg = get_package_share_directory('pick_place_sim')
    world = os.path.join(pkg, 'worlds', 'factory.sdf')
    controllers = os.path.join(pkg, 'config', 'controllers.yaml')

    moveit_config = (
        MoveItConfigsBuilder('arm', package_name='pick_place_sim')
        .robot_description(file_path='urdf/arm.urdf.xacro',
                           mappings={'controllers_file': controllers})
        .robot_description_semantic(file_path='config/moveit/arm.srdf')
        .robot_description_kinematics(file_path='config/moveit/kinematics.yaml')
        .joint_limits(file_path='config/moveit/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    sim_time = {'use_sim_time': True}

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [
            PythonExpression(["'-s ' if '", LaunchConfiguration('headless'), "' == 'true' else ''"]),
            '-v ', LaunchConfiguration('gz_verbosity'), f' -r {world}']}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/conveyor/box_cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ] + [f'/gripper/detach_{i}@std_msgs/msg/Empty]ignition.msgs.Empty' for i in range(6)],
    )

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[moveit_config.robot_description, sim_time],
    )

    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', 'factory', '-name', 'arm', '-topic', 'robot_description'],
    )

    # one spawner loads the controllers sequentially (parallel spawners race on Humble)
    controllers_after_spawn = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot,
        on_exit=[Node(package='controller_manager', executable='spawner', output='screen',
                      arguments=['joint_state_broadcaster', 'arm_controller', 'gripper_controller',
                                 '--controller-manager-timeout', '120'])]))

    # Camera "calibration": where the camera sits in the world (matches factory.sdf)
    cam_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0.3', '--y', '0', '--z', '2.0', '--roll', '0', '--pitch', '1.5708',
                   '--yaw', '0', '--frame-id', 'world', '--child-frame-id', 'camera_link'],
        parameters=[sim_time])
    optical_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                   '--frame-id', 'camera_link', '--child-frame-id', 'camera_optical'],
        parameters=[sim_time])

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[moveit_config.to_dict(), sim_time,
                    {'publish_robot_description_semantic': True}],
    )

    detector = Node(package='pick_place_sim', executable='box_detector.py',
                    output='screen', parameters=[sim_time])
    moving = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'moving'"])
    box_spawner = Node(package='pick_place_sim', executable='box_spawner.py',
                       output='screen',
                       parameters=[sim_time, {'spacing': PythonExpression(['1.5 if ', moving, ' else 0.9'])}])
    pick_place = Node(package='pick_place_sim', executable='pick_place.py',
                      output='screen', parameters=[sim_time, {'mode': LaunchConfiguration('mode')}],
                      condition=IfCondition(LaunchConfiguration('auto')))

    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', os.path.join(pkg, 'config', 'cell.rviz')],
        parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics, sim_time],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('auto', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('mode', default_value='moving',
                              description='moving = pick from the running belt, stop = stop-and-pick'),
        DeclareLaunchArgument('gz_verbosity', default_value='1'),
        # Fortress looks for system plugins (gz_ros2_control) on this path
        AppendEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '/opt/ros/humble/lib'),
        # Gazebo transport picks the first network interface; a VPN one silently dropped messages
        SetEnvironmentVariable('IGN_IP', '127.0.0.1'),
        gz, bridge, rsp, spawn_robot, controllers_after_spawn,
        cam_tf, optical_tf, move_group, detector,
        TimerAction(period=8.0, actions=[box_spawner, pick_place]),
        rviz,
    ])
