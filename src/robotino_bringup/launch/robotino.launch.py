from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
#from launch.substitutions import Command


def generate_launch_description():

    ign_gazebo_launch = get_package_share_path('ros_ign_gazebo') \
        / 'launch' / 'ign_gazebo.launch.py'
    robotino_urdf = get_package_share_path('robotino_description') \
        / 'urdf' / 'robotino.urdf'

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ign_gazebo_launch.as_posix()),
        launch_arguments={
            'ign_args': '-r empty.sdf'
        }.items(),
    )

#    path_to_urdf = get_package_share_path('pr2_description') / 'robots' / 'pr2.urdf.xacro'
#    robot_state_publisher_node = launch_ros.actions.Node(
#        package='robot_state_publisher',
#        executable='robot_state_publisher',
#        parameters=[{
#            'robot_description': ParameterValue(
#                Command(['xacro ', str(path_to_urdf)]), value_type=str
#            )
#        }]
#    )

    spawn = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
           '-name', 'robotino',
           '-file', robotino_urdf.as_posix()],
        output='screen',
    )

    planning_service = Node(
        package='robotino_planning',
        executable='planning_service',
        output='screen',
    )

    planning_client = Node(
        package='robotino_planning',
        executable='planning_client',
        output='screen',
    )

    control_action_server = Node(
        package='robotino_control',
        executable='control_action_server',
        output='screen',
    )

    control_action_client = Node(
        package='robotino_control',
        executable='control_action_client',
        output='screen',
    )

    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/model/robotino/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                   '/model/robotino/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
        output='screen'
    )

    return LaunchDescription([
        ign_gazebo,
        spawn,
        planning_service,
        planning_client,
        control_action_server,
        control_action_client,
        bridge,
    ])

