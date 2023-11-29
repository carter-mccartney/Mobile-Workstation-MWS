import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    robot_dir = get_package_share_directory('mws_robot')
    pkg_path = os.path.join(robot_dir)
    xacro_file = os.path.join(pkg_path,'description','mws_robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])
    robot_description = {"robot_description": robot_description_config}

    default_config_topics = os.path.join(robot_dir, 'params', 'twist_mux_topics.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')],
            parameters=[
                default_config_topics
            ]
        )

    controller_params_file = os.path.join(robot_dir,'params','diffbot_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file,
                    robot_description]
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "joint_state_broadcaster"]
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[TimerAction(period=20.0, actions=[diff_drive_spawner])],
        )
    )


    # Code for delaying a node (I haven't tested how effective it is)
    # 
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner

    # Launch them all!
    return LaunchDescription([
        twist_mux,
        controller_manager,
        delayed_diff_drive_spawner
    ])
