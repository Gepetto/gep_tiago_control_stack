from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Arguments
    pkg_arg       = DeclareLaunchArgument('pkg', default_value='tiago_lfc_bringup')
    lfc_yaml_arg  = DeclareLaunchArgument('lfc_yaml', default_value='config/linear_feedback_controller_params.yaml')
    jse_yaml_arg  = DeclareLaunchArgument('jse_yaml', default_value='config/joint_state_estimator_params.yaml')
    pc_yaml_arg   = DeclareLaunchArgument('pc_yaml',  default_value='config/dummy_controllers.yaml')

    pkg  = LaunchConfiguration('pkg')
    lfcP = PathJoinSubstitution([FindPackageShare(pkg), LaunchConfiguration('lfc_yaml')])
    jseP = PathJoinSubstitution([FindPackageShare(pkg), LaunchConfiguration('jse_yaml')])
    pcP  = PathJoinSubstitution([FindPackageShare(pkg), LaunchConfiguration('pc_yaml')])

    # Spawner for all 3 controllers
    spawn_linear_feedback_controller = Node(
        package='controller_manager', executable='spawner',
        name='spawner_linear_feedback_controller',
        arguments=[
            'linear_feedback_controller',
            '--inactive',
            '--param-file', lfcP,
        ],
        output='screen'
    )
    spawn_joint_state_estimator = Node(
        package='controller_manager', executable='spawner',
        name='spawner_joint_state_estimator',
        arguments=[
            'joint_state_estimator',
            '--inactive',
            '--param-file', jseP,
        ],
        output='screen'
    )
    spawn_passthrough_controller = Node(
        package='controller_manager', executable='spawner',
        name='spawner_passthrough_controller',
        arguments=[
            'passthrough_controller',
            '--inactive',
            '--param-file', pcP,
        ],
        output='screen'
    )

    # Deactivate the old arm controller if present
    # TODO : can it works without being deactivated, as it does not use same hardware interface ?
    deactivate_arm = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'switch_controllers',
            '--deactivate', 'arm_controller'
        ],
        output='screen'
    )

    # Activate the chain all at once (Passthrough + LFC + JSE)
    activate_chain = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'switch_controllers',
            '--activate',
            'passthrough_controller', 'linear_feedback_controller', 'joint_state_estimator',
            '--strict'
        ],
        output='screen'
    )

    # Orchestration
    chain_spawn_joint_state_estimator = RegisterEventHandler(
        OnProcessExit(target_action=spawn_linear_feedback_controller, on_exit=[spawn_joint_state_estimator])
    )
    chain_spawn_passthrough_controller = RegisterEventHandler(
        OnProcessExit(target_action=spawn_joint_state_estimator, on_exit=[spawn_passthrough_controller])
    )
    chain_switch_1 = RegisterEventHandler(
        OnProcessExit(target_action=spawn_passthrough_controller, on_exit=[deactivate_arm])
    )
    chain_switch_2 = RegisterEventHandler(
        OnProcessExit(target_action=deactivate_arm, on_exit=[activate_chain])
    )

    return LaunchDescription([
        pkg_arg, lfc_yaml_arg, jse_yaml_arg, pc_yaml_arg,
        spawn_linear_feedback_controller,
        chain_spawn_joint_state_estimator,
        chain_spawn_passthrough_controller,
        chain_switch_1,
        chain_switch_2,
    ])
