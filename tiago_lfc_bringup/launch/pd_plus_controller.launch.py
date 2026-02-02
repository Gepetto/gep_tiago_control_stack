from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    LogInfo,
)
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # launch robot simulation in gazebo
    launch_gazebo_tiago = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("tiago_lfc_bringup"),
                    "launch",
                    "tiago_gazebo.launch.py",
                ]
            )
        ),
    )

    # action to activate LFC controllers
    activate_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("tiago_lfc_bringup"),
                    "launch",
                    "switch_to_lfc_controllers.launch.py",
                ]
            )
        ),
    )

    # pd_plus_controller node
    pd_plus_controller_params = PathJoinSubstitution(
        [
            FindPackageShare("tiago_lfc_bringup"),
            "config",
            "pd_plus_controller_params.yaml",
        ]
    )
    pd_plus_controller_node = Node(
        package="linear_feedback_controller",
        executable="pd_plus_controller",
        parameters=[pd_plus_controller_params],
        output="screen",
    )

    # Track whether tuck_arm has been done
    tuck_arm_done = False
    controllers_done = False
    spawners_seen = 0
    expected_spawners = 10

    def on_tuck_arm_exit_callback(event, context):
        """Callback when tuck_arm.py exits."""
        nonlocal tuck_arm_done
        name = event.process_name or ""
        rc_ok = event.returncode == 0
        if "tuck_arm.py" in name:
            if rc_ok:
                if not tuck_arm_done:
                    tuck_arm_done = True
                    return [
                        LogInfo(
                            msg="tuck_arm.py completed. Activating LFC controllers."
                        ),
                        activate_controllers,
                    ]
                else:
                    return [
                        LogInfo(
                            msg="Controllers already activated. Ignoring duplicate tuck_arm exit."
                        )
                    ]
            else:
                return [LogInfo(msg="tuck_arm.py failed. Not starting controllers.")]
        return []

    on_tuck_arm_exit_handler = RegisterEventHandler(
        OnProcessExit(on_exit=on_tuck_arm_exit_callback)
    )

    def on_spawners_exit_callback(event, context):
        """Callback when any controller spawner process exits."""
        nonlocal controllers_done, spawners_seen

        name = event.process_name or ""
        rc_ok = event.returncode == 0

        if not name.startswith("spawner-"):
            return []

        if not rc_ok:
            return [
                LogInfo(
                    msg=f"{name} exited with code {event.returncode}. Not starting pd_plus_controller."
                )
            ]

        spawners_seen += 1

        if spawners_seen >= expected_spawners and not controllers_done:
            controllers_done = True
            return [
                LogInfo(
                    msg=f"All spawners done ({spawners_seen}/{expected_spawners}). Starting pd_plus_controller."
                ),
                pd_plus_controller_node,
            ]

        return [
            LogInfo(
                msg=f"{name} done ({spawners_seen}/{expected_spawners}). Waiting for others..."
            )
        ]

    on_spawners_exit_handler = RegisterEventHandler(
        OnProcessExit(on_exit=on_spawners_exit_callback)
    )

    return LaunchDescription(
        [
            on_tuck_arm_exit_handler,
            on_spawners_exit_handler,
            launch_gazebo_tiago,
        ]
    )
