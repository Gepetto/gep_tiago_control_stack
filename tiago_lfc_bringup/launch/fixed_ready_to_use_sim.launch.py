from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    LogInfo,
)
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

    # action to activate LFC controllers (will be triggered AFTER tuck_arm)
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
        launch_arguments={
            "pkg": "tiago_lfc_bringup",
            "lfc_yaml": "config/fixed/linear_feedback_controller_params.yaml",
            "jse_yaml": "config/fixed/joint_state_estimator_params.yaml",
            "pc_yaml": "config/fixed/dummy_controllers.yaml",
        }.items(),
    )

    # Track whether tuck_arm has been done
    tuck_arm_done = False

    def on_tuck_arm_exit_callback(event, context):
        """
        Wait for tuck_arm.py to complete before activating LFC controllers.
        This ensures the arm is in a safe position before switching controllers.
        """
        nonlocal tuck_arm_done
        name = event.process_name or ""
        rc_ok = event.returncode == 0

        if "tuck_arm.py" in name:
            if rc_ok:
                if not tuck_arm_done:
                    tuck_arm_done = True
                    return [
                        LogInfo(
                            msg="tuck_arm.py completed successfully. Now activating LFC controllers..."
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
                return [
                    LogInfo(
                        msg=f"tuck_arm.py failed with code {event.returncode}. NOT starting controllers."
                    )
                ]
        return []

    on_tuck_arm_exit_handler = RegisterEventHandler(
        OnProcessExit(on_exit=on_tuck_arm_exit_callback)
    )

    return LaunchDescription(
        [
            on_tuck_arm_exit_handler,  # Register the handler before launching
            launch_gazebo_tiago,  # Start Gazebo + Tiago (includes tuck_arm)
        ]
    )
