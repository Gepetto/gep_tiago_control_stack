# Launch specific config for tiago on gazebo.
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base_type = DeclareLaunchArgument("base_type", default_value="pmb2")
    has_screen = DeclareLaunchArgument("has_screen", default_value="False")
    arm_type = DeclareLaunchArgument("arm_type", default_value="tiago-arm")
    end_effector = DeclareLaunchArgument("end_effector", default_value="pal-gripper")
    ft_sensor = DeclareLaunchArgument("ft_sensor", default_value="no-ft-sensor")
    wrist_model = DeclareLaunchArgument("wrist_model", default_value="wrist-2017")
    camera_model = DeclareLaunchArgument("camera_model", default_value="no-camera")
    laser_model = DeclareLaunchArgument("laser_model", default_value="no-laser")
    navigation = DeclareLaunchArgument("navigation", default_value="False")
    advanced_nav = DeclareLaunchArgument("advanced_navigation", default_value="False")
    slam = DeclareLaunchArgument("slam", default_value="False")
    moveit = DeclareLaunchArgument("moveit", default_value="True")
    world_name = DeclareLaunchArgument("world_name", default_value="empty")
    namespace = DeclareLaunchArgument("namespace", default_value="")
    tuck_arm = DeclareLaunchArgument("tuck_arm", default_value="True")
    is_public_sim = DeclareLaunchArgument("is_public_sim", default_value="True")

    include_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tiago_gazebo"), "launch", "tiago_gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "base_type": LaunchConfiguration("base_type"),
            "has_screen": LaunchConfiguration("has_screen"),
            "arm_type": LaunchConfiguration("arm_type"),
            "end_effector": LaunchConfiguration("end_effector"),
            "ft_sensor": LaunchConfiguration("ft_sensor"),
            "wrist_model": LaunchConfiguration("wrist_model"),
            "camera_model": LaunchConfiguration("camera_model"),
            "laser_model": LaunchConfiguration("laser_model"),
            "navigation": LaunchConfiguration("navigation"),
            "advanced_navigation": LaunchConfiguration("advanced_navigation"),
            "slam": LaunchConfiguration("slam"),
            "moveit": LaunchConfiguration("moveit"),
            "world_name": LaunchConfiguration("world_name"),
            "namespace": LaunchConfiguration("namespace"),
            "tuck_arm": LaunchConfiguration("tuck_arm"),
            "is_public_sim": LaunchConfiguration("is_public_sim"),
        }.items(),
    )

    return LaunchDescription(
        [
            base_type,
            has_screen,
            arm_type,
            end_effector,
            ft_sensor,
            wrist_model,
            camera_model,
            laser_model,
            navigation,
            advanced_nav,
            slam,
            moveit,
            world_name,
            namespace,
            tuck_arm,
            is_public_sim,
            include_gazebo,
        ]
    )
