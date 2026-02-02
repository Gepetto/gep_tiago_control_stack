import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Configuration ---
    # Chemin vers ton fichier URDF modifie (genere par le script python precedent)
    urdf_file_path = "/tmp/tiago_effort.urdf"

    # Verification de l'existence du fichier
    if not os.path.exists(urdf_file_path):
        print(f"\n[ERREUR CRITIQUE] Le fichier {urdf_file_path} est introuvable !")
        print("Tu dois d'abord generer ce fichier en patchant l'URDF original.")
        # On continue quand meme au cas ou le fichier serait cree juste apres,
        # mais ca va surement planter le spawner.

    # Lecture du contenu du fichier
    robot_desc = ""
    if os.path.exists(urdf_file_path):
        with open(urdf_file_path, "r") as infp:
            robot_desc = infp.read()

    # --- 1. Lancer Gazebo (Standard, Monde Vide) ---
    # On utilise gazebo_ros qui est installe par defaut avec ros-jazzy-gazebo-ros-pkgs
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("gazebo_ros"),
                        "launch",
                        "gazebo.launch.py",  # Lance server + client
                    ]
                )
            ]
        ),
        # Par defaut, ca lance un monde vide (empty.world)
    )

    # --- 2. Robot State Publisher ---
    # Publie ton URDF modifie sur /robot_description
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
    )

    # --- 3. Spawner le robot ---
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "tiago",  # Nom du robot
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
        output="screen",
    )

    # --- 4. Joint State Broadcaster (Optionnel mais recommande) ---
    # Permet d'avoir l'etat des joints dans /joint_states pour que RSP fonctionne bien
    load_jsb = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    # Astuce : On ne lance le broadcaster qu'apres avoir spawn le robot
    return LaunchDescription(
        [
            gazebo,
            rsp_node,
            spawn_entity,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[load_jsb],
                )
            ),
        ]
    )
