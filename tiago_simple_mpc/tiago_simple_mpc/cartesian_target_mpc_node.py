#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.qos_overriding_options import QoSOverridingOptions

from linear_feedback_controller_msgs.msg import Control, Sensor
from linear_feedback_controller_msgs_py.numpy_conversions import (
    sensor_msg_to_numpy,
    control_numpy_to_msg,
)
import linear_feedback_controller_msgs_py.lfc_py_types as lfc_py_types

# --- NOUVEAUX IMPORTS ---
from tiago_simple_mpc.tools.model_utils import load_full_pinocchio_model


class MPCNode(Node):
    def __init__(self, pin_model, pin_data):
        super().__init__("mpc_node")
        self.get_logger().info(
            "Loading MPC node (Full Body / End-Effector Tracking)..."
        )

        self.model = pin_model
        self.data = pin_data

        self.nq = self.model.nq
        self.nv = self.model.nv
        self.nx = self.nq + self.nv
        self.nu = self.nv

        # MPC settings
        self.dt = 0.01

        # --- CONFIGURATION DU FRAME CIBLE ---
        # "gripper_tool_link" est le repère entre les doigts de la pince Tiago
        target_frame = "gripper_tool_link"

        # Vérification de l'existence du frame
        if not self.model.existFrame(target_frame):
            self.get_logger().error(
                f"Frame '{target_frame}' introuvable dans le modèle !"
            )
            raise ValueError(f"Frame incorrect: {target_frame}")

        # Initialisation du MPC avec le nom du frame
        # self.mpc = MPCController(
        #    self.model, target_frame_name=target_frame, dt=self.dt, T=10
        # )

        # --- DEFINITION DE LA CIBLE (x, y, z) ---
        # Repère world/base_footprint.
        # Tiago : x=0.6m devant, z=0.8m haut (hauteur poitrine environ)
        self.target_pos = np.array([0.6, 0.0, 0.8])

        self.get_logger().info(f"Cible initialisée à : {self.target_pos}")

        # Storage
        self.current_sensor_py = None
        self.x_measured = None

        # --- COMMUNICATIONS ROS (Inchangé) ---
        self.pub_control = self.create_publisher(
            Control,
            "/control",
            qos_profile=QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self.sub_sensor = self.create_subscription(
            Sensor,
            "sensor",
            self.sensor_callback,
            qos_profile=QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(f"Node prêt. Contrôle de {self.nq} articulations.")

    def sensor_callback(self, msg):
        try:
            self.current_sensor_py = sensor_msg_to_numpy(msg)

            # Récupération état complet
            q = self.current_sensor_py.joint_state.position
            v = self.current_sensor_py.joint_state.velocity

            # Vérification dimensionnelle robuste
            if len(q) != self.nq:
                # Parfois le LFC envoie plus/moins de joints si mal configuré
                # On pourrait filtrer ici, mais pour l'instant on warning
                self.get_logger().warn(
                    f"Mismatch dimensions! Modèle: {self.nq}, Reçu: {len(q)}",
                    throttle_duration_sec=1.0,
                )
                return

            self.x_measured = np.concatenate([q, v])

        except Exception as e:
            self.get_logger().error(f"Error sensor: {e}")

    def control_loop(self):
        """Boucle de contrôle 100Hz"""
        if self.x_measured is None:
            return

        # --- APPEL MPC CARTESIEN ---
        # On fournit l'état mesuré (q,v) et la position désirée (x,y,z)
        try:
            u_ff = self.mpc.solve(self.x_measured, self.target_pos)
        except Exception as e:
            self.get_logger().error(f"Erreur Solveur: {e}")
            return

        # --- ENVOI COMMANDE ---
        # Matrice K nulle (Full MPC Feedforward)
        K = np.zeros((self.nu, self.nx))

        control_py = lfc_py_types.Control(
            feedback_gain=K, feedforward=u_ff, initial_state=self.current_sensor_py
        )

        try:
            msg = control_numpy_to_msg(control_py)
            self.pub_control.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Erreur publication: {e}")


def main(args=None):
    rclpy.init(args=args)

    print("[MPC node] Loading Pinocchio model...")

    try:
        # 1. Charger tout le modèle (pas de liste de joints restrictive)
        model, data = load_full_pinocchio_model()

        if model is None:
            raise Exception("Impossible de charger le modèle (XML/URDF introuvable ?)")

        print(
            f"[MPC node] Modèle chargé: {model.nq} variables de config, {model.nv} variables de vitesse"
        )
        print(
            f"[MPC node] Liste des Frames : {[f.name for f in model.frames if 'gripper' in f.name]}"
        )  # Debug frames

        # 2. Lancer le Node
        # node = MPCNode(model, data)
        # rclpy.spin(node)

    except Exception as e:
        print(f"[MPC node] Critical error in main: {e}")
        import traceback

        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
