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

from tiago_simple_mpc.tools.model_utils import load_reduced_pinocchio_model
from tiago_simple_mpc.simple_mpc_controller import MPCController


class MPCNode(Node):
    def __init__(self, pin_model, pin_data):
        super().__init__("mpc_node")
        self.get_logger().info("Loading MPC node...")

        self.model = pin_model
        self.data = pin_data

        self.nq = self.model.nq
        self.nv = self.model.nv
        self.nx = self.nq + self.nv
        self.nu = self.nv

        # MPC frequency
        self.dt = 0.01

        # MPC INIT
        self.mpc = MPCController(self.model, dt=self.dt, T=10)

        # TARGET (Safety posture)
        self.q_ref = np.zeros(self.nq)
        self.q_ref[0] = 1.5  # (shoudler)

        self.v_ref = np.zeros(self.nv)
        self.x_ref = np.concatenate([self.q_ref, self.v_ref])

        # Storage of state
        self.current_sensor_py = None
        self.x_measured = None

        # Publisher to the LFC
        self.pub_control = self.create_publisher(
            Control,
            "/control",
            qos_profile=QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        # Subscriber from the LFC
        self.sub_sensor = self.create_subscription(
            Sensor,
            "sensor",
            self.sensor_callback,
            qos_profile=QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        # Control timer (100 Hz)
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(f"Node ready. Controlling {self.nq} joints.")

    def sensor_callback(self, msg):
        """Conversion message ROS -> Numpy"""
        try:
            self.current_sensor_py = sensor_msg_to_numpy(msg)

            q = self.current_sensor_py.joint_state.position
            v = self.current_sensor_py.joint_state.velocity

            if len(q) != self.nq:
                self.get_logger().warn(
                    f"Dimensions incohérentes : Reçu {len(q)}, Attendu {self.nq}"
                )
                return

            self.x_measured = np.concatenate([q, v])

        except Exception as e:
            self.get_logger().error(f"Error sensor: {e}")

    def control_loop(self):
        """Main loop"""
        if self.x_measured is None:
            return

        # SOLVE MPC
        u_ff = self.mpc.solve(self.x_measured, self.x_ref)

        # PREPARATION OF CONTROL MESSAGE
        # Pure torque: K=0, feedforward=u_ff
        K = np.zeros((self.nu, self.nx))

        control_py = lfc_py_types.Control(
            feedback_gain=K, feedforward=u_ff, initial_state=self.current_sensor_py
        )

        # SEND
        try:
            msg = control_numpy_to_msg(control_py)
            self.pub_control.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Erreur publication: {e}")


def main(args=None):
    rclpy.init(args=args)

    print("[MPC node] Loading Pinocchio model...")

    arm_joints = [
        "arm_1_joint",
        "arm_2_joint",
        "arm_3_joint",
        "arm_4_joint",
        "arm_5_joint",
        "arm_6_joint",
        "arm_7_joint",
    ]

    try:
        # Load reduced model
        model, data = load_reduced_pinocchio_model(arm_joints)
        print(f"[MPC node] model loaded with success : {model.nq} DDL")

        # Launch MPC node
        node = MPCNode(model, data)
        rclpy.spin(node)

    except Exception as e:
        print(f"[MPC node] Critical error in main: {e}")
    finally:
        # Global cleanup
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
