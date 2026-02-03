#!/usr/bin/env python3
"""
Cartesian MPC Controller Node for Tiago Robot
Supports both fixed-base and floating-base configurations.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import pinocchio as pin
import time

from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.qos_overriding_options import QoSOverridingOptions

from linear_feedback_controller_msgs.msg import Control, Sensor
from linear_feedback_controller_msgs_py.numpy_conversions import (
    sensor_msg_to_numpy,
    control_numpy_to_msg,
)
import linear_feedback_controller_msgs_py.lfc_py_types as lfc_py_types

# MPC imports
from tiago_simple_mpc.core.model_utils import load_reduced_pinocchio_model
from tiago_simple_mpc.mpc.build_cartesian_target_ocp import build_cartesian_target_ocp
from tiago_simple_mpc.mpc.mpc_builder import MPCController


class MPCNode(Node):
    def __init__(self, pin_model, pin_data, target_frame, has_free_flyer=False):
        """
        MPC Controller Node for Cartesian end-effector tracking.

        Args:
            pin_model: Pinocchio model (with or without FreeFlyer)
            pin_data: Pinocchio data
            target_frame: Name of end-effector frame to track
            has_free_flyer: True if model has floating base, False for fixed base
        """
        super().__init__("mpc_node")
        self.get_logger().info("Initializing MPC node (Cartesian SE(3) Tracking)...")

        # Store model
        self.model = pin_model
        self.data = pin_data
        self.has_free_flyer = has_free_flyer
        self.target_frame = target_frame

        # Model dimensions
        self.nq = self.model.nq
        self.nv = self.model.nv
        self.nx = self.nq + self.nv

        self.get_logger().info(
            f"Model dimensions:\n"
            f"  - nq (positions): {self.nq}\n"
            f"  - nv (velocities): {self.nv}\n"
            f"  - nx (state): {self.nx}\n"
            f"  - has_free_flyer: {self.has_free_flyer}"
        )

        # MPC settings
        self.dt = 0.01  # 100 Hz
        self.mpc_horizon = 20  # 200ms prediction horizon

        # VERIFY TARGET FRAME
        if not self.model.existFrame(target_frame):
            available_frames = [
                f.name for f in self.model.frames if "arm" in f.name.lower()
            ]
            self.get_logger().error(
                f"Frame '{target_frame}' not found!\n"
                f"Available arm frames: {available_frames}"
            )
            raise ValueError(f"Invalid target frame: {target_frame}")

        # Define target as SE(3) pose (position + orientation)
        self.target_pose = pin.SE3(np.eye(3), np.array([2.0, 2.0, 1.0]))

        self.get_logger().info(
            f"Target SE(3) pose:\n"
            f"  Position: {self.target_pose.translation}\n"
            f"  Rotation: identity"
        )

        # Measured state
        self.current_sensor_py = None
        self.x_measured = None
        self.first_measurement_received = False
        self.mpc_controller = None  # Will be initialized after first measurement

        # Statistics
        self.control_iterations = 0
        self.total_solve_time = 0.0
        self.max_solve_time = 0.0

        # ROS 2 COMMUNICATIONS
        qos_rt = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_opts = QoSOverridingOptions.with_default_policies()

        self.pub_control = self.create_publisher(
            Control, "/control", qos_profile=qos_rt, qos_overriding_options=qos_opts
        )

        self.sub_sensor = self.create_subscription(
            Sensor,
            "sensor",
            self.sensor_callback,
            qos_profile=qos_rt,
            qos_overriding_options=qos_opts,
        )

        # Control timer at 100Hz
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("MPC Node ready! Waiting for sensor data...")

    def _convert_freeflyer_to_planar(self, q_ff, v_ff):
        """
        Convert FreeFlyer (3D) to Planar (2D).

        Args:
            q_ff: Configuration FreeFlyer base (7,)
                Structure:
                [0:3]   → x, y, z (position base)
                [3:7]   → qx, qy, qz, qw (quaternion base)

            v_ff: Velocity FreeFlyer base (6,)
                Structure:
                [0:3]  → vx, vy, vz (linear velocity base)
                [3:6]  → ωx, ωy, ωz (angular velocity base)

        Returns:
            q_planar: Configuration Planar base (4,)
                    [x, y, cos(θ), sin(θ)]

            v_planar: Vitesse Planar base (3,)
                    [vx, vy, ω]
        """

        # POSITION BASE 2D (7 → 4)
        x = q_ff[0]
        y = q_ff[1]
        # z = q_ff[2]  # not used in planar

        # Quaternion
        qx = q_ff[3]
        qy = q_ff[4]
        qz = q_ff[5]
        qw = q_ff[6]

        # Yaw calculation (rotation around Z)
        theta = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))

        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        # Construction de q_planar (plus efficace sans concatenate pour 4 valeurs)
        q_planar = np.array([x, y, cos_theta, sin_theta])

        # --- VITESSE BASE 2D (6 → 3) ---
        vx = v_ff[0]
        vy = v_ff[1]
        # vz = v_ff[2]  # not used in planar

        # ωx = v_ff[3]  # not used in planar
        # ωy = v_ff[4]  # not used in planar
        omega = v_ff[5]  # Vitesse de rotation autour de Z (lacet)

        # Construction de v_planar (plus efficace)
        v_planar = np.array([vx, vy, omega])

        # --- VALIDATION ---
        assert q_planar.shape == (4,), (
            f"❌ q_planar shape error: {q_planar.shape}, expected (4,)"
        )
        assert v_planar.shape == (3,), (
            f"❌ v_planar shape error: {v_planar.shape}, expected (3,)"
        )

        return q_planar, v_planar

    def sensor_callback(self, msg):
        """
        Sensor callback with dynamic reconstruction based on has_free_flyer.
        Uses pre-computed joint structure mapping.
        """
        try:
            self.current_sensor_py = sensor_msg_to_numpy(msg)

            # Extract measured joint state
            q_measured = self.current_sensor_py.joint_state.position[
                2:
            ]  # Without wheels
            v_measured = self.current_sensor_py.joint_state.velocity[
                2:
            ]  # Without wheels

            # Validation: Check joints dimension
            n_joints = self.nv - 3 if self.has_free_flyer else self.nv
            if len(q_measured) != n_joints:
                self.get_logger().error(
                    f"Sensor dimension mismatch!\n"
                    f"  Expected arm joints: {n_joints}\n"
                    f"  Received: q={len(q_measured)}, v={len(v_measured)}",
                    throttle_duration_sec=2.0,
                )
                return

            # Reconstruction: FreeFlyer vs Fixed Base
            if self.has_free_flyer:
                # with planar base
                base_pose_ff = (
                    self.current_sensor_py.base_pose
                )  # [x, y, z, qx, qy, qz, qw]
                base_twist_ff = (
                    self.current_sensor_py.base_twist
                )  # [vx, vy, vz, wx, wy, wz]

                # Validation
                if len(base_pose_ff) != 7:
                    self.get_logger().error(
                        f"base_pose invalid: {len(base_pose_ff)} (expected 7)"
                    )
                    return
                if len(base_twist_ff) != 6:
                    self.get_logger().error(
                        f"base_twist invalid: {len(base_twist_ff)} (expected 6)"
                    )
                    return

                # CONVERSION FREEFLYER → PLANAR
                base_pose_planar, base_twist_planar = self._convert_freeflyer_to_planar(
                    base_pose_ff, base_twist_ff
                )

                # Concatenate: [base_pose, joint_configs]
                q_full = np.concatenate([base_pose_planar, q_measured])
                v_full = np.concatenate([base_twist_planar, v_measured])

            else:
                # Fixed base, (no freeflyer)
                q_full = q_measured
                v_full = v_measured

            # Final validation
            if len(q_full) != self.nq:
                self.get_logger().error(
                    f"Configuration dimension mismatch!\n"
                    f"  Expected nq: {self.nq}\n"
                    f"  Reconstructed: {len(q_full)}\n"
                    f"  Breakdown:\n"
                    f"    - Base: {4 if self.has_free_flyer else 0}\n"
                    f"    - Arm joints: {len(q_measured)}\n"
                    f"    - Total: {len(q_full)}"
                )
                return

            if len(v_full) != self.nv:
                self.get_logger().error(
                    f"Velocity dimension mismatch!\n"
                    f"  Expected nv: {self.nv}\n"
                    f"  Reconstructed: {len(v_full)}\n"
                    f"  Breakdown:\n"
                    f"    - Base: {3 if self.has_free_flyer else 0}\n"
                    f"    - Arm velocities: {len(v_measured)}\n"
                    f"    - Total: {len(v_full)}"
                )
                return

            # Construct full state
            self.x_measured = np.concatenate([q_full, v_full])

            # Initialize MPC on first measurement
            if not self.first_measurement_received:
                self._initialize_mpc()
                self.first_measurement_received = True
                self.get_logger().info("First measurement received, MPC initialized!")

        except Exception as e:
            self.get_logger().error(
                f"Sensor callback error: {e}", throttle_duration_sec=1.0
            )

    def _initialize_mpc(self):
        """
        Initialize MPC controller with first measured state.
        """
        if self.x_measured is None:
            raise RuntimeError("Cannot initialize MPC without measured state!")

        self.get_logger().info("Building OCP...")

        # Build OCP using your modular builder
        ocp = build_cartesian_target_ocp(
            x0=self.x_measured,
            target_pose=self.target_pose,
            frame_name=self.target_frame,
            model=self.model,
            has_free_flyer=self.has_free_flyer,
            dt=self.dt,
            horizon_length=self.mpc_horizon,
        )

        # Create MPC controller
        self.mpc_controller = MPCController(
            ocp=ocp,
            x0=self.x_measured,
            u0=None,  # Will use zeros
            max_iter=20,
            verbose=True,
        )

        # Store control dimension
        self.nu = self.mpc_controller.nu

        self.get_logger().info(
            f"MPC initialized:\n"
            f"  - State dim (nx): {self.nx}\n"
            f"  - Control dim (nu): {self.nu}\n"
            f"  - Horizon: {self.mpc_horizon} steps\n"
            f"  - dt: {self.dt}s"
        )

    def control_loop(self):
        """MPC control loop at 100Hz."""
        # Wait for first measurement
        if not self.first_measurement_received or self.x_measured is None:
            return

        try:
            # Solve MPC
            t_start = time.time()
            dt, u_optimal = self.mpc_controller.step(self.x_measured)
            solve_time = time.time() - t_start

            # Build control message
            # Zero feedback matrix (pure feedforward MPC)
            K = np.zeros((self.nu, self.nx))

            control_py = lfc_py_types.Control(
                feedback_gain=K,
                feedforward=u_optimal,
                initial_state=self.current_sensor_py,
            )

            # Publish
            msg = control_numpy_to_msg(control_py)
            self.pub_control.publish(msg)

            # Update statistics
            self.control_iterations += 1
            self.total_solve_time += solve_time
            self.max_solve_time = max(self.max_solve_time, solve_time)

            # Log every second
            if self.control_iterations % 100 == 0:
                avg_time = self.total_solve_time / self.control_iterations
                self.get_logger().info(
                    f"📊 MPC Stats (iter {self.control_iterations}):\n"
                    f"  - Avg solve: {avg_time * 1000:.2f}ms\n"
                    f"  - Max solve: {self.max_solve_time * 1000:.2f}ms\n"
                    f"  - Last solve: {solve_time * 1000:.2f}ms"
                )

        except Exception as e:
            self.get_logger().error(f"MPC step failed: {e}")

    def _log_diagnostics(self, last_solve_time):
        """Log detailed MPC diagnostics."""
        try:
            traj = self.mpc.get_predicted_trajectory()
            current_ee_pos = self.get_current_ee_position()
            error = np.linalg.norm(current_ee_pos - self.target_pos)
            avg_solve_time = self.total_solve_time / self.control_iterations

            self.get_logger().info(
                f"\n📊 MPC Diagnostics (t={self.control_iterations * self.dt:.1f}s):\n"
                f"  ╔══════════════════════════════════════╗\n"
                f"  ║ Optimization                         ║\n"
                f"  ║  - Cost: {traj['cost']:.6f}               ║\n"
                f"  ║  - Iterations: {traj['iterations']:<2d}                   ║\n"
                f"  ║  - Solve time: {last_solve_time * 1000:.2f} ms (avg: {avg_solve_time * 1000:.2f})║\n"
                f"  ║  - Max time: {self.max_solve_time * 1000:.2f} ms             ║\n"
                f"  ╠══════════════════════════════════════╣\n"
                f"  ║ Tracking                             ║\n"
                f"  ║  - Position error: {error:.4f} m          ║\n"
                f"  ║  - Current EE: [{current_ee_pos[0]:.3f}, {current_ee_pos[1]:.3f}, {current_ee_pos[2]:.3f}]  ║\n"
                f"  ║  - Target:     [{self.target_pos[0]:.3f}, {self.target_pos[1]:.3f}, {self.target_pos[2]:.3f}]  ║\n"
                f"  ╚══════════════════════════════════════╝"
            )

            # Warning if solve time exceeds control period
            if last_solve_time > self.dt:
                self.get_logger().warn(
                    f"Solve time ({last_solve_time * 1000:.2f}ms) exceeds control period ({self.dt * 1000:.0f}ms)!"
                )

        except Exception as e:
            self.get_logger().error(f"Diagnostics error: {e}")

    def get_current_ee_position(self):
        """Computes current Cartesian position of the end-effector."""
        try:
            q = self.x_measured[: self.nq]
            return self.mpc.get_frame_position(q)
        except Exception as e:
            self.get_logger().error(f"Error computing EE position: {e}")
            return np.zeros(3)


def main(args=None):
    rclpy.init(args=args)

    print("\n" + "=" * 60)
    print("  🤖 Tiago MPC Cartesian Controller")
    print("=" * 60 + "\n")

    try:
        USE_FREE_FLYER = True
        TARGET_FRAME = "gripper_grasping_frame"

        print("Loading reduced model...")
        target_joints = [
            # 'wheel_left_joint', 'wheel_right_joint',
            # 'torso_lift_joint',
            "arm_1_joint",
            "arm_2_joint",
            "arm_3_joint",
            "arm_4_joint",
            "arm_5_joint",
            "arm_6_joint",
            "arm_7_joint",
        ]
        model, data, visual_model, visual_data = load_reduced_pinocchio_model(
            target_joints, has_free_flyer=USE_FREE_FLYER
        )

        if model is None:
            raise RuntimeError("Failed to load Pinocchio model! Check URDF path.")

        print(f"Model loaded: nq={model.nq}, nv={model.nv}")

        # Create MPC node
        print("\nInitializing MPC node...")
        node = MPCNode(
            model, data, target_frame=TARGET_FRAME, has_free_flyer=USE_FREE_FLYER
        )

        # Spin
        print("\n Starting control loop...\n")
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n Interrupted by user")
    except Exception as e:
        print(f"\n Critical error: {e}")
        import traceback

        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        print("\n Shutdown complete\n")


if __name__ == "__main__":
    main()
