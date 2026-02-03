#!/usr/bin/env python3
"""
Simple OCP solver node for Tiago reaching task.
Loads URDF from ROS topic, builds OCP, solves once, and displays results on meshcat
"""

import os
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
import numpy as np
import pinocchio as pin
import crocoddyl

from tiago_simple_mpc.core.model_utils import load_reduced_pinocchio_model
from tiago_simple_mpc.ocp.ocp_builder import OCPBuilder
from tiago_simple_mpc.ocp.cost_manager import CostModelManager

from linear_feedback_controller_msgs.msg import Sensor
from linear_feedback_controller_msgs_py.numpy_conversions import sensor_msg_to_numpy

from tiago_simple_mpc.ocp.visualizer import TrajectoryVisualizer

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
from pinocchio.visualize import MeshcatVisualizer


class OCPReachingNode(Node):
    """Simple node to test OCP resolution on Tiago."""

    def __init__(self):
        super().__init__("ocp_reaching_test")

        self.dt = 0.01
        self.horizon_steps = 100  # T
        self.has_free_flyer = True

        target_joints = [
            # 'torso_lift_joint',
            "arm_1_joint",
            "arm_2_joint",
            "arm_3_joint",
            "arm_4_joint",
            "arm_5_joint",
            "arm_6_joint",
            "arm_7_joint",
        ]

        self.model, self.data, self.visual_model, self.visual_data = (
            load_reduced_pinocchio_model(
                target_joints_names=target_joints, has_free_flyer=self.has_free_flyer
            )
        )

        # Pin model dimensions
        self.nq = self.model.nq
        self.nv = self.model.nv
        self.nx = self.nq + self.nv

        self.get_logger().info(f"Model loaded: nq={self.model.nq}, nv={self.model.nv}")

        # Initialize MeshCat
        self.get_logger().info("Initializing MeshCat viewer...")
        self.viz_server = meshcat.Visualizer()
        self.viz_server.open()

        # Create Pinocchio visualizer
        self.viz = MeshcatVisualizer(self.model, None, self.visual_model)
        self.viz.initViewer(self.viz_server)
        self.viz.loadViewerModel()

        self.target_frame = "gripper_grasping_frame"

        # Get frame ID
        if not self.model.existFrame(self.target_frame):
            self.get_logger().error(f"Frame '{self.target_frame}' not found!")
            raise ValueError(f"Frame '{self.target_frame}' does not exist in model")

        self.frame_id = self.model.getFrameId(self.target_frame)
        self.get_logger().info(f"Target frame ID: {self.frame_id}")

        self.sensor_received = False  # Will be set in callback
        self.current_sensor_py = None
        self.x_measured = None  # Will be set in callback
        self.target_pose = None  # Will be set in callback

        self.visualizer = None  # Will be initialized in callback

        # QoS for real-time
        qos_rt = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub_sensor = self.create_subscription(
            Sensor, "sensor", self.sensor_callback_oneshot, qos_profile=qos_rt
        )

        self.get_logger().info("Waiting for initial sensor measurement...")

    def sensor_callback_oneshot(self, msg):
        """ONE-SHOT sensor callback."""
        if self.sensor_received:
            return

        try:
            self.current_sensor_py = sensor_msg_to_numpy(msg)

            # Extract measured joint state (skip wheels)
            q_measured = self.current_sensor_py.joint_state.position[
                2:
            ]  # Without wheels
            v_measured = self.current_sensor_py.joint_state.velocity[
                2:
            ]  # Without wheels

            # DEBUG: Print raw received data
            self.get_logger().info(
                f"\n{'=' * 60}\n"
                f"RAW SENSOR DATA RECEIVED:\n"
                f"{'=' * 60}\n"
                f"Full position (with wheels): {self.current_sensor_py.joint_state.position}\n"
                f"Full velocity (with wheels): {self.current_sensor_py.joint_state.velocity}\n"
                f"Arm position (wheels skipped): {q_measured}\n"
                f"Arm velocity (wheels skipped): {v_measured}\n"
                f"{'=' * 60}"
            )

            # validation: Check joints dimension
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
            self.x_measured

            # Visualize current configuration
            self.viz.display(q_full)

            # Compute measured EE position
            pin.forwardKinematics(self.model, self.data, q_full)
            pin.updateFramePlacement(self.model, self.data, self.frame_id)
            ee_measured_pose = self.data.oMf[self.frame_id].copy()  # Full SE3 pose
            ee_measured = ee_measured_pose.translation.copy()

            # Set target pose
            self.target_pose = pin.SE3(np.eye(3), np.array([2.0, 2.0, 1.0]))

            self.get_logger().info(
                f"Initial state received:\n"
                f"State dimensions: q={len(q_full)}/{self.nq}, v={len(v_full)}/{self.nv}\n"
                f"EE measured: [{ee_measured[0]:.3f}, {ee_measured[1]:.3f}, {ee_measured[2]:.3f}]\n"
                f"Target set:  [{self.target_pose.translation[0]:.3f}, {self.target_pose.translation[1]:.3f}, {self.target_pose.translation[2]:.3f}]"
            )

            # Display frames
            frames_to_display = [self.target_frame]
            frame_ids = [
                self.model.getFrameId(frame_name) for frame_name in frames_to_display
            ]
            self.viz.displayFrames(True, frame_ids)

            # Display EE position as green sphere
            self.viz.viewer["ee_sphere"].set_object(
                g.Sphere(0.04),
                g.MeshLambertMaterial(color=0x00FF00, opacity=0.7),  # Green
            )
            self.viz.viewer["ee_sphere"].set_transform(
                tf.translation_matrix(ee_measured)
            )

            # Display target pose (only a red sphere for position)
            self.viz.viewer["target_sphere"].set_object(
                g.Sphere(0.06),
                g.MeshLambertMaterial(color=0xFF0000, opacity=0.8),  # Red
            )
            self.viz.viewer["target_sphere"].set_transform(
                tf.translation_matrix(self.target_pose.translation)
            )

            # Mark as received and cleanup
            self.sensor_received = True
            self.destroy_subscription(self.sub_sensor)

            # Continue with OCP
            self.get_logger().info("Building OCP problem...")
            self.build_ocp()

            self.get_logger().info("Solving OCP...")
            converged = self.solve_ocp()

            if converged:
                # Display final state
                nq = self.model.nq
                q_final = self.solver.xs[-1][:nq]
                self._visualize_state(q_final, "final")

            # self.display_results()
            self.get_logger().info("OCP test completed!")

            # Initialize visualizer
            self.visualizer = TrajectoryVisualizer(
                viz=self.viz,
                model=self.model,
                data=self.data,
                frame_id=self.frame_id,
                target_pose=self.target_pose,
                dt=self.dt,
                logger=self.get_logger(),
            )

            # Display results
            self.visualizer.display_results(
                xs_solution=self.xs_solution,
                us_solution=self.us_solution,
                cost=self.solver.cost,
            )

            # Replay trajectory in infinite loop
            self.visualizer.replay_trajectory_loop(
                trajectory_q=self.trajectory_q,
                fps=30,
                slowdown=2.0,
                pause_between_loops=2.0,  # number of seconds to pause between loops
            )

        except Exception as e:
            self.get_logger().error(f"Error in sensor callback: {e}")
            import traceback

            traceback.print_exc()

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

        # 2D base position (7 → 4)
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

        # Build q_planar
        q_planar = np.array([x, y, cos_theta, sin_theta])

        # 2D base velocity (6 → 3)
        vx = v_ff[0]
        vy = v_ff[1]
        # vz = v_ff[2]  # not used in planar

        # ωx = v_ff[3]  # not used in planar
        # ωy = v_ff[4]  # not used in planar
        omega = v_ff[5]  # Rotational velocity around Z (yaw)

        # Build v_planar
        v_planar = np.array([vx, vy, omega])

        # Validation
        assert q_planar.shape == (4,), (
            f"q_planar shape error: {q_planar.shape}, expected (4,)"
        )
        assert v_planar.shape == (3,), (
            f"v_planar shape error: {v_planar.shape}, expected (3,)"
        )

        return q_planar, v_planar

    def build_ocp(self):
        """Build the OCP problem using OCPBuilder."""

        self.ocp_builder = OCPBuilder(
            initial_state=self.x_measured,
            rmodel=self.model,
            dt=self.dt,
            horizon_length=self.horizon_steps,
            has_free_flyer=self.has_free_flyer,
            wheel_params=None,  # Use default Tiago params
        )

        # Get state and actuation models
        state = self.ocp_builder.state
        actuation = self.ocp_builder.actuation

        running_cost_manager = CostModelManager(state, actuation)

        # Cost 1: Reach the target (main objective)
        ee_tracking_weight = 1e3
        running_cost_manager.add_frame_placement_cost(
            frame_name=self.target_frame,
            target_pose=self.target_pose,
            weight=ee_tracking_weight,
        )

        # Cost 2: State regularization (keep robot close to initial config)
        pkg_share = get_package_share_directory("tiago_simple_mpc")
        state_weights_config = os.path.join(
            pkg_share, "config", "regulation_state_weights.yaml"
        )
        self.get_logger().info(f"Config path: {state_weights_config}")
        state_reg_weight = 1e-2
        running_cost_manager.add_weighted_regulation_state_cost(
            x_ref=self.x_measured,
            config_filepath=state_weights_config,
            weight=state_reg_weight,
        )

        # Cost 3: Control regularization (keep controls small)
        control_weights_config = os.path.join(
            pkg_share, "config", "regulation_control_weights.yaml"
        )
        self.get_logger().info(f"Config path: {control_weights_config}")
        control_reg_weight = 1e-3
        running_cost_manager.add_weighted_regulation_control_cost(
            config_filepath=control_weights_config,
            weight=control_reg_weight,
        )

        terminal_cost_manager = CostModelManager(state, actuation)

        terminal_cost_manager.add_frame_placement_cost(
            frame_name=self.target_frame,
            target_pose=self.target_pose,
            weight=ee_tracking_weight * 10,  # 10x stronger at the end
        )

        self.problem = self.ocp_builder.build(
            running_cost_managers=[running_cost_manager] * self.horizon_steps,
            terminal_cost_manager=terminal_cost_manager,
            integrator_type="euler",
        )

        self.get_logger().info(
            f"OCP built with {self.horizon_steps} running nodes + 1 terminal node"
        )

    def solve_ocp(self):
        """Solve the OCP using DDP solver."""

        # Create solver
        self.solver = crocoddyl.SolverFDDP(self.problem)

        # Solver options
        self.solver.setCallbacks([crocoddyl.CallbackVerbose()])

        xs_init = [self.x_measured] * (self.horizon_steps + 1)
        us_init = [np.zeros(self.ocp_builder.nu)] * self.horizon_steps

        # Solve
        self.get_logger().info("Running FDDP solver...")
        MAX_ITER = 50  # max iteration
        converged = self.solver.solve(xs_init, us_init, MAX_ITER, False)

        # Extract trajectory (only positions, not velocities)
        nq = self.model.nq
        self.trajectory_q = [xs[:nq] for xs in self.solver.xs]

        if converged:
            self.get_logger().info(f"Solver converged in {self.solver.iter} iterations")
        else:
            self.get_logger().warn(
                f"Solver did NOT converge after {self.solver.iter} iterations"
            )

        # Store solution
        self.xs_solution = self.solver.xs
        self.us_solution = self.solver.us


def main(args=None):
    rclpy.init(args=args)

    try:
        node = OCPReachingNode()
        rclpy.spin(node)
        # Node does everything in __init__, no need to spin
        # node.destroy_node()
    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
