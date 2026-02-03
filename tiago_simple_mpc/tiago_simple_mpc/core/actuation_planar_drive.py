"""
Custom Actuation Model for Planar Mobile Manipulator
Handles differential drive kinematics for wheeled base + arm joints
"""

import numpy as np
import crocoddyl


class ActuationModelPlanarDrive(crocoddyl.ActuationModelAbstract):
    """
    Actuation model for a planar differential drive robot with manipulator arm.

    Maps control inputs [u_wheel_left, u_wheel_right, u_arm_joints...]
    to generalized forces [tau_base_x, tau_base_y, tau_base_theta, tau_arm].

    Key concept:
    - Wheel torques generate forces on the planar base via differential drive kinematics
    - The Jacobian relates wheel velocities to base velocities
    - We use the transpose to map wheel torques to base forces

    STATE STRUCTURE (reduced model):
    - q: [x, y, cos(θ), sin(θ), q_arm1, ..., q_arm7]  (11D: planar base + 7 arm)
    - v: [vx, vy, ωz, v_arm1, ..., v_arm7]  (10D: base 3DOF + 7 arm)
    - NO wheel positions/velocities in state!

    Base representation:
    - Position: (x, y) ∈ ℝ²
    - Orientation: θ represented as (cos θ, sin θ) on unit circle S¹
    - Velocity: (vx, vy, ωz) in local frame
    """

    def __init__(self, state, wheel_radius, wheel_separation):
        """
        Args:
            state: Crocoddyl state (reduced, without wheel joints)
                   nq = 11 (x, y, cos(θ), sin(θ), 7 arm joints)
                   nv = 10 (vx, vy, ωz, 7 arm velocities)
            wheel_radius: Radius of wheels (m)
            wheel_separation: Distance between left/right wheels (m)
        """
        # nu = 2 (wheels) + n_arm_joints
        # state.nv = 3 (base) + 7 (arm) = 10
        nu = 2 + (state.nv - 3)  # 2 wheels + arm joints
        crocoddyl.ActuationModelAbstract.__init__(self, state, nu)

        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation

        r = wheel_radius
        d = wheel_separation

        # Differential drive Jacobian: maps [ω_left, ω_right] → [v_x, v_y, ω_z]
        # v_x = r * (ω_L + ω_R) / 2
        # v_y = 0  (non-holonomic constraint)
        # ω_z = r * (ω_R - ω_L) / d
        self.J_diff = np.array(
            [
                [r / 2, r / 2],  # v_x contribution
                [0.0, 0.0],  # v_y = 0 (non-holonomic)
                [-r / d, r / d],  # ω_z contribution
            ]
        )

        # Torque matrix:
        # Maps [τ_L, τ_R] → [F_x, F_y, M_z]
        self.J_force = np.array(
            [
                [1 / r, 1 / r],  # F_x = (τ_L + τ_R) / r
                [0.0, 0.0],  # F_y = 0 (non-holonomic)
                [-d / (2 * r), d / (2 * r)],  # M_z = d/(2r) * (τ_R - τ_L)
            ]
        )

        print("[ActuationPlanarDrive] Initialized (REDUCED STATE):")
        print(
            f"  nq = {state.nq} (4 planar base + {state.nq - 4} arm) - NO wheel joints"
        )
        print(f"  nv = {state.nv} (3 base velocities + {state.nv - 3} arm)")
        print(f"  nu = {nu} (2 wheels + {nu - 2} arm joints)")
        print(f"  Wheel radius: {wheel_radius:.4f} m")
        print(f"  Wheel separation: {wheel_separation:.4f} m")
        print(f"  Force Jacobian (wheels→base):\n{self.J_force}")

    def calc(self, data, x, u):
        """
        Compute generalized forces from control inputs.

        Args:
            data: Actuation data (output)
            x: State [q, v] (reduced, without wheels)
            u: Controls [u_wheel_L, u_wheel_R, u_arm...]

        INPUT:  u = [τ_L, τ_R, τ_arm1, τ_arm2, ..., τ_arm7]
                    └──┬──┘   └──────────┬──────────────┘
                Wheels (2)           Arms (7)

        OUTPUT: τ = [F_x, F_y, M_z, τ_arm1, τ_arm2, ..., τ_arm7]
                    └────┬────┘   └──────────┬──────────────┘
                    Base (3)            Arms (7)
        """
        # Extract controls
        u_wheels = u[0:2]  # [torque_left, torque_right]
        u_arm = u[2:]  # Arm joint torques

        assert u_wheels.shape == (2,), f"u_wheels should be (2,), got {u_wheels.shape}"
        assert u_arm.shape == (self.state.nv - 3,), (
            f"u_arm should be ({self.state.nv - 3},), got {u_arm.shape}"
        )

        # Map wheel torques to base forces via Jacobian
        # τ_base = J * τ_wheels
        tau_base = self.J_force @ u_wheels

        # Build generalized force vector (reduced)
        # Structure: [tau_base (3), tau_arm (7)]
        data.tau[:3] = tau_base  # Base forces [F_x, F_y, M_z]
        data.tau[3:] = u_arm  # Arm torques

    def calcDiff(self, data, x, u):
        """
        Compute derivatives of actuation mapping.

        For linear actuation τ = f(u), we have:
        - ∂τ/∂x = 0 (no state dependency)
        - ∂τ/∂u = actuation matrix

        dtau_du structure (10 x 9):
                u_L   u_R  u_arm[0] ... u_arm[6]
        τ[0]  │ 1/r   1/r      0    ...    0    │  F_x
        τ[1]  │  0     0       0    ...    0    │  F_y
        τ[2]  │-d/2r  d/2r     0    ...    0    │  M_z
        τ[3]  │  0     0       1    ...    0    │  τ_arm1
        τ[4]  │  0     0       0    ...    0    │  τ_arm2
        ...   │ ...   ...     ...   ...   ...   │  ...
        τ[9]  │  0     0       0    ...    1    │  τ_arm7
        """
        # Initialize to zero
        data.dtau_dx.fill(0.0)
        data.dtau_du.fill(0.0)

        # ∂τ/∂u mapping:
        # Base influenced by wheels
        data.dtau_du[0:3, 0:2] = self.J_force  # (3, 2)

        # Arm joints actuated directly
        n_arm = self.state.nv - 3
        data.dtau_du[3:, 2:] = np.eye(n_arm)  # (7, 7)

    def createData(self):
        """Create data container for this actuation model."""
        data = ActuationDataPlanarDrive(self)
        return data


class ActuationDataPlanarDrive(crocoddyl.ActuationDataAbstract):
    """
    Data container for ActuationModelPlanarDrive.
    Inherits standard structure from Crocoddyl.
    """

    def __init__(self, model):
        crocoddyl.ActuationDataAbstract.__init__(self, model)
