import numpy as np
import crocoddyl
import pinocchio as pin

import os
from ament_index_python.packages import get_package_share_directory

from tiago_simple_mpc.ocp.ocp_builder import OCPBuilder
from tiago_simple_mpc.ocp.cost_manager import CostModelManager
from tiago_simple_mpc.mpc.mpc_builder import MPCOCP


def build_cartesian_target_ocp(
    x0: np.ndarray,
    target_pose: pin.SE3,
    frame_name: str,
    model: pin.Model,
    has_free_flyer: bool,
    dt: float,
    horizon_length: int,
) -> MPCOCP:
    """Builds a Crocoddyl OCP for reaching a Cartesian target with the end-effector.


    Returns:
        MPCOCP: A class containing all the OCP informations :
            - problem
            - solver
            - dt
            - horizon_length
    """
    # Build OCP using OCPBuilder
    ocp_builder = OCPBuilder(
        initial_state=x0,
        rmodel=model,
        dt=dt,
        horizon_length=horizon_length,
        has_free_flyer=has_free_flyer,
        wheel_params=None,
    )

    # Create costs
    running_cost_manager = CostModelManager(ocp_builder.state, ocp_builder.actuation)

    # Cost 1: Reach the target (main objective)
    ee_tracking_weight = 1e3
    running_cost_manager.add_frame_placement_cost(
        frame_name=frame_name, target_pose=target_pose, weight=ee_tracking_weight
    )

    # Cost 2: State regularization (keep robot close to initial config)
    pkg_share = get_package_share_directory("tiago_simple_mpc")
    state_weights_config = os.path.join(
        pkg_share, "config", "regulation_state_weights.yaml"
    )
    state_reg_weight = 1e-2
    running_cost_manager.add_weighted_regulation_state_cost(
        x_ref=x0,
        config_filepath=state_weights_config,
        weight=state_reg_weight,
    )

    # Cost 3: Control regularization (keep controls small)
    control_weights_config = os.path.join(
        pkg_share, "config", "regulation_control_weights.yaml"
    )
    control_reg_weight = 1e-3
    running_cost_manager.add_weighted_regulation_control_cost(
        config_filepath=control_weights_config,
        weight=control_reg_weight,
    )

    terminal_cost_manager = CostModelManager(ocp_builder.state, ocp_builder.actuation)

    terminal_cost_manager.add_frame_placement_cost(
        frame_name=frame_name,
        target_pose=target_pose,
        weight=ee_tracking_weight * 10,  # 10x stronger at the end
    )

    # Finalize OCP
    problem = ocp_builder.build(
        running_cost_managers=[running_cost_manager] * horizon_length,
        terminal_cost_manager=terminal_cost_manager,
        integrator_type="euler",
    )
    solver = crocoddyl.SolverFDDP(problem)

    return MPCOCP(
        problem=problem,
        solver=solver,
        dt=dt,
        horizon_length=horizon_length,
    )
