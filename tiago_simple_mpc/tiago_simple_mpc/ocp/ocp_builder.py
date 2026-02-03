import pinocchio as pin
import crocoddyl
import numpy as np

from tiago_simple_mpc.ocp.cost_manager import CostModelManager

# Import custom differential drive actuation
from tiago_simple_mpc.core.actuation_planar_drive import ActuationModelPlanarDrive


class OCPBuilder:
    """Builds a Crocoddyl Optimal Control Problem (OCP).

    Attributes:
        rmodel (pin.Model): The Pinocchio model of the robot.
        initial_state (np.ndarray): The starting state (q, v) of the robot.
        dt (float): The time step duration for the integration scheme.
        horizon_length (int): The number of nodes (time steps) in the OCP horizon.
        has_free_flyer (bool): Whether the robot has a planar mobile base.
        state (crocoddyl.StateMultibody): The state model for the multibody system.
        actuation (crocoddyl.ActuationModelAbstract): The actuation model (PlanarDrive for
            mobile base, Full for fixed base).
        nu (int): Number of control inputs.
    """

    def __init__(
        self,
        initial_state: np.ndarray,
        rmodel: pin.Model,
        dt: float,
        horizon_length: int,
        has_free_flyer: bool,
        wheel_params: dict = None,
    ):
        """Initializes the OCPBuilder.

        Args:
            initial_state (np.ndarray): Initial state vector [q, v].
            rmodel (pin.Model): The Pinocchio robot model.
            dt (float): The time step (delta t) for each action model.
            horizon_length (int): The number of running nodes in the OCP. The total
                                trajectory will have N+1 nodes.
            has_free_flyer (bool): Whether the robot has a planar free-flyer base.
            wheel_params (dict): Parameters for differential drive (if has_free_flyer=True).
                                Expected keys: 'radius', 'separation', 'left_idx', 'right_idx'
        """
        self.rmodel = rmodel
        self.initial_state = initial_state
        self.dt = dt
        self.horizon_length = horizon_length
        self.has_free_flyer = has_free_flyer

        self.state = crocoddyl.StateMultibody(self.rmodel)  # input x = (q, v)

        # Configure actuation model based on robot type
        if self.has_free_flyer:
            # Default Tiago wheel parameters (can be overridden)
            default_wheel_params = {
                "radius": 0.0985,  # m
                "separation": 0.4044,  # m
                "left_idx": 3,  # Index in velocity vector
                "right_idx": 4,
            }

            # Merge with user-provided params
            if wheel_params is not None:
                default_wheel_params.update(wheel_params)

            params = default_wheel_params

            self.actuation = ActuationModelPlanarDrive(
                self.state,
                wheel_radius=params["radius"],
                wheel_separation=params["separation"],
            )

            self.nu = self.state.nv - 3 + 2  # Remove base DOFs, add 2 wheels
            print(f"[OCPBuilder] Differential drive: nu = {self.nu}")

        else:
            # Fixed base robot: full actuation
            self.actuation = crocoddyl.ActuationModelFull(self.state)
            self.nu = self.state.nv
            print(f"🔧 [OCPBuilder] Full actuation: nu = {self.nu}")

    def build(
        self,
        running_cost_managers: list[CostModelManager],
        terminal_cost_manager: CostModelManager,
        integrator_type: str = "euler",
    ) -> crocoddyl.ShootingProblem:
        """Constructs the Crocoddyl shooting problem.

        Args:
            running_cost_managers (List[CostModelManager]): A list of cost managers, one for
                each running node of the horizon.
            terminal_cost_manager (CostModelManager): The cost manager for the terminal node.
            integrator_type (str, optional): The integration scheme to use.
                Options: 'euler' or 'rk4'. Defaults to 'euler'.

        Returns:
            crocoddyl.ShootingProblem: The fully assembled optimal control problem.
        """
        if len(running_cost_managers) != self.horizon_length:
            raise ValueError(
                f"number of 'running_cost_managers' ({len(running_cost_managers)}) "
                f"must be equal to 'horizon_length' ({self.horizon_length})."
            )

        running_models = self._create_running_models(
            running_cost_managers, integrator_type
        )
        terminal_model = self._create_terminal_model(
            terminal_cost_manager, integrator_type
        )

        problem = crocoddyl.ShootingProblem(
            self.initial_state, running_models, terminal_model
        )

        return problem

    def _create_running_models(
        self, cost_managers: list[CostModelManager], integrator_type: str = "euler"
    ) -> list:
        """Creates the list of integrated action models for the running nodes.

        Args:
            cost_managers (List[CostModelManager]): List of cost managers for the horizon.
            integrator_type (str): The integration scheme ('euler' or 'rk4').

        Returns:
            List[crocoddyl.IntegratedActionModelAbstract]: The list of configured action models.
        """
        running_models = []
        for cost_manager in cost_managers:
            dmodel = crocoddyl.DifferentialActionModelFreeFwdDynamics(
                self.state, self.actuation, cost_manager.cost_model_sum
            )
            if integrator_type.lower() == "euler":
                running_model = crocoddyl.IntegratedActionModelEuler(dmodel, self.dt)
            elif integrator_type.lower() == "rk4":
                running_model = crocoddyl.IntegratedActionModelRK(
                    dmodel, crocoddyl.RKType(4), self.dt
                )
            else:
                raise ValueError(
                    f"Unknown integrator type: '{integrator_type}'. Choose 'euler' or 'rk4'."
                )

            running_models.append(running_model)

        return running_models

    def _create_terminal_model(
        self, cost_manager: CostModelManager, integrator_type: str = "euler"
    ) -> crocoddyl.IntegratedActionModelEuler:
        """Creates the integrated action model for the terminal node.

        Args:
            cost_manager (CostModelManager): The cost manager for the terminal node.
            integrator_type (str): The integration scheme ('euler' or 'rk4').

        Returns:
            crocoddyl.IntegratedActionModelAbstract: The configured terminal action model.
        """
        dmodel = crocoddyl.DifferentialActionModelFreeFwdDynamics(
            self.state, self.actuation, cost_manager.cost_model_sum
        )

        if integrator_type.lower() == "euler":
            terminal_model = crocoddyl.IntegratedActionModelEuler(dmodel, 0.0)
        elif integrator_type.lower() == "rk4":
            terminal_model = crocoddyl.IntegratedActionModelRK(
                dmodel, crocoddyl.RKType(4), 0.0
            )
        else:
            raise ValueError(
                f"Unknown integrator type: '{integrator_type}'. Choose 'euler' or 'rk4'."
            )

        return terminal_model
