from dataclasses import dataclass
import crocoddyl
import numpy as np
import pinocchio as pin


@dataclass
class MPCOCP:
    problem: crocoddyl.ShootingProblem
    solver: crocoddyl.SolverFDDP
    dt: float
    horizon_length: int


class MPCController:
    def __init__(
        self,
        ocp: MPCOCP,
        x0: np.ndarray,
        u0: np.ndarray | None = None,
        max_iter: int = 5,
        verbose: bool = True,
    ):
        """
        Initialize MPC controller.

        Args:
            ocp: MPCOCP instance (problem + solver + dt + horizon_length)
            x0: Initial state [q, v]
            u0: Initial control guess (if None, uses zeros)
            max_iter: Max FDDP iterations per MPC step
            verbose: Print convergence info
        """
        self.ocp = ocp
        self.dt = ocp.dt
        self.N = ocp.horizon_length

        # Compteur d'itérations MPC
        self.k = 0
        self.max_iter = max_iter
        self.verbose = verbose

        # Extract dimensions from first running model
        model0 = ocp.problem.runningModels[0]
        self.nx = model0.state.nx
        self.nu = model0.nu

        # Initialize control guess
        if u0 is None:
            u0 = np.zeros(self.nu)

        # Initialize trajectories for warm-start
        self.xs_init = [x0.copy() for _ in range(self.N + 1)]
        self.us_init = [u0.copy() for _ in range(self.N)]

        # Set initial state in OCP
        self.ocp.problem.x0 = x0.copy()

    def _update_initial_state(self, x_meas: np.ndarray):
        """
        Update the initial state of the OCP with measured state.
        Must be called before each OCP solve.

        Args:
            x_meas: Measured state from sensors
        """
        self.xs_init[0] = x_meas.copy()
        self.ocp.problem.x0 = x_meas.copy()

    def update_target(self, new_target: pin.SE3):
        """
        Update target pose during MPC execution.

        Args:
            new_target: New SE3 target pose

        TODO: Implement reference update in running models cost functions
        """
        # TODO: Update frame placement cost reference
        # for model in self.ocp.problem.runningModels:
        #     frame_cost = model.differential.costs.costs["frame_placement"].cost
        #     frame_cost.ref = new_target
        #
        # # Update terminal cost too
        # terminal_cost = self.ocp.problem.terminalModel.differential.costs.costs["frame_placement"].cost
        # terminal_cost.ref = new_target

        if self.verbose:
            print("⚠️  update_target() called but not yet implemented!")

    def reset(self, x0: np.ndarray, u0: np.ndarray | None = None):
        """
        Reset MPC controller to new initial state.

        Args:
            x0: New initial state
            u0: New initial control guess (optional)
        """
        self.k = 0

        if u0 is None:
            u0 = np.zeros(self.nu)

        self.xs_init = [x0.copy() for _ in range(self.N + 1)]
        self.us_init = [u0.copy() for _ in range(self.N)]
        self.ocp.problem.x0 = x0.copy()

    def step(self, x_meas: np.ndarray, max_iter: int | None = None):
        """
        Execute one MPC step (solve OCP and return optimal control).

        Args:
            x_meas: Current measured state
            max_iter: Override default max iterations (optional)

        Returns:
            tuple: (dt, u_optimal)
                - dt: Time step duration
                - u_optimal: Optimal control to apply
        """
        self._update_initial_state(x_meas)

        if max_iter is None:
            max_iter = self.max_iter

        converged = self.ocp.solver.solve(
            self.xs_init,
            self.us_init,
            max_iter,
            False,
        )

        # logger
        if self.verbose:
            if converged:
                print(
                    f"SUCCESS: FDDP {self.k} converged in {self.ocp.solver.iter} iterations."
                )
            else:
                print(
                    f"WARNING: FDDP {self.k} did not converge (iter={self.ocp.solver.iter}, cost={self.ocp.solver.cost})."
                )

        # get result from solver to be used as next input
        u_optimal = self.ocp.solver.us[0].copy()

        # Update warm start
        xs = self.ocp.solver.xs
        us = self.ocp.solver.us

        self.xs_init = [xs[i].copy() for i in range(1, len(xs))]
        self.xs_init.append(xs[-1].copy())

        self.us_init = [us[i].copy() for i in range(1, len(us))]
        self.us_init.append(us[-1].copy())

        self.k += 1
        return self.dt, u_optimal
