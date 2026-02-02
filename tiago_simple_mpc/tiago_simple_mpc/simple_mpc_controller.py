import numpy as np
import crocoddyl


class MPCController:
    def __init__(self, pin_model, dt=1e-2, T=10):
        """
        :param pin_model: The Pinocchio model (reduced to the arm)
        :param dt: Time step (e.g., 10ms <=> 100Hz)
        :param T: Prediction horizon (number of nodes)
        """
        self.pin_model = pin_model
        self.dt = dt
        self.T = T

        self.state = crocoddyl.StateMultibody(pin_model)
        self.actuation = crocoddyl.ActuationModelFloatingBase(
            self.state
        )  # ActuationModelFull

        self.weight_state_reg = 10.0
        self.weight_ctrl_reg = 0.001

        self.problem = self._create_problem()

        self.solver = crocoddyl.SolverFDDP(self.problem)

        # Callbacks for debug (optional, slows down a bit)
        # self.solver.setCallbacks([crocoddyl.CallbackVerbose()])

        # Memory for warm-start (hot start)
        self.xs = [self.state.zero()] * (self.T + 1)
        self.us = [np.zeros(self.actuation.nu)] * self.T

        print(f"[MPC] Initialized with T={T}, dt={dt}, nq={pin_model.nq}")

    def _create_stage_model(self, x_ref):
        """Create the action model for one time step (t < T)"""

        # Cost management
        cost_model = crocoddyl.CostModelSum(self.state, self.actuation.nu)

        # C1 : State regularization (going towards x_ref)
        x_residual = crocoddyl.ResidualModelState(self.state, x_ref)
        x_activation = crocoddyl.ActivationModelWeightedQuad(
            np.array(
                [1] * self.pin_model.nq + [0.1] * self.pin_model.nv
            )  # Strong pressure on position, weak on velocity
        )
        x_reg = crocoddyl.CostModelResidual(self.state, x_activation, x_residual)
        cost_model.addCost("stateReg", x_reg, self.weight_state_reg)

        # C2 : Control regularization (minimize u)
        u_residual = crocoddyl.ResidualModelControl(self.state, self.actuation.nu)
        u_reg = crocoddyl.CostModelResidual(self.state, u_residual)
        cost_model.addCost("ctrlReg", u_reg, self.weight_ctrl_reg)

        DAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(
            self.state, self.actuation, cost_model
        )

        return crocoddyl.IntegratedActionModelEuler(DAM, self.dt)

    def _create_problem(self):
        """Build the shooting problem"""
        x0 = self.state.zero()

        # Running models
        running_model = self._create_stage_model(x0)

        # Terminal model
        terminal_model = self._create_stage_model(x0)

        passivity = False  # Set to true to add pure gravity (if needed)
        if passivity:
            # Trick: if u=0, the model just simulates passive physics
            running_model.differential.armature = np.array([0.1] * self.pin_model.nv)

        problem = crocoddyl.ShootingProblem(
            x0, [running_model] * self.T, terminal_model
        )
        return problem

    def update_target(self, x_ref):
        """Update the target (x_ref) in all cost models"""
        # The target x_ref is a vector of size (nq + nv)

        # For each time node (0 to T-1)
        for model in self.problem.runningModels:
            model.differential.costs.costs["stateReg"].cost.residual.reference = x_ref

        # And for the final node
        self.problem.terminalModel.differential.costs.costs[
            "stateReg"
        ].cost.residual.reference = x_ref

    def solve(self, x_measured, x_ref):
        """
        Main function called at each loop.
        :param x_measured: Current state of the robot [q, v]
        :param x_ref: Desired target state [q_ref, v_ref]
        :return: The torque to apply
        """

        # Update the target
        self.update_target(x_ref)

        # Set the initial state of the optimization problem (current state of the robot)
        self.problem.x0 = x_measured

        # Solve (warm-start with previous solutions)
        # Shift the previous solution (receding horizon):
        # xs[t] becomes xs[t+1]... and duplicate the last one
        # It's a simple heuristic to help the solver

        self.solver.solve(self.xs, self.us, 1)  # 1 iteration max (real-time MPC)

        # Retrieve results for the next loop
        self.xs = list(self.solver.xs)
        self.us = list(self.solver.us)

        # Return the first optimal action u0
        # self.solver.us[0] contains the torques computed for time t
        return self.solver.us[0]
