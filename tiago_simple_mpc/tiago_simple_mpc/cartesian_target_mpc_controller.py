import numpy as np
import crocoddyl


class CartesianMPCController:
    def __init__(self, pin_model, target_frame_name, dt=1e-2, T=10):
        self.pin_model = pin_model
        self.dt = dt
        self.T = T

        # On trouve l'ID du frame qu'on veut piloter (le bout du bras)
        self.frame_id = self.pin_model.getFrameId(target_frame_name)

        self.state = crocoddyl.StateMultibody(pin_model)
        self.actuation = crocoddyl.ActuationModelFull(self.state)

        # Poids des coûts
        self.weight_pos = 100.0  # Priorité absolue : atteindre la cible
        self.weight_state = 0.1  # Faible : garder une posture stable si possible
        self.weight_ctrl = 0.001  # Très faible : économie d'énergie

        self.problem = self._create_problem()
        self.solver = crocoddyl.SolverFDDP(self.problem)

        # Warm-start
        self.xs = [self.state.zero()] * (self.T + 1)
        self.us = [np.zeros(self.actuation.nu)] * self.T

        print(f"[MPC] Tracking frame '{target_frame_name}' (ID: {self.frame_id})")

    def _create_stage_model(self, target_pos_init):
        cost_model = crocoddyl.CostModelSum(self.state, self.actuation.nu)

        # --- Coût 1 : Position du End-Effector (Le plus important) ---
        # On veut que le frame aille vers target_pos_init
        frame_ref = target_pos_init  # Vecteur 3D (x, y, z)
        # ResidualModelFrameTranslation compare la pos du frame à la ref
        pos_residual = crocoddyl.ResidualModelFrameTranslation(
            self.state, self.frame_id, frame_ref
        )
        # Activation quadratique standard
        pos_cost = crocoddyl.CostModelResidual(self.state, pos_residual)
        cost_model.addCost("gripperPos", pos_cost, self.weight_pos)

        # --- Coût 2 : Régularisation d'état (Posture par défaut) ---
        # Pour éviter que le bras parte dans tous les sens (drift)
        x_default = self.state.zero()  # Ou une posture confortable
        state_residual = crocoddyl.ResidualModelState(self.state, x_default)
        state_reg = crocoddyl.CostModelResidual(self.state, state_residual)
        cost_model.addCost("stateReg", state_reg, self.weight_state)

        # --- Coût 3 : Régularisation de commande ---
        u_residual = crocoddyl.ResidualModelControl(self.state, self.actuation.nu)
        ctrl_reg = crocoddyl.CostModelResidual(self.state, u_residual)
        cost_model.addCost("ctrlReg", ctrl_reg, self.weight_ctrl)

        # Modèle dynamique
        DAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(
            self.state, self.actuation, cost_model
        )
        return crocoddyl.IntegratedActionModelEuler(DAM, self.dt)

    def _create_problem(self):
        x0 = self.state.zero()
        # On initialise la cible à (0,0,0) pour l'instant, on la mettra à jour
        target_init = np.zeros(3)

        running_model = self._create_stage_model(target_init)
        terminal_model = self._create_stage_model(target_init)

        return crocoddyl.ShootingProblem(x0, [running_model] * self.T, terminal_model)

    def update_target_position(self, target_pos_3d):
        """
        Met à jour la cible (x,y,z) pour tout l'horizon
        """
        # Mettre à jour les modèles courants
        for model in self.problem.runningModels:
            # On accède au coût "gripperPos" -> son residuel -> sa reference
            model.differential.costs.costs[
                "gripperPos"
            ].cost.residual.reference = target_pos_3d

        # Mettre à jour le modèle terminal
        self.problem.terminalModel.differential.costs.costs[
            "gripperPos"
        ].cost.residual.reference = target_pos_3d

    def solve(self, x_measured, target_pos_3d):
        """
        :param x_measured: État robot [q, v]
        :param target_pos_3d: Cible cartésienne [x, y, z]
        """
        # 1. Mise à jour de la cible dans la fonction de coût
        self.update_target_position(target_pos_3d)

        # 2. Mise à jour de l'état initial
        self.problem.x0 = x_measured

        # 3. Résolution
        self.solver.solve(self.xs, self.us, 1)  # 1 itération (MPC temps réel)

        self.xs = list(self.solver.xs)
        self.us = list(self.solver.us)

        return self.solver.us[0]
