import crocoddyl
import numpy as np
import pinocchio as pin
import yaml


class CostModelManager:
    """
    Cost structure to easily build a cost model with chainable cost functions
    """

    def __init__(self, state, actuation):
        self.state = state
        self.actuation = actuation
        self.cost_model_sum = crocoddyl.CostModelSum(self.state, self.actuation.nu)

    def add_frame_placement_cost(
        self, frame_name: str, target_pose: pin.SE3, weight: float, name: str = None
    ):
        """
        Add a frame placement (6D pose) tracking cost.

        This cost penalizes the difference between the current 6D pose (position + orientation)
        of a specified frame and a target pose. It uses the SE3 logarithm map to compute
        the pose error in a numerically stable way.

        Mathematical Formulation:
        L(x) = (weight / 2) * ||log(M_frame(q)^{-1} * M_target)||^2

        where:
        - M_frame(q) is the SE3 pose of the frame computed via forward kinematics
        - M_target is the target SE3 pose
        - log() is the SE3 logarithm map, producing a 6D tangent vector [v; ω]
            with v ∈ ℝ³ (linear) and ω ∈ ℝ³ (angular)
        - ||·|| is the Euclidean norm in ℝ⁶

        This formulation handles both position and orientation errors in a unified way,
        and is particularly robust for frames with complex local transformations.

        Args:
            frame_name (str): Name of the frame to track (e.g., 'gripper_grasping_frame').
            target_pose (pin.SE3): The target SE3 pose (position + orientation) in the world frame.
                                Can be constructed as:
                                - pin.SE3(rotation_matrix, translation_vector)
                                - pin.SE3.Identity() for identity pose
            weight (float): The scalar weight for this cost. Higher values prioritize
                        pose tracking over other costs.
            name (str, optional): A unique name for the cost. If None, defaults to
                                "frame_placement_{frame_name}".

        Returns:
            self: The CostModelManager instance for chainable calls.

        Raises:
            TypeError: If target_pose is not a pin.SE3 object.
            ValueError: If the frame doesn't exist in the model.
        """

        # Input Validation
        if not isinstance(target_pose, pin.SE3):
            raise TypeError(
                f"target_pose must be a pinocchio.SE3 object, but got {type(target_pose)}. "
                f"Create it with: pin.SE3(rotation_matrix, translation_vector)"
            )

        # Check if frame exists
        if not self.state.pinocchio.existFrame(frame_name):
            raise ValueError(
                f"Frame '{frame_name}' does not exist in the robot model. "
                f"Available frames: {[self.state.pinocchio.frames[i].name for i in range(self.state.pinocchio.nframes)]}"
            )

        # Get frame ID
        frame_id = self.state.pinocchio.getFrameId(frame_name)

        # Create residual: measures SE3 difference between frame pose and target
        # This uses the logarithm map: log(M_current^{-1} * M_target)
        # The residual is a 6D vector: [linear_error; angular_error]
        residual = crocoddyl.ResidualModelFramePlacement(
            self.state, frame_id, target_pose, self.actuation.nu
        )

        # Wrap residual in a cost model
        # The cost is: (weight/2) * ||residual||^2
        cost = crocoddyl.CostModelResidual(self.state, residual)

        # Generate default name if not provided
        if name is None:
            name = f"frame_placement_{frame_name}"

        # Add to cost sum
        self.cost_model_sum.addCost(name=name, cost=cost, weight=weight)

        return self

    def add_regulation_state_cost(
        self, x_ref: np.ndarray, weight: float, name: str = "regulation_state"
    ):
        """
        Add a state regulation cost.
        Isotropic weighting.

        This cost penalizes the difference between the current state `x` and a
        specified reference state `x_ref`.

        Mathematical Formulation:
        L(x) = (weight / 2) * ||x - x_ref||^2

        Args:
            x_ref (np.ndarray): The reference state vector [q_ref, v_ref] to track.
                            Its size must be equal to the state dimension (nx).
            weight (float): The scalar weight for this cost.
            name (str): A unique name for the cost.

        Returns:
            self: The CostModelManager instance for chainable calls.
        """

        # Input Validation
        if not isinstance(x_ref, np.ndarray):
            raise TypeError(f"x_ref must be a numpy array, but got {type(x_ref)}.")
        if x_ref.shape != (self.state.nx,):
            raise ValueError(
                f"The reference state x_ref must have shape ({self.state.nx},), "
                f"but got shape {x_ref.shape}."
            )

        residual = crocoddyl.ResidualModelState(self.state, x_ref, self.actuation.nu)
        cost = crocoddyl.CostModelResidual(self.state, residual)
        self.cost_model_sum.addCost(name=name, cost=cost, weight=weight)
        return self

    def add_weighted_regulation_state_cost(
        self,
        x_ref: np.ndarray,
        config_filepath: str,
        weight: float = 1.0,
        name: str = "regulation_weighted_state",
    ):
        """Adds a weighted state regulation cost by loading weights from a YAML file.

        This cost penalizes the state deviation `x - x_ref` using an anisotropic quadratic function.
        The weights for each state variable are specified in the provided configuration file.

        Mathematical Formulation:

            The cost is defined as: L(x) = (weight / 2) * ||x ⊖ x_ref||_w^2
            Where:
            - `x` is the state vector.
            - `x_ref` is the reference state vector.
            - `w` is the vector of weights loaded from the YAML file.
            - `weight` is a global scalar weight for this cost term.

            Expanded Form:
            L(x) = weight * (1/2) * (w_1*(x_1 - x_ref_1)^2 + w_2*(x_2 - x_ref_2)^2 + ... + w_n*(x_n - x_ref_n)^2)

        Args:
            config_filepath (str): The path to the YAML file with the state weights. size is nv * 2
            x_ref (np.ndarray): The reference state vector [q_ref, v_ref] to track. size is nv + nq
            weight (float): A global scalar weight for the entire cost term.
            name (str): A unique name for the cost.

        Returns:
            self: The CostModelManager instance for chainable calls.
        """

        # Input Validation
        if not isinstance(x_ref, np.ndarray):
            raise TypeError(f"x_ref must be a numpy array, but got {type(x_ref)}.")
        if x_ref.shape != (self.state.nx,):
            raise ValueError(
                f"The reference state x_ref must have shape ({self.state.nx},), "
                f"but got shape {x_ref.shape}."
            )

        # Load and validate Weights from YAML file
        try:
            with open(config_filepath, "r") as f:
                config_data = yaml.safe_load(f)

                q_weights = np.array(config_data["q_weights"])
                v_weights = np.array(config_data["v_weights"])

                weights = np.concatenate([q_weights, v_weights])

        except FileNotFoundError:
            raise FileNotFoundError(
                f"Configuration file not found at: {config_filepath}"
            )
        except KeyError as e:
            raise KeyError(
                f"Key {e} not found in the configuration file: {config_filepath}. Both 'q_weights' and 'v_weights' are required."
            )
        except Exception as e:
            raise RuntimeError(
                f"Failed to load or parse weights from {config_filepath}: {e}"
            )

        # Check that the final weight vector matches the model's state dimension
        if len(weights) != 2 * self.state.nv:
            raise ValueError(
                f"Combined weights from file have size {len(weights)}, "
                f"but the model's state dimension is {2 * self.state.nv}."
            )

        # Create and add cost to the model
        residual = crocoddyl.ResidualModelState(self.state, x_ref, self.actuation.nu)
        activation = crocoddyl.ActivationModelWeightedQuad(weights)
        cost = crocoddyl.CostModelResidual(self.state, activation, residual)
        self.cost_model_sum.addCost(name=name, cost=cost, weight=weight)
        return self

    def add_regulation_control_cost(
        self, weight: float, u_ref: np.ndarray = None, name: str = "regulation_control"
    ):
        """
        Add a command regulation cost.
        Isotropic weighting.

        This cost penalizes the difference between the current control `u` and a
        reference control `u_ref` (feedforward term). If `u_ref` is not provided,
        the cost defaults to penalizing the control effort `u`.

        Mathematical Formulation:
            L(u) = (weight / 2) * ||u - u_ref||^2

            Where:
            - `u` is the control vector.
            - `u_ref` is the reference control vector (defaults to a zero vector).
            - `weight` is a global scalar weight for this cost term.

        Args:
            weight (float): The scalar weight for this cost.
            u_ref (np.ndarray, optional): The reference control vector to track.
                                        Its size must be equal to the control dimension (nu).
                                        If None, a zero vector is used. Defaults to None.
            name (str): A unique name for the cost.

        Returns:
            self: The CostModelManager instance for chainable calls.
        """

        # Define and validate the reference control
        reference = u_ref if u_ref is not None else np.zeros(self.actuation.nu)
        if not isinstance(reference, np.ndarray):
            raise TypeError(f"u_ref must be a numpy array, but got {type(reference)}.")
        if reference.shape != (self.actuation.nu,):
            raise ValueError(
                f"The reference control u_ref must have shape ({self.actuation.nu},), "
                f"but got shape {reference.shape}."
            )

        residual = crocoddyl.ResidualModelControl(self.state, reference)
        cost = crocoddyl.CostModelResidual(self.state, residual)
        self.cost_model_sum.addCost(name=name, cost=cost, weight=weight)
        return self

    def add_weighted_regulation_control_cost(
        self,
        config_filepath: str,
        weight: float = 1.0,
        u_ref: np.ndarray = None,
        name: str = "regulation_weighted_control",
    ):
        """
        Adds a weighted command regulation cost by loading weights from a YAML file.

        This cost penalizes the control input `u - u_ref` using an anisotropic quadratic function.
        The weights for each actuator are specified in the provided configuration file.

        Mathematical Formulation:
            L(u) = (weight / 2) * ||u - u_ref||_w^2

            Where:
            - `u` is the control vector.
            - `u_ref` is the reference control vector (defaults to a zero vector).
            - `w` is the vector of weights loaded from the YAML file.
            - `weight` is a global scalar weight for this cost term.

            Expanded Form:
            L(u) = (weight / 2) * sum_{i=1 to nu} [ w_i * (u_i - u_ref_i)^2 ]


        Args:
            config_filepath (str): The path to the YAML file containing the control weights.
                               The file must contain a key 'u_weights' with a list of numbers.
            weight (float): A global scalar weight for the entire cost term.
            u_ref (np.ndarray, optional): The reference control vector (feedforward).
                                        Must have size `nu`. Defaults to a zero vector if None.
            name (str): A unique name for the cost.

        Returns:
            self: The CostModelManager instance for chainable calls.
        """

        # Load and validate Weights from YAML file
        try:
            with open(config_filepath, "r") as f:
                config_data = yaml.safe_load(f)

                weights_list = config_data["u_weights"]
                weights = np.array(weights_list)

        except FileNotFoundError:
            raise FileNotFoundError(
                f"Configuration file not found at: {config_filepath}"
            )
        except KeyError:
            raise KeyError(
                f"'u_weights' key not found in the configuration file: {config_filepath}"
            )
        except Exception as e:
            raise RuntimeError(
                f"Failed to load or parse weights from {config_filepath}: {e}"
            )

        if weights.ndim != 1 or len(weights) != self.actuation.nu:
            raise ValueError(
                f"Loaded 'u_weights' must be a 1D array of size {self.actuation.nu}, "
                f"but got size {len(weights)}."
            )

        # Define and validate the reference control
        reference = u_ref if u_ref is not None else np.zeros(self.actuation.nu)
        if not isinstance(reference, np.ndarray):
            raise TypeError(f"u_ref must be a numpy array, but got {type(reference)}.")
        if reference.shape != (self.actuation.nu,):
            raise ValueError(
                f"The reference control u_ref must have shape ({self.actuation.nu},), "
                f"but got shape {reference.shape}."
            )

        # Create and add cost to the model
        residual = crocoddyl.ResidualModelControl(self.state, reference)
        activation = crocoddyl.ActivationModelWeightedQuad(weights)
        cost = crocoddyl.CostModelResidual(self.state, activation, residual)
        self.cost_model_sum.addCost(name=name, cost=cost, weight=weight)

        return self

    def get_costs(self):
        """
        Return the final constructed CostModelManager object.

        Returns:
            crocoddyl.CostModelManager: The set of configured costs.
        """
        return self.cost_model_sum

    def display_costs(self):
        """
        Prints a detailed, human-readable summary of all costs configured
        in this cost model.
        """
        print("\n---- Cost Model Summary ----")

        costs_map = self.cost_model_sum.costs

        if not costs_map:
            print("  No costs have been configured in this model.")
            print("--------------------------\n")
            return

        print(f"  Total number of costs: {len(costs_map)}")
        print("--------------------------")

        for item in costs_map:
            name = item.key

            cost_item = item.data()

            weight = cost_item.weight
            cost_type = type(cost_item.cost).__name__

            print(f"  > Cost Name: '{name}'")
            print(f"    - Weight: {weight}")
            print(f"    - Type  : {cost_type}")

        print("--------------------------\n")
