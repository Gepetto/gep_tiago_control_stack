import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
import pinocchio as pin
import numpy as np


class ModelLoaderNode(Node):
    """Create a node to get model from the URDF."""

    def __init__(self):
        super().__init__("temp_model_loader")
        self.urdf_xml = None

        # QoS Latched
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(String, "/robot_description", self.callback, qos)

    def callback(self, msg):
        self.urdf_xml = msg.data


def load_reduced_pinocchio_model(target_joints_names, has_free_flyer=False):
    """
    This function :
    1. Creates a temporary ROS node.
    2. Waits (blocks) until it receives the URDF from Gazebo/Robot State Publisher.
    3. Builds the full Pinocchio model (with or without FreeFlyer).
    4. Reduces the model to keep only 'target_joints_names'.
    5. Returns (model, data).
    """

    # Retrieving the XML via ROS
    print("[ModelLoader] Waiting for robot description (/robot_description)...")

    loader_node = ModelLoaderNode()

    # Loop until the message is received
    while rclpy.ok() and loader_node.urdf_xml is None:
        rclpy.spin_once(loader_node, timeout_sec=0.1)

    if loader_node.urdf_xml is None:
        raise RuntimeError("Unable to retrieve URDF. Is Gazebo running?")

    print("[ModelLoader] URDF received! Building Pinocchio model...")
    full_urdf_string = loader_node.urdf_xml

    # Destroy the temporary node to clean up
    loader_node.destroy_node()

    # Building the Full Model
    if has_free_flyer:
        full_model = pin.buildModelFromXML(full_urdf_string, pin.JointModelFreeFlyer())
        print("[ModelLoader] Building model with FreeFlyer.")
    else:
        full_model = pin.buildModelFromXML(full_urdf_string)
        print("[ModelLoader] Building model (Fixed Base).")

    # Identifying joints to lock
    locked_joint_ids = []

    # List all joints of the full robot
    for name in full_model.names:
        if name == "universe":
            continue

        # If the name is not in our target list, we lock it
        if name not in target_joints_names:
            if full_model.existJointName(name):
                locked_joint_ids.append(full_model.getJointId(name))

    # Building the Reduced Model
    reduced_model = pin.buildReducedModel(
        full_model,
        list_of_geom_models=[],
        list_of_joints_to_lock=locked_joint_ids,
        reference_configuration=np.zeros(full_model.nq),
    )[0]
    reduced_data = reduced_model.createData()

    print(
        f"[ModelLoader] Reduced model generated: {reduced_model.nq} DOF (Expected: {len(target_joints_names)})"
    )

    return reduced_model, reduced_data


def load_full_pinocchio_model(has_free_flyer=False):
    """
    This function :
    1. Creates a temporary ROS node.
    2. Waits (blocks) until it receives the URDF from Gazebo/Robot State Publisher.
    3. Builds the full Pinocchio model (with or without FreeFlyer).
    4. Returns (model, data).
    """

    # Retrieving the XML via ROS
    print("[ModelLoader] Waiting for robot description (/robot_description)...")

    loader_node = ModelLoaderNode()

    # Loop until the message is received
    while rclpy.ok() and loader_node.urdf_xml is None:
        rclpy.spin_once(loader_node, timeout_sec=0.1)

    if loader_node.urdf_xml is None:
        raise RuntimeError("Unable to retrieve URDF. Is Gazebo running?")

    print("[ModelLoader] URDF received! Building Pinocchio model...")
    full_urdf_string = loader_node.urdf_xml

    # Destroy the temporary node to clean up
    loader_node.destroy_node()

    # Building the Full Model
    if has_free_flyer:
        full_model = pin.buildModelFromXML(full_urdf_string, pin.JointModelFreeFlyer())
        print("[ModelLoader] Building model with FreeFlyer.")
    else:
        full_model = pin.buildModelFromXML(full_urdf_string)
        print("[ModelLoader] Building model (Fixed Base).")

    full_data = full_model.createData()

    # print joint info for debugging
    print("=" * 40)
    print(f"[ModelLoader] Model Dimensions: nq={full_model.nq}, nv={full_model.nv}")
    print(f"[ModelLoader] List of {len(full_model.names)} joints:")
    for i, name in enumerate(full_model.names):
        # On affiche l'ID, le nom, et la taille (nq, nv) du joint
        j_model = full_model.joints[i]
        print(f"  ID {i:02d} | Name: {name:<25} | nq={j_model.nq} | nv={j_model.nv}")
    print("=" * 40)

    return full_model, full_data
