"""

Description:
    utility module to load and build Pinocchio models from URDF descriptions
    published on ROS 2 topics. 
    Needed by MPC controllers as urdf is not directly accessible via classic files.
    Provides functions to create full or reduced kinematic
    models for robotics simulation and control applications.

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
import pinocchio as pin
import numpy as np

class ModelLoaderNode(Node):
    """Create a node to get model from the URDF."""
    def __init__(self):
        super().__init__('temp_model_loader')
        self.urdf_xml = None

        # QoS Latched
        qos = QoSProfile(depth=1, 
                         durability=DurabilityPolicy.TRANSIENT_LOCAL, 
                         reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(String, 
                                 '/robot_description', 
                                 self.callback, 
                                 qos)

    def callback(self, msg):
        self.urdf_xml = msg.data

def load_reduced_pinocchio_model(target_joints_names, has_free_flyer=False):
    """
    This function :
    1. Creates a temporary ROS node.
    2. Waits (blocks) until it receives the URDF from Gazebo/Robot State Publisher.
    3. Builds the full Pinocchio model (with Planar or Fixed base).
    4. Reduces the model to keep only 'target_joints_names'.
    5. Returns (model, data, visual_model, visual_data).
    """

    print("[ModelLoader] Waiting for robot description (/robot_description)...")
    loader_node = ModelLoaderNode()

    while rclpy.ok() and loader_node.urdf_xml is None:
        rclpy.spin_once(loader_node, timeout_sec=0.1)

    if loader_node.urdf_xml is None:
        raise RuntimeError("Unable to retrieve URDF. Is Gazebo running?")

    print("[ModelLoader] URDF received! Building Pinocchio model...")
    full_urdf_string = loader_node.urdf_xml
    loader_node.destroy_node()

    # Build full model
    if has_free_flyer:
        full_model = pin.buildModelFromXML(full_urdf_string, pin.JointModelPlanar())
        print("[ModelLoader] 🌐 Building model with Planar base (2D mobile robot).")
    else:
        full_model = pin.buildModelFromXML(full_urdf_string)
        print("[ModelLoader] 🔧 Building model (Fixed Base).")

    # Build geometry models from URDF string
    try:
        full_visual_model = pin.buildGeomFromUrdfString(
            full_model,
            full_urdf_string,
            pin.GeometryType.VISUAL,
            package_dirs=[]
        )
        print(f"[ModelLoader] 🎨 Visual model loaded: {full_visual_model.ngeoms} geometries")
    except Exception as e:
        print(f"[ModelLoader] ⚠️ Could not load visual model: {e}")
        full_visual_model = pin.GeometryModel()

    # Identify joints to lock
    locked_joint_ids = []
    kept_joints_info = []

    for joint_id in range(1, full_model.njoints):
        joint_name = full_model.names[joint_id]

        if has_free_flyer and joint_name == "root_joint":
            kept_joints_info.append(f"  {joint_name} (Planar: nq=4, nv=3)")
            continue

        if joint_name in target_joints_names:
            joint_model = full_model.joints[joint_id]
            nq = joint_model.nq
            nv = joint_model.nv
            joint_type = joint_model.shortname()
            kept_joints_info.append(f"  {joint_name} ({joint_type}: nq={nq}, nv={nv})")
        else:
            locked_joint_ids.append(joint_id)

    print(f"[ModelLoader] Keeping {len(kept_joints_info)} joints:")
    for info in kept_joints_info:
        print(info)

    # Build reduced model with geometry
    reduced_model, reduced_geom_models = pin.buildReducedModel(
        full_model,
        list_of_geom_models=[full_visual_model],
        list_of_joints_to_lock=locked_joint_ids,
        reference_configuration=pin.neutral(full_model)
    )
    
    reduced_data = reduced_model.createData()
    
    # Extract reduced visual model
    reduced_visual_model = reduced_geom_models[0] if reduced_geom_models else pin.GeometryModel()
    reduced_visual_data = reduced_visual_model.createData()

    # Summary
    print(f"\n[ModelLoader] ✅ Reduced model created:")
    print(f"  📊 Dimensions: nq={reduced_model.nq}, nv={reduced_model.nv}")
    print(f"  🎯 Target joints: {len(target_joints_names)}")
    print(f"  🔒 Locked joints: {len(locked_joint_ids)}")
    print(f"  🎨 Visual geometries: {reduced_visual_model.ngeoms}")

    if has_free_flyer:
        joints_nq = reduced_model.nq - 4
        joints_nv = reduced_model.nv - 3
        print(f"  🤖 Planar base: nq=4, nv=3")
        print(f"  🦾 joints: nq={joints_nq}, nv={joints_nv}")

    return reduced_model, reduced_data, reduced_visual_model, reduced_visual_data


def load_full_pinocchio_model(has_free_flyer=False):
    """
    This function :
    1. Creates a temporary ROS node.
    2. Waits (blocks) until it receives the URDF from Gazebo/Robot State Publisher.
    3. Builds the full Pinocchio model (with Planar or Fixed base).
    4. Returns (model, data).
    
    Args:
        has_free_flyer: If True, adds Planar base (2D mobile robot)
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
        full_model = pin.buildModelFromXML(full_urdf_string, pin.JointModelPlanar())
        print("[ModelLoader] 🌐 Building model with Planar base (2D mobile robot).")
    else:
        full_model = pin.buildModelFromXML(full_urdf_string)
        print("[ModelLoader] 🔧 Building model (Fixed Base).")

    full_data = full_model.createData()

    # print joint info for debugging
    print("="*40)
    print(f"[ModelLoader] Model Dimensions: nq={full_model.nq}, nv={full_model.nv}")
    print(f"[ModelLoader] List of {len(full_model.names)} joints:")
    for i, name in enumerate(full_model.names):
        j_model = full_model.joints[i]
        joint_type = j_model.shortname()
        print(f"  ID {i:02d} | Name: {name:<25} | Type: {joint_type:<10} | nq={j_model.nq} | nv={j_model.nv}")
    print("="*40)

    return full_model, full_data
