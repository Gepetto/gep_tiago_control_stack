#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import xml.etree.ElementTree as ET

JOINTS_TO_PATCH = [
    "wheel_left_joint",
    "wheel_right_joint",
    "torso_lift_joint",
    "head_1_joint",
    "head_2_joint",
    "gripper_left_finger_joint",
    "gripper_right_finger_joint",
]


class URDFPatcher(Node):
    def __init__(self):
        super().__init__("urdf_patcher")
        self.urdf_xml = None

        # On s'abonne avec un profil QoS durable pour attraper le message
        qos_profile = rclpy.qos.QoSProfile(
            depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.sub = self.create_subscription(
            String, "/robot_description", self.callback, qos_profile
        )
        self.get_logger().info("En attente de /robot_description...")

    def callback(self, msg):
        self.urdf_xml = msg.data
        self.get_logger().info("URDF reçu !")


def add_effort_interface(xml_string, output_path):
    # Parsing du XML
    try:
        root = ET.fromstring(xml_string)
    except ET.ParseError as e:
        print(f"Erreur de parsing XML : {e}")
        return False

    print("--- Patching URDF ---")

    # On cherche la section <ros2_control>
    # Note: L'espace de noms (xmlns) peut parfois casser la recherche,
    # on fait une recherche agnostique
    ros2_control_tags = root.findall("ros2_control")

    patches_count = 0

    for r2c in ros2_control_tags:
        for joint in r2c.findall("joint"):
            joint_name = joint.get("name")

            if joint_name in JOINTS_TO_PATCH:
                # Vérifie si l'interface effort existe déjà
                existing_interfaces = [
                    ci.get("name") for ci in joint.findall("command_interface")
                ]

                if "effort" not in existing_interfaces:
                    # Création de la balise <command_interface name="effort" />
                    new_ci = ET.Element("command_interface")
                    new_ci.set("name", "effort")

                    # On l'ajoute (souvent on veut voir un min/max, mais pour effort simple pas obligé)
                    joint.append(new_ci)
                    print(f"[V] Ajout interface EFFORT sur : {joint_name}")
                    patches_count += 1
                else:
                    print(f"[x] Déjà présent sur : {joint_name}")

    if patches_count == 0:
        print("Aucun patch appliqué (balises introuvables ou déjà présentes ?)")
    else:
        print(f"Succès : {patches_count} modifications appliquées.")

    # Sauvegarde
    tree = ET.ElementTree(root)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return True


def main():
    rclpy.init()
    node = URDFPatcher()

    # Attendre le message
    while rclpy.ok() and node.urdf_xml is None:
        rclpy.spin_once(node, timeout_sec=1.0)

    if node.urdf_xml:
        output_file = "/tmp/tiago_effort.urdf"
        success = add_effort_interface(node.urdf_xml, output_file)

        if success:
            print(f"\n[OK] Nouveau fichier généré : {output_file}")
            print("\nPOUR LANCER LA SIMULATION AVEC CE ROBOT :")
            print("-" * 50)
            print(
                f"ros2 run gazebo_ros spawn_entity.py -entity tiago_mpc -file {output_file} -x 0.0 -y 0.0 -z 0.0"
            )
            print("-" * 50)

            # Optionnel : Republier sur robot_description pour que le LFC le voie ?
            # C'est risqué car robot_state_publisher republie souvent par dessus.
            # Mieux vaut lancer le noeud MPC en lui donnant ce fichier XML directement.
    else:
        print("Timeout : Impossible de récupérer l'URDF.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
