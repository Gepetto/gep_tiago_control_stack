#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

class GTConverter(Node):
    def __init__(self):
        super().__init__('gt_converter')
        self.sub = self.create_subscription(
            Odometry, 
            '/mobile_base_controller/odom', 
            self.cb, 
            10
        )
        
        # publish on fake joint for ros2_control
        self.pub = self.create_publisher(JointState, '/ground_truth_interfaces', 10)

    def cb(self, msg):
        js = JointState()
        js.header = msg.header
        
        # On définit des noms d'interfaces que ton LFC cherchera
        # Attention : adapte ces noms à ce que tu as mis dans ton URDF ros2_control
        js.name = [
            # 'ground_truth_sensor/position.x', 
            # 'ground_truth_sensor/position.y', 
            # 'ground_truth_sensor/position.z',
            # 'ground_truth_sensor/orientation.x', 
            # 'ground_truth_sensor/orientation.y', 
            # 'ground_truth_sensor/orientation.z', 
            # 'ground_truth_sensor/orientation.w'
            "gt_x", "gt_y", "gt_z",         # Position
            "gt_qx", "gt_qy", "gt_qz", "gt_qw" # Orientation (Quaternion)
        ]
        
        # Mapping Position (q)
        js.position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x, 
            msg.pose.pose.orientation.y, 
            msg.pose.pose.orientation.z, 
            msg.pose.pose.orientation.w
        ]
        
        # Mapping Vitesse (v) - On utilise le champ velocity du JointState
        js.velocity = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
            0.0 # Padding pour le quaternion w (pas de vitesse w)
        ]
        
        self.pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    converter = GTConverter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
