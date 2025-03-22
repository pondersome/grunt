#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class ArmPresetPublisher(Node):
    def __init__(self):
        super().__init__('arm_preset_publisher')
        # Publisher for joint states
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        # Subscriber for preset commands
        self.create_subscription(String, 'arm_preset', self.preset_callback, 10)

        # Get a prefix parameter for joint name aliasing (default is empty)
        self.declare_parameter('prefix', '')
        self.prefix = self.get_parameter('prefix').value

        # Define the joint name suffixes
        # The full joint names will be created as: {prefix}_{suffix} if a prefix is provided.
        self.joint_suffixes = {
            'pan': 'base_link_to_link1',
            'shoulder': 'link1_to_link2',
            'elbow': 'link2_to_link3',
            'wrist': 'link3_to_gripper_link'
        }

        # Define preset configurations.
        # Joint order: pan, shoulder, elbow, wrist.
        # Using nav_forward values for all presets initially.
        default_values = [-0.085, 1.027, 2.761, 0.081] #nav_fowward - tucked and looking slightly down to detect ground obstacles
        tenhut_values = [-0.085, 0.059, 0.0, 1.5] #stand at attention, eyes forward
        lookup_values = [-0.085, 0.959, 0.925, 0.989] #look up toward human

        self.presets = {
            'nav_forward': default_values,
            'tenhut': tenhut_values,
            'lookup': lookup_values,
        }

        self.get_logger().info("ArmPresetPublisher initialized and waiting for preset commands...")

    def get_joint_names(self):
        names = []
        for joint in ['pan', 'shoulder', 'elbow', 'wrist']:
            suffix = self.joint_suffixes[joint]
            if self.prefix:
                names.append(f"{self.prefix}{suffix}")
            else:
                names.append(suffix)
        return names

    def preset_callback(self, msg: String):
        preset_name = msg.data.strip()
        self.get_logger().info(f"Received preset command: '{preset_name}'")
        if preset_name not in self.presets:
            self.get_logger().warn(f"Preset '{preset_name}' is not defined.")
            return

        joint_positions = self.presets[preset_name]

        # Create and populate the JointState message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.get_joint_names()
        joint_state.position = joint_positions

        # Publish the joint state message once
        self.joint_state_pub.publish(joint_state)
        self.get_logger().info(f"Published joint states for preset '{preset_name}'.")

def main(args=None):
    rclpy.init(args=args)
    node = ArmPresetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
