#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from math import pi

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
        default_values = [-0.0, 1.027, 2.761, 0.081] # nav_fowward - tucked and looking slightly down to detect ground obstacles
        tenhut_values = [-0.0, 0.059, 0.0, 1.5] # stand at attention, eyes forward
        lookup_values = [-0.0, 0.959, 0.925, 0.989] # look up toward human in front of bot for face detection
        lookout_values = [-0.0, 0.959, 0.925, 1.5] # same as look up but camera looking forward 
        reach_values = [-0.0, -1.0, 0.65, 0.081] # reach forward but with a bit of reserve to reach more
        self.presets = {
            'bumper': default_values,
            'tenhut': tenhut_values,
            'lookup': lookup_values,
            'lookout': lookout_values,
            'reach': reach_values,
        }
        self.bearing_presets = {
            "back-left": -2.094395102,   # -120°
            "full-left": -1.570796327,   # -90°
            "left": -0.7853981634,       # -45° same as military eyes left
            "leftish": -0.436332313,     # -25°
            "forward": 0,                # 0°
            "rightish": 0.436332313,     # 25°
            "right": 0.7853981634,       # 45° same as military eyes right
            "full-right": 1.570796327,   # 90°
            "back-right": 2.094395102,   # 120°
            "back": 3.141592654,         # 180°
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
        preset_spec = msg.data.strip()
        self.get_logger().info(f"Received preset command: '{preset_spec}'")

        # Split preset and optional bearing/yaw
        if '@' in preset_spec:
            preset_name, bearing_spec = preset_spec.split('@', 1)
            preset_name = preset_name.strip()
            bearing_spec = bearing_spec.strip()
        else:
            preset_name = preset_spec
            bearing_spec = None

        if preset_name not in self.presets:
            self.get_logger().warn(f"Preset '{preset_name}' is not defined.")
            return

        joint_positions = self.presets[preset_name].copy()  # copy to avoid mutating original

        # Adjust pan angle if bearing_spec provided
        if bearing_spec:
            try:
                yaw = float(bearing_spec)
                # Wrap to [-pi, pi]
                yaw = (yaw + pi) % (2 * pi) - pi
                joint_positions[0] = yaw
                self.get_logger().info(f"Applied direct yaw {yaw:.3f} rad to pan joint.")
            except ValueError:
                if bearing_spec in self.bearing_presets:
                    yaw = self.bearing_presets[bearing_spec]
                    # Wrap to [-pi, pi]
                    yaw = (yaw + pi) % (2 * pi) - pi
                    joint_positions[0] = yaw
                    self.get_logger().info(f"Applied bearing preset '{bearing_spec}' with yaw {yaw:.3f} rad.")
                else:
                    self.get_logger().warn(f"Bearing spec '{bearing_spec}' unrecognized, leaving pan joint unchanged.")

        # Publish the joint state
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.get_joint_names()
        joint_state.position = joint_positions

        self.joint_state_pub.publish(joint_state)
        self.get_logger().info(f"Published joint states for preset '{preset_name}' with final yaw {joint_positions[0]:.3f} rad.")

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
