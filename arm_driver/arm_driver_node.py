import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import board, busio, math
from adafruit_servokit import ServoKit

class ArmDriver(Node):
    def __init__(self):
        super().__init__("arm_driver")

        self.kit = ServoKit(channels = 16)

        self.joint_map = {
            "joint1": 5,
            "joint2": 4,
            "joint3": 3,
            "joint4": 2,
            "joint5": 1,
            "joint6": 0,
        }

        self.joint_limits = {
            "joint_1": (0, 180),
            "joint_2": (0, 180),
            "joint_3": (0, 180),
            "joint_4": (0, 180),
            "joint_5": (0, 180),
            "joint_6": (0, 180),
        }
        
        self.calibration = {
            "joint1": {"channel": 5, "offset": 90, "invert": False},
            "joint2": {"channel": 4, "offset": 90, "invert": False},
            "joint3": {"channel": 3, "offset": 90, "invert": True},
            "joint4": {"channel": 2, "offset": 90, "invert": False},
            "joint5": {"channel": 1, "offset": 90, "invert": False},
            "joint6": {"channel": 0, "offset": 90, "invert": False},
        }

        self.create_subscription(JointState, '/arm/joint_targets', self.joint_callback, 10)
        
        self.actual_state_pub = self.create_publisher(JointState, "/joint_states", 10)

                    
        self.get_logger().info("Arm driver started")

    def joint_callback(self, msg):
        for name, angle_rad in zip(msg.name, msg.position):
            if name not in self.calibration:
                self.get_logger().warn(f"Unknown joint: {name}")
                continue

            cfg = self.calibration[name]
            ch = cfg['channel']

            angle_deg = math.degrees(angle_rad) + cfg['offset']

            if cfg['invert']:
                angle_deg = 180 - angle_deg

            # angle_deg = angle_deg + cfg['offset']

            self.kit.servo[ch].angle = angle_deg

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_map.keys())
        msg.position = [math.radians(self.kit.servo[ch].angle) for ch in range(6)]
        self.actual_state_pub.publish(msg)

def main():
	rclpy.init()
	node = ArmDriver()
	rclpy.spin(node)
	rclpy.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
