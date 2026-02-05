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
            "joint_1": 5,
            "joint_2": 4,
            "joint_3": 3,
            "joint_4": 2,
            "joint_5": 1,
            "joint_6": 0,
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
            "joint_1": {"channel": 5, "offset": 90, "invert": False},
            "joint_2": {"channel": 4, "offset": 90, "invert": False},
            "joint_3": {"channel": 3, "offset": 90, "invert": True},
            "joint_4": {"channel": 2, "offset": 90, "invert": False},
            "joint_5": {"channel": 1, "offset": 90, "invert": False},
            "joint_6": {"channel": 0, "offset": 90, "invert": False},
        }

        self.create_subscription(JointState, '/arm/joint_targets', self.joint_callback, 10)

        self.get_logger().info("Arm driver started")

    def joint_callback(self, msg):
    #     # self.get_logger().info(str(list(zip(msg.name, msg.position))))
    #     for i, angle in enumerate(msg.position):
    #         ch = 5 - i
    #         angle_deg = math.degrees(angle) + 90
    #         # print(ch, angle+90)
    #         self.get_logger().info(f"ch={ch} angle = {angle + 90}")
    #         if ch < 0:
    #             break
            
    #         if ch == 3:
    #              angle_deg = 180 - angle_deg

    #         self.kit.servo[ch].angle = angle_deg

        for name, angle_rad in zip(msg.name, msg.position):
            if name not in self.calibration:
                self.get_logger().warn(f"Unknown joint: {name}")
                continue

            cfg = self.calibration[name]
            ch = cfg['channel']

            angle_deg = math.degrees(angle_rad)

            if cfg['invert']:
                angle_deg = 180 - angle_deg

            angle_deg = angle_deg + cfg['offset']

            self.kit.servo[ch].angle = angle_deg

def main():
	rclpy.init()
	node = ArmDriver()
	rclpy.spin(node)
	rclpy.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
