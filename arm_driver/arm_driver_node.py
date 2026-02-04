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

        self.create_subscription(JointState, '/arm/joint_targets', self.joint_callback, 10)

        self.get_logger().info("Arm driver started")

    def radian_to_servo_deg(self, angle_rad):
        return math.degrees(angle_rad)

    def joint_callback(self, msg):
        # self.get_logger().info(str(list(zip(msg.name, msg.position))))
        for i, angle in enumerate(msg.position):
            ch = 5 - i
            angle_deg = self.radian_to_servo_deg(angle)
            # print(ch, angle+90)
            # self.get_logger().info(f"ch={ch} angle = {angle + 90}")
            if ch < 0:
                break
            
            if ch == 2:
                 angle_deg = 180 - angle_deg

            self.kit.servo[ch].angle = angle_deg + 90

def main():
	rclpy.init()
	node = ArmDriver()
	rclpy.spin(node)
	rclpy.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
