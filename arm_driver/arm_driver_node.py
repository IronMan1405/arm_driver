import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import board, busio
from adafruit_pca9685 import PCA9685

class ArmDriver(Node):
    def __init__(self):
        super().__init__("arm_driver")

        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50

        self.min_duty = 0x0800
        self.max_duty = 0x1800

        self.joint_states = self.create_subscription(JointState, '/arm/joint_targets', self.joint_callback, 10)

        self.get_logger().info("Arm driver started")

    def angle_to_duty(self, angle_rad):
        angle_deg = angle_rad * 57.2958
        angle_deg = max(-90, min(90, angle_deg))

        span = self.max_duty = self.min_duty
        duty = self.min_duty + ((angle_deg + 90) / 180) * span

        return int(duty)

    def joint_callback(self, msg):
        for i, angle in enumerate(msg.position):
            ch = 5 - i
            if ch >= 5:
                break

        duty = self.angle_to_duty(angle)
        self.pca.channels[ch].duty_cycle = duty

def main():
	rclpy.init()
	node = ArmDriver()
	rclpy.spin(node)
	rclpy.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
