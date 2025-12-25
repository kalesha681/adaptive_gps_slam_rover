import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import gpiod
import time

CHIP = "gpiochip4"   # Raspberry Pi 5 GPIO controller
TRIG = 23            # BCM numbering
ECHO = 24

class UltrasonicTestNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_test_node')

        self.publisher = self.create_publisher(Range, '/ultrasonic/range', 10)
        self.timer = self.create_timer(0.3, self.read_distance)

        chip = gpiod.Chip(CHIP)

        self.trig = chip.get_line(TRIG)
        self.echo = chip.get_line(ECHO)

        self.trig.request(
            consumer="ultrasonic_trig",
            type=gpiod.LINE_REQ_DIR_OUT,
            default_vals=[0]
        )

        self.echo.request(
            consumer="ultrasonic_echo",
            type=gpiod.LINE_REQ_DIR_IN
        )

        self.get_logger().info("Ultrasonic test node started")

    def read_distance(self):
        self.trig.set_value(1)
        time.sleep(10e-6)
        self.trig.set_value(0)

        timeout = time.time() + 0.04

        while self.echo.get_value() == 0:
            start = time.time()
            if start > timeout:
                self.get_logger().warn("Echo start timeout")
                return

        while self.echo.get_value() == 1:
            end = time.time()
            if end > timeout:
                self.get_logger().warn("Echo end timeout")
                return

        pulse = end - start
        distance = (pulse * 343.0) / 2.0

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ultrasonic_link"
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26
        msg.min_range = 0.02
        msg.max_range = 4.0
        msg.range = distance

        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = UltrasonicTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
