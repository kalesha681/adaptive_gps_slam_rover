import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist  # Import Twist for velocity commands
from rclpy.qos import qos_profile_sensor_data

class ReactiveAvoidance(Node):
    def __init__(self):
        super().__init__('avoidance_logic')

        # Publisher for Robot Velocity (The Command Signal)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for Lidar
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

        self.detection_range = 0.5  # Increased slightly to give more reaction time
        self.get_logger().info('Avoidance Logic Node Started. Publishing to /cmd_vel')

    def scan_callback(self, msg):
        # 1. Find the closest object
        closest_distance = float('inf')
        closest_index = -1

        for i, range_val in enumerate(msg.ranges):
            if 0.05 < range_val < float('inf'):
                if range_val < closest_distance:
                    closest_distance = range_val
                    closest_index = i

        # Create the command message
        cmd = Twist()

        # 2. Logic Decision
        if closest_distance < self.detection_range:
            
            # Calculate angle
            angle_rad = msg.angle_min + (closest_index * msg.angle_increment)
            angle_deg = math.degrees(angle_rad)
            if angle_deg < 0:
                angle_deg += 360

            # Determine Direction and set Velocity
            direction = "UNKNOWN"

            # Case 1: Obstacle FRONT (315 to 45 deg) -> Move BACK
            if (angle_deg >= 315 or angle_deg < 45):
                direction = "FRONT"
                cmd.linear.x = -0.2  # Move Backward
                cmd.angular.z = 0.0

            # Case 2: Obstacle LEFT (45 to 135 deg) -> Move RIGHT (Forward + Turn Right)
            elif (angle_deg >= 45 and angle_deg < 135):
                direction = "LEFT"
                cmd.linear.x = 0.15   # Move Forward slowly
                cmd.angular.z = -0.5  # Turn Right (Clockwise)

            # Case 3: Obstacle BACK (135 to 225 deg) -> Move FRONT
            elif (angle_deg >= 135 and angle_deg < 225):
                direction = "BACK"
                cmd.linear.x = 0.2    # Move Forward
                cmd.angular.z = 0.0

            # Case 4: Obstacle RIGHT (225 to 315 deg) -> Move LEFT (Forward + Turn Left)
            elif (angle_deg >= 225 and angle_deg < 315):
                direction = "RIGHT"
                cmd.linear.x = 0.15   # Move Forward slowly
                cmd.angular.z = 0.5   # Turn Left (Counter-Clockwise)

            self.get_logger().warning(f'Obstacle {direction} ({closest_distance:.2f}m). Taking Action.')

        else:
            # Case 5: Path Clear -> REST
            self.get_logger().info(f'Path Clear ({closest_distance:.2f}m). Resting.')
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # 3. Publish the command
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
