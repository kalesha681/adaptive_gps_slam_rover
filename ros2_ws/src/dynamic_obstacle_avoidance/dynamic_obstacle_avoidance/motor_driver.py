import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# --- PI 5 SPECIFIC IMPORTS ---
from gpiozero import Motor, Device
from gpiozero.pins.lgpio import LGPIOFactory

# FORCE PI 5 HARDWARE MODE
try:
    Device.pin_factory = LGPIOFactory(chip=4)
except Exception as e:
    print(f"Warning: Could not force Chip 4 factory. {e}")

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        
        try:
            # --- SINGLE BTS7960 SETUP ---
            # We treat the entire robot as "one big motor"
            # RPWM -> GPIO 17 (Forward speed)
            # LPWM -> GPIO 27 (Backward speed)
            self.main_driver = Motor(forward=17, backward=27)
            
            self.get_logger().info("Single BTS7960 Initialized (Forward/Backward Only)")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to GPIO: {e}")

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # We ONLY look at linear.x (Forward/Back)
        # We IGNORE angular.z (Turning) because hardware cannot do it
        speed = msg.linear.x

        # Limit speed to safe range -1.0 to 1.0
        speed = max(min(speed, 1.0), -1.0)

        # Apply speed to the single driver
        if hasattr(self, 'main_driver'):
            self.main_driver.value = speed

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
