import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
import math

class MPU9250Node(Node):
    def __init__(self):
        super().__init__('mpu9250_node')

        # --- Configuration ---
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('frame_id', 'imu_link')
        
        bus_number = self.get_parameter('i2c_bus').value
        self.frame_id = self.get_parameter('frame_id').value

        # Initialize MPU9250 with Low Pass Filter (DLPF) enabled
        # This is critical to filter out motor vibrations from your robot
        try:
            self.mpu = MPU9250(
                address_ak=AK8963_ADDRESS,
                address_mpu_master=MPU9050_ADDRESS_68,
                bus=bus_number,
                gfs=GFS_250,   # Gyro Full Scale 250dps (Higher precision)
                afs=AFS_2G,    # Accel Full Scale 2G (Higher precision)
                mfs=AK8963_BIT_16, 
                mode=AK8963_MODE_C100HZ
            )
            self.mpu.configure()
            self.get_logger().info("MPU-9250 Initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MPU-9250: {e}")
            return

        # Publishers
        # We publish 'data_raw' so a fusion filter (Madgwick/EKF) can process it later
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        
        # 100Hz Polling Rate
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        try:
            # Read all axes
            accel = self.mpu.readAccelerometerMaster()
            gyro = self.mpu.readGyroscopeMaster()
            mag = self.mpu.readMagnetometerMaster()
            
            timestamp = self.get_clock().now().to_msg()

            # --- Construct IMU Message ---
            imu_msg = Imu()
            imu_msg.header.stamp = timestamp
            imu_msg.header.frame_id = self.frame_id

            # MPU9250-jmdev returns Gs -> Convert to m/s^2
            imu_msg.linear_acceleration.x = accel[0] * 9.80665
            imu_msg.linear_acceleration.y = accel[1] * 9.80665
            imu_msg.linear_acceleration.z = accel[2] * 9.80665

            # Returns deg/s -> Convert to rad/s
            imu_msg.angular_velocity.x = math.radians(gyro[0])
            imu_msg.angular_velocity.y = math.radians(gyro[1])
            imu_msg.angular_velocity.z = math.radians(gyro[2])
            
            # Identity matrix for covariance (unknown variance)
            imu_msg.orientation_covariance[0] = -1.0 

            self.imu_pub.publish(imu_msg)

            # --- Construct Magnetometer Message ---
            mag_msg = MagneticField()
            mag_msg.header.stamp = timestamp
            mag_msg.header.frame_id = self.frame_id

            # Returns uT -> Convert to Tesla
            mag_msg.magnetic_field.x = mag[0] * 1e-6
            mag_msg.magnetic_field.y = mag[1] * 1e-6
            mag_msg.magnetic_field.z = mag[2] * 1e-6

            self.mag_pub.publish(mag_msg)

        except Exception as e:
            self.get_logger().warning(f"Sensor read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MPU9250Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
