import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import time

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        
        # --- CONFIGURATION ---
        self.serial_port = '/dev/ttyAMA0'
        self.baud_rate = 9600
        # ---------------------

        self.publisher_ = self.create_publisher(NavSatFix, '/fix', 10)
        
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to GPS on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to GPS: {e}")
            self.ser = None

        # Create a timer to check for data 10 times per second
        self.timer = self.create_timer(0.1, self.read_gps_data)

    def read_gps_data(self):
        if self.ser is None:
            return

        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
                
                # We only care about the GNGGA or GPGGA sentence (Global Positioning System Fix Data)
                if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                    msg = self.parse_nmea_gga(line)
                    if msg:
                        self.publisher_.publish(msg)
                        
        except Exception as e:
            self.get_logger().warn(f"Error reading GPS: {e}")

    def parse_nmea_gga(self, line):
        """
        Manually parses NMEA string to extract Lat/Lon
        Example: $GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
        """
        parts = line.split(',')
        
        # Check if the sentence is complete (usually has ~15 parts)
        if len(parts) < 10:
            return None

        try:
            # 1. Get Raw Data
            raw_lat = parts[2]
            lat_dir = parts[3]
            raw_lon = parts[4]
            lon_dir = parts[5]
            quality = parts[6]
            altitude = parts[9]

            # If empty (no fix), return empty message or skip
            if not raw_lat or not raw_lon:
                # self.get_logger().warn("No Satellite Fix yet...")
                return None

            # 2. Convert to Decimal Degrees
            # NMEA format is DDMM.MMMM (Degrees + Minutes)
            # We need pure Degrees: DD + (MM.MMMM / 60)
            
            # Latitude
            lat_deg = float(raw_lat[:2])
            lat_min = float(raw_lat[2:])
            latitude = lat_deg + (lat_min / 60)
            if lat_dir == 'S':
                latitude = -latitude

            # Longitude (can be 3 digits for degrees)
            lon_deg = float(raw_lon[:3])
            lon_min = float(raw_lon[3:])
            longitude = lon_deg + (lon_min / 60)
            if lon_dir == 'W':
                longitude = -longitude

            # 3. Create ROS Message
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps_link"
            
            msg.latitude = latitude
            msg.longitude = longitude
            
            # Status: 0 = No Fix, 1 = Fix
            if quality != '0':
                msg.status.status = 0 # STATUS_FIX
            else:
                msg.status.status = -1 # STATUS_NO_FIX
            
            # Altitude
            if altitude:
                msg.altitude = float(altitude)

            return msg

        except ValueError:
            return None

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
