import serial
import time

# --- CONFIGURATION ---
# On Pi 5, the GPIO pins 8/10 usually map to /dev/ttyAMA0
# Standard M8N baud rate is 9600 (sometimes 38400)
SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 9600
# ---------------------

print(f"Trying to connect to GPS on {SERIAL_PORT} at {BAUD_RATE} baud...")

try:
    # Open the serial port
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print("Connected! Waiting for data (this may take a few seconds)...")
    print("-------------------------------------------------------------")

    while True:
        # Read a line from the GPS
        line = ser.readline()
        
        # Decode bytes to string
        try:
            line_str = line.decode('utf-8', errors='replace').strip()
        except Exception:
            line_str = ""

        # Only print lines that look like NMEA data (Start with $)
        if line_str.startswith('$'):
            print(f"RAW DATA: {line_str}")
            
            # Highlight the line that contains the position (GNGGA or GPGGA)
            if "GGA" in line_str:
                print(">>> POSITION DATA FOUND! <<<")

except serial.SerialException as e:
    print(f"\nError: Could not open serial port {SERIAL_PORT}.")
    print("Make sure you disabled the 'Serial Console' in raspi-config.")
    print(f"Details: {e}")

except KeyboardInterrupt:
    print("\nTest stopped by user.")
