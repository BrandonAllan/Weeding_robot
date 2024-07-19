#!/usr/bin/env python3

import serial
import time

def main():
    serial_port = '/dev/ttyACM0'  # Replace with your actual serial port
    baud_rate = 115200
    ser = None  # Initialize ser outside try block

    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        print(f"Opened serial port {serial_port}")
        time.sleep(1)

        # Example of sending a character (byte) to Arduino
        ser.write(b'X')
        print(f"Sent: X")

        # Read incoming data
        while True:
            if ser.in_waiting > 0:
                data = ser.readline().decode().strip()
                print(f"Received: {data}")

            # Optional: Uncomment the following line to send additional characters or commands
            # ser.write(b'Y')

            time.sleep(0.1)  # Add a small delay to prevent excessive polling

    except serial.SerialException as e:
        print(f"Failed to open serial port {serial_port}: {e}")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if ser and ser.is_open:
            # Do not close the serial port here if you want to continue reading
            print(f"Closing serial port {serial_port}")
            ser.close()

if __name__ == '__main__':
    main()
