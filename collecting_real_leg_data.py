import serial
import numpy as np
import pandas as pd
import time
from math import sin, cos, tan, radians, sqrt

# Set the serial port and baud rate (match with the Arduino code)
serial_port = 'COM19'
baud_rate = 115200
output_file = 'real_leg_rpy.csv'

def angle_2_length(a: int) -> float:
    '''
    Convert the angle to the length of the string displaced

    args:
        a: int: angle in degrees

    returns:
        float: length of the string displaced
    '''
    if a < 90:
        return sqrt(1525 + 1500*sin(radians(a))) - 39
    else:
        return 16 + (radians(a) - radians(90))*3 

def main():
    # Initialize serial connection
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=10)
        print(f"Connected to {serial_port} at {baud_rate} baud")
    except serial.SerialException:
        print(f"Failed to connect to {serial_port}")
        exit()

    # Initialize an empty DataFrame
    data = pd.DataFrame(columns=["theta1", "theta2", "theta3", "L1", "L2", "L3", "roll", "pitch", "yaw"])
    counter = 0
    try:
        while True:
            # Read a line from the serial buffer
            line = ser.readline().decode('utf-8').strip()
            if line:
                if line == "D":
                    print("Data collection complete.")
                    data.to_csv(output_file, index=False)
                    print(f"Data saved to {output_file}")
                    break
                else:
                    print(f"Received: {line}")
                    print(f"Data points collected: {counter}")
                    counter += 1
                    values = line.split(",")  # Split data by comma
                    if values[0] == "data":
                        # Append the data to the DataFrame
                        a1, a2, a3 = values[1:4]
                        l1, l2, l3 = angle_2_length(int(a1)), angle_2_length(int(a2)), angle_2_length(int(a3))
                        y, p, r = values[4:]
                        data = pd.concat([data, pd.DataFrame([[a1, a2, a3, l1, l2, l3, r, p, y]], columns=data.columns)])

    except KeyboardInterrupt:
        print("\nStopped by user.")
        # Save the DataFrame to a CSV file when interrupted
        data.to_csv(output_file, index=False)
        print(f"Data saved to {output_file}")

    finally:
        ser.close()
        print("Serial connection closed.")

if __name__ == "__main__":
    main()
