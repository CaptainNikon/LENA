import serial
import csv
import time
from datetime import datetime

# === CONFIGURATION ===
serial_port = 'COM3'       # Update if needed
baud_rate = 115200
csv_filename = 'log_output.csv'

# === CONNECT TO SERIAL ===
try:
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
except serial.SerialException as e:
    print(f"ERROR: Could not open serial port {serial_port}.")
    print(e)
    exit(1)

time.sleep(2)  # Let Arduino reset

print(f"Logging from {serial_port} to {csv_filename}...")

# === OPEN CSV FILE ===
with open(csv_filename, mode='w', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter='\t')

    # Header row with added Epoch timestamp
    writer.writerow(["EpochTT", "Distance_mm", "Temp_C", "AccX_g", "AccY_g", "AccZ_g", "MagX", "MagY", "MagZ"])

    try:
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line:
                values = line.split('\t')  # tab-separated from Arduino
                if len(values) == 8:
                    epoch_tt = time.time()  # e.g. 1714993678.123
                    writer.writerow([epoch_tt] + values)
                    print(f"{epoch_tt:.3f}\t{line}")
                else:
                    print("Malformed line:", line)
    except KeyboardInterrupt:
        print("\nLogging stopped by user.")
    finally:
        ser.close()
