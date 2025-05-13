# Code to run the data acquisation of the ground station
# All modules needed can be installed using: pip install pyserial matplotlib numpy os

import serial
import threading
import tkinter as tk
from tkinter import scrolledtext
import time
import csv
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
from collections import deque
import os

# Config of the Arduino port
serial_port = 'COM3'
baud_rate = 115200

# Initialization of buffers to save time while saving
raw_buffer = []
calib_buffer = []
FLUSH_THRESHOLD = 10 # Save every n lines
raw_filename = 'log_raw_output.csv'
calib_filename = 'log_calibrated_output.csv'

#buffer for plotting 
plot_length = 100
data_buffer = deque(maxlen=plot_length)

# Calibration matrix accelerometer
Clbmtrx_acc = np.array([
    [ 0.9816, -0.0377, -0.0231, -0.0151],
    [-0.0008,  1.0021,  0.0005, -0.0559],
    [ 0.0363,  0.0023,  1.0283, -0.0195]
])

# GLOBAL STATE
ser = None
running = True

def calibrate(values):
    global Clbmtrx_acc
    try:
        distance = int(values[0])
        temp_c = float(values[1]) * 0.01

        acc_x = float(values[2]) * 0.015748
        acc_y = float(values[3]) * 0.015748
        acc_z = float(values[4]) * 0.015748
        raw_acc = np.array([acc_x, acc_y, acc_z, 1.0])
        calibrated_acc = Clbmtrx_acc @ raw_acc 
        acc_x_c, acc_y_c, acc_z_c = calibrated_acc
        
        mag_x = int(values[5])
        mag_y = int(values[6])
        mag_z = int(values[7])
        ground_t = float(values[8])
        ground_h = 0.858*float(values[9])+0.005
    except Exception as e:
        raise ValueError(f"Calibration failed: {e}")
    return [distance, temp_c, acc_x_c, acc_y_c, acc_z_c, mag_x, mag_y, mag_z, ground_t, ground_h]

# === SERIAL READER THREAD ===
def serial_reader(log_widget):
    global ser, running, raw_buffer, calib_buffer, raw_filename, calib_filename, data_buffer

    with open(raw_filename, mode='w', newline='') as raw_file, \
         open(calib_filename, mode='w', newline='') as calib_file:

        raw_writer = csv.writer(raw_file, delimiter='\t')
        calib_writer = csv.writer(calib_file, delimiter='\t')

        # Write headers
        raw_writer.writerow(["EpochTT", "Raw_Distance", "Raw_Temp", "Raw_AccX", "Raw_AccY", "Raw_AccZ",
                             "Raw_MagX", "Raw_MagY", "Raw_MagZ", "Raw_Ground_T", "Raw_Ground_H"])
        calib_writer.writerow(["EpochTT", "Distance_mm", "Temp_C", "AccX_g", "AccY_g", "AccZ_g",
                               "MagX", "MagY", "MagZ", "Ground_T", "Ground_H"])

        while running:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    values = line.split('\t')
                    if len(values) == 11:
                        epoch_tt = time.time()

                        # Store raw values
                        raw_row = [epoch_tt] + values
                        raw_buffer.append(raw_row)

                        try:
                            # Try calibration
                            calibrated = calibrate(values)
                            
                            # Basic validation (optional): check if length and type are as expected
                            if not isinstance(calibrated, list) or len(calibrated) < 8:
                                raise ValueError("Calibration returned unexpected result")

                        except Exception as e:
                            print(f"Calibration failed at {epoch_tt:.3f}, using raw values. Error: {e}")
                            # Use raw values, converting them to float
                            try:
                                calibrated = list(map(float, values))
                            except ValueError:
                                print(f"Invalid raw data format at {epoch_tt:.3f}, skipping this line.")
                                continue  # skip this line entirely if raw values are invalid

                            # Proceed with data handling
                            calib_row = [epoch_tt] + calibrated
                            calib_buffer.append(calib_row)
                            display_line = f"{epoch_tt:.3f}\t{'\t'.join(map(str, calibrated))}\n"

                            # Buffer for plotting
                            temp_c = calibrated[1]
                            acc_x, acc_y, acc_z = calibrated[2:5]
                            mag_x, mag_y, mag_z = calibrated[5:8]

                            data_buffer.append({
                                "time": epoch_tt,
                                "acc_x": acc_x,
                                "acc_y": acc_y,
                                "acc_z": acc_z,
                                "temp": temp_c,
                                "mag_x": mag_x,
                                "mag_y": mag_y,
                                "mag_z": mag_z,
                                })

                        except ValueError as ve:
                            display_line = f"[Calibration Error] {ve} | Line: {line}\n"
                    else:
                        display_line = f"[Malformed] {line}\n"

                    log_widget.insert(tk.END, display_line)
                    log_widget.see(tk.END)

                    # Write buffer to file after threshold
                    if len(raw_buffer) >= FLUSH_THRESHOLD:
                        raw_writer.writerows(raw_buffer)
                        calib_writer.writerows(calib_buffer)
                        raw_buffer.clear()
                        calib_buffer.clear()

            except Exception as e:
                log_widget.insert(tk.END, f"[Error] {e}\n")
                log_widget.see(tk.END)

        # Final saving on exit
        if raw_buffer:
            raw_writer.writerows(raw_buffer)
        if calib_buffer:
            calib_writer.writerows(calib_buffer)


# === SEND COMMAND ===
def send_command(entry_widget, log_widget):
    cmd = entry_widget.get().strip()
    if cmd:
        try:
            ser.write((cmd + '\n').encode('utf-8'))
            log_widget.insert(tk.END, f"[Sent] {cmd}\n")
            log_widget.see(tk.END)
            entry_widget.delete(0, tk.END)
        except Exception as e:
            log_widget.insert(tk.END, f"[Send Error] {e}\n")
            log_widget.see(tk.END)

# Plotting
def create_plots(root):
    global data_buffer

    # Setup matplotlib figure with 3 subplots
    fig = Figure(figsize=(8, 6), dpi=100)

    ax1 = fig.add_subplot(311)  # Temperature
    ax2 = fig.add_subplot(312)  # Acceleration
    ax3 = fig.add_subplot(313)  # Magnetic field

    # Initialize empty lines
    temp_line, = ax1.plot([], [], label="Temp (°C)", color='orange')
    acc_x_line, = ax2.plot([], [], label="Acc X")
    acc_y_line, = ax2.plot([], [], label="Acc Y")
    acc_z_line, = ax2.plot([], [], label="Acc Z")
    mag_x_line, = ax3.plot([], [], label="Mag X")
    mag_y_line, = ax3.plot([], [], label="Mag Y")
    mag_z_line, = ax3.plot([], [], label="Mag Z")

    # Configure plots
    for ax in [ax1, ax2, ax3]:
        ax.grid(True)
        ax.legend(loc='upper right')

    ax1.set_ylabel("Temp (°C)")
    ax2.set_ylabel("Acc (g)")
    ax3.set_ylabel("Mag")

    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.get_tk_widget().pack(padx=10, pady=10)

    def update_plot():
        if not data_buffer:
            root.after(200, update_plot)
            return

        times = [d["time"] for d in data_buffer]
        temps = [d["temp"] for d in data_buffer]

        acc_x = [d["acc_x"] for d in data_buffer]
        acc_y = [d["acc_y"] for d in data_buffer]
        acc_z = [d["acc_z"] for d in data_buffer]

        mag_x = [d["mag_x"] for d in data_buffer]
        mag_y = [d["mag_y"] for d in data_buffer]
        mag_z = [d["mag_z"] for d in data_buffer]

        # Update data for all lines
        temp_line.set_data(times, temps)

        acc_x_line.set_data(times, acc_x)
        acc_y_line.set_data(times, acc_y)
        acc_z_line.set_data(times, acc_z)

        mag_x_line.set_data(times, mag_x)
        mag_y_line.set_data(times, mag_y)
        mag_z_line.set_data(times, mag_z)

        # Rescale axes
        for ax in [ax1, ax2, ax3]:
            ax.relim()
            ax.autoscale_view()

        canvas.draw()
        root.after(50, update_plot)

    update_plot()


# GUI
def start_gui():
    global ser, data_buffer, plot_length
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        #log_widget.insert(tk.END, f"[Info] Connected to {serial_port} at {baud_rate} baud\n")
        time.sleep(2)  # Allow Arduino to reset
    except serial.SerialException as e:
        print(f"Could not open serial port {serial_port}: {e}")
        return

    root = tk.Tk()
    root.title("CanSat Logger + Command Sender")

    log_widget = scrolledtext.ScrolledText(root, width=100, height=30)
    log_widget.pack(padx=10, pady=10)

    command_frame = tk.Frame(root)
    command_frame.pack(pady=5)

    entry = tk.Entry(command_frame, width=50)
    entry.pack(side=tk.LEFT, padx=5)

    send_button = tk.Button(command_frame, text="Send", command=lambda: send_command(entry, log_widget))
    send_button.pack(side=tk.LEFT)

    # Predefined buttons
    for cmd in ["p", "s", "r"]:
        btn = tk.Button(command_frame, text=cmd.capitalize(), command=lambda c=cmd: entry.insert(0, c))
        btn.pack(side=tk.LEFT, padx=2)

    # Launch Serial thread
    threading.Thread(target=serial_reader,args=(log_widget,),daemon=True).start()

    create_plots(root)

    # Handle close
    def on_close():
        global running
        running = False
        if ser:
            ser.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


# === RUN GUI ===
if __name__ == "__main__":
    try:
        start_gui()
    except KeyboardInterrupt:
        print('Interrupted')
        try:
            with open(raw_filename, mode='w', newline='') as raw_file, \
                open(calib_filename, mode='w', newline='') as calib_file:
                raw_writer = csv.writer(raw_file, delimiter='\t')
                calib_writer = csv.writer(calib_file, delimiter='\t')
                if raw_buffer:
                    raw_writer.writerows(raw_buffer)
                if calib_buffer:
                    calib_writer.writerows(calib_buffer)
        except SystemExit:
            os._exit(130)
