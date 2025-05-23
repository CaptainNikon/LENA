# Code to run the data acquisation of the ground station
# All modules needed can be installed using: pip install pyserial matplotlib numpy os

import serial
import threading
import tkinter as tk
from tkinter import messagebox
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

# Initialization of data saving buffers
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

predefined_commands = [
    ("Burst Mode", "CBB"),
    ("Survey Mode", "CSS"),
    
    ("Temp ON", "STI"),
    ("Temp OFF", "STO"),
    
    ("Ultra ON", "SUI"),
    ("Ultra OFF", "SUO"),
    
    ("Accel ON", "SAI"),
    ("Accel OFF", "SAO"),
    
    ("Hall ON", "SHI"),
    ("Hall OFF", "SHO"),
]

# Conversion from raw to value magnetometer (µT per LSB)
MAG_SCALE = 0.15

# GLOBAL STATES
ser = None
running = True

def calibrate(values):
    global Clbmtrx_acc
    try:
        timep = float(values[0])/100
        distance = 0.919*float(values[1])+0.625 #Calibrated
        temp_c = 0.961*(float(values[2])*(1/16))+0.641 #Calibrated
        acc_x = float(values[3]) * 0.015748
        acc_y = float(values[4]) * 0.015748
        acc_z = float(values[5]) * 0.015748
        raw_acc = np.array([acc_x, acc_y, acc_z, 1.0])
        calibrated_acc = Clbmtrx_acc @ raw_acc # Calibrated
        acc_x_c, acc_y_c, acc_z_c = calibrated_acc #Calibrated
        
        mag_x = float(values[6])*MAG_SCALE
        mag_y = float(values[7])*MAG_SCALE
        mag_z = float(values[8])*MAG_SCALE
        ground_t = 1.8079*float(values[9])-28.5537 # Calibrated
        ground_h = 0.858*float(values[10])+0.005 #Calibrated
    except Exception as e:
        raise ValueError(f"Calibration failed: {e}")
    return [timep, distance, temp_c, acc_x_c, acc_y_c, acc_z_c, mag_x, mag_y, mag_z, ground_t, ground_h]

    
def process_line(values, line, raw_buffer, calib_buffer, data_buffer, log_widget):
    try:
        timeperror= float(values[0])
        raw_buffer.append(values)

        try:
            calibrated = calibrate(values)
        except Exception as e:
            print(f"Calibration failed at {timeperror:.3f}, using raw values. Error: {e}")
            try:
                calibrated = list(map(float, values))
            except ValueError:
                print(f"Invalid raw data format at {values[0]}, skipping this line.")
                return None

        calib_buffer.append(calibrated)

        data_buffer.append({
            "timep": calibrated[0],
            "dist": calibrated[1],
            "temp_c": calibrated[2],
            "acc_x": calibrated[3],
            "acc_y": calibrated[4],
            "acc_z": calibrated[5],
            "mag_x": calibrated[6],
            "mag_y": calibrated[7],
            "mag_z": calibrated[8],
        })

        display_line = line + '\n'
        log_widget.insert(tk.END, display_line)
        log_widget.see(tk.END)

        return True

    except ValueError:
        print(f"Ignored malformed sensor line: {line}")
    except Exception as e:
        print(f"[Unexpected error in process_line] {e}")
    return None


# === SERIAL READER THREAD ===
def serial_reader(value_log, command_log):
    global ser, running, raw_buffer, calib_buffer, raw_filename, calib_filename, data_buffer

    with open(raw_filename, mode='w', newline='') as raw_file, \
         open(calib_filename, mode='w', newline='') as calib_file:

        raw_writer = csv.writer(raw_file, delimiter='\t')
        calib_writer = csv.writer(calib_file, delimiter='\t')

        # Write headers
        raw_writer.writerow(["Time", "Raw_Distance", "Raw_Temp", "Raw_AccX", "Raw_AccY", "Raw_AccZ",
                             "Raw_MagX", "Raw_MagY", "Raw_MagZ", "Raw_Ground_T", "Raw_Ground_H"])
        calib_writer.writerow(["Time", "Distance_mm", "Temp_C", "AccX_g", "AccY_g", "AccZ_g",
                               "MagX", "MagY", "MagZ", "Ground_T", "Ground_H"])

        while running:
            try:
                if ser is None or not ser.is_open:
                    command_log.insert(tk.END, "[Error] Serial port is not open.\n")
                    command_log.see(tk.END)
                    break

                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                values = line.split('\t')
                if len(values) == 11:
                    success = process_line(values, line, raw_buffer, calib_buffer, data_buffer, value_log)

                    if success and len(raw_buffer) >= FLUSH_THRESHOLD:
                        raw_writer.writerows(raw_buffer)
                        calib_writer.writerows(calib_buffer)
                        raw_buffer.clear()
                        calib_buffer.clear()
                else:
                    # Log any non-sensor line in the command log window
                    command_log.insert(tk.END, f"[Info] {line}\n")
                    command_log.see(tk.END)

            except Exception as e:
                command_log.insert(tk.END, f"[Error] {e}\n")
                command_log.see(tk.END) 

        # Final saving on exit
        if raw_buffer:
            raw_writer.writerows(raw_buffer)
        if calib_buffer:
            calib_writer.writerows(calib_buffer)


# === SEND COMMAND ===
def send_command(entry_widget, command_log):
    cmd = entry_widget.get().strip()
    if cmd:
        try:
            ser.write((cmd + '\n').encode('utf-8'))
            command_log.insert(tk.END, f"[Command sent] {cmd}\n")
            command_log.see(tk.END)
            entry_widget.delete(0, tk.END)
        except Exception as e:
            command_log.insert(tk.END, f"[Command Send Error] {e}\n")
            command_log.see(tk.END)

# Plotting
def create_plots(root):
    global data_buffer

    # Setup matplotlib figure with 3 subplots
    fig = Figure(figsize=(12, 6), dpi=100)

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
        ax.legend(loc='center left')

    ax1.set_ylabel("Temp (°C)")
    ax2.set_ylabel("Acc (g)")
    ax3.set_ylabel("Mag")

    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.get_tk_widget().pack(padx=10, pady=10)

    def update_plot():
        if not data_buffer:
            root.after(200, update_plot)
            return

        filtered = [d for d in data_buffer if d["timep"] > 0 and d["temp_c"] != 0]

        if not filtered:
            root.after(200, update_plot)
            return

        times = [d["timep"] for d in filtered]
        temps = [d["temp_c"] for d in filtered]

        acc_x = [d["acc_x"] for d in filtered]
        acc_y = [d["acc_y"] for d in filtered]
        acc_z = [d["acc_z"] for d in filtered]

        mag_x = [d["mag_x"] for d in filtered]
        mag_y = [d["mag_y"] for d in filtered]
        mag_z = [d["mag_z"] for d in filtered]

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
        root.after(200, update_plot)

    update_plot()


def add_control_buttons(command_frame, command_log):
    toggle_states = {}

    def send_serial_command(cmd):
        try:
            ser.write((cmd + '\n').encode('utf-8'))
            command_log.insert(tk.END, f"[Command sent] {cmd}\n")
            command_log.see(tk.END)
        except Exception as e:
            command_log.insert(tk.END, f"[Serial Error] {e}\n")
            command_log.see(tk.END)

    def create_toggle_button(label, cmd_on, cmd_off, confirm=False):
        def toggle():
            current = toggle_states.get(label, False)
            new_state = not current
            new_cmd = cmd_on if new_state else cmd_off
            color = "green" if new_state else "red"
            new_label = label

            def do_send():
                toggle_states[label] = new_state
                btn.config(bg=color, text=new_label)
                send_serial_command(new_cmd)

            if confirm:
                if messagebox.askyesno("Confirm Action", f"Are you sure you want to send '{new_cmd}'?"):
                    do_send()
            else:
                do_send()

        btn = tk.Button(command_frame, text=label, width=10, command=toggle, bg="red", fg="white")
        btn.pack(side=tk.LEFT, padx=3)
        toggle_states[label] = False

    # Sensor toggles
    create_toggle_button("Temp", "STI", "STO")
    create_toggle_button("Ultra", "SUI", "SUO")
    create_toggle_button("Accel", "SAI", "SAO")
    create_toggle_button("Hall", "SHI", "SHO")

    # Servo toggle with confirmation
    create_toggle_button("Servo", "SS1", "SS0", confirm=True)

    # Burst/Survey toggle
    mode_state = {"burst": False}

    def toggle_mode():
        mode_state["burst"] = not mode_state["burst"]
        cmd = "CBB" if mode_state["burst"] else "CSS"
        label = "Burst Mode" if mode_state["burst"] else "Survey Mode"
        color = "green" if mode_state["burst"] else "red"
        mode_btn.config(text=label, bg=color)
        send_serial_command(cmd)

    mode_btn = tk.Button(command_frame, text="Survey Mode", bg="red", fg="white",
                         width=12, command=toggle_mode)
    mode_btn.pack(side=tk.LEFT, padx=10)


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

    # Sensor log
    value_log = scrolledtext.ScrolledText(root, width=100, height=10)
    value_log.pack(padx=10, pady=(10, 5))

    # Command log
    command_log = scrolledtext.ScrolledText(root, width=100, height=5)
    command_log.pack(padx=10, pady=(0, 10))


    command_frame = tk.Frame(root)
    command_frame.pack(pady=5)

    add_control_buttons(command_frame, command_log)

    # Launch Serial thread
    threading.Thread(target=serial_reader, args=(value_log, command_log), daemon=True).start()

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
