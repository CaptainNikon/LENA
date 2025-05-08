import serial
import threading
import tkinter as tk
from tkinter import scrolledtext
import time
import csv

# === CONFIGURATION ===
serial_port = 'COM3'  # Replace with your actual port
baud_rate = 115200
csv_filename = 'log_output_gui.csv'

# === GLOBAL STATE ===
ser = None
running = True


# === SERIAL READER THREAD ===
def serial_reader(log_widget):
    global ser, running
    with open(csv_filename, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter='\t')
        writer.writerow(["EpochTT", "Distance_mm", "Temp_C", "AccX_g", "AccY_g", "AccZ_g",
                         "MagX", "MagY", "MagZ", "Ground_T", "Ground_H"])

        while running:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    values = line.split('\t')
                    if len(values) == 10:
                        epoch_tt = time.time()
                        writer.writerow([epoch_tt] + values)
                        display_line = f"{epoch_tt:.3f}\t{line}\n"
                    else:
                        display_line = f"[Malformed] {line}\n"
                    log_widget.insert(tk.END, display_line)
                    log_widget.see(tk.END)
            except Exception as e:
                log_widget.insert(tk.END, f"[Error] {e}\n")
                log_widget.see(tk.END)


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


# === BUILD GUI ===
def start_gui():
    global ser

    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
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

    # Start the serial reading thread
    threading.Thread(target=serial_reader, args=(log_widget,), daemon=True).start()

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
    start_gui()
