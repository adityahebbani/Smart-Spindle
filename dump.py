import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog, messagebox
import threading
import serial
import serial.tools.list_ports
import json
import re

class SmartSpindleGUI:
    def __init__(self, master):
        self.master = master
        master.title("Smart Spindle Data Logger")
        master.geometry("700x500")

        # Serial connection variables
        self.ser = None
        self.read_thread = None
        self.stop_thread = False

        # Top frame for serial port selection and connect button
        self.top_frame = ttk.Frame(master)
        self.top_frame.pack(padx=10, pady=5, fill='x')

        ttk.Label(self.top_frame, text="Serial Port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(self.top_frame, textvariable=self.port_var, width=15)
        self.port_combo['values'] = self.get_serial_ports()
        self.port_combo.pack(side="left", padx=5)

        self.connect_button = ttk.Button(self.top_frame, text="Connect", command=self.toggle_connect)
        self.connect_button.pack(side="left", padx=5)

        # Middle frame for action buttons
        self.middle_frame = ttk.Frame(master)
        self.middle_frame.pack(padx=10, pady=5, fill='x')
        self.dump_button = ttk.Button(self.middle_frame, text="Dump Flash", command=self.dump_flash, state="disabled")
        self.dump_button.pack(side="left", padx=5)
        self.export_button = ttk.Button(self.middle_frame, text="Export JSON", command=self.export_json, state="disabled")
        self.export_button.pack(side="left", padx=5)

        # Scrolled text for output
        self.output_text = scrolledtext.ScrolledText(master, wrap='word')
        self.output_text.pack(padx=10, pady=5, fill='both', expand=True)

        # Data structure to store events
        self.events = []

    def get_serial_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        return ports

    def toggle_connect(self):
        if self.ser and self.ser.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "Please enter/select a serial port.")
            return

        try:
            # Change baudrate if needed
            self.ser = serial.Serial(port, 9600, timeout=1)
            self.connect_button.config(text="Disconnect")
            self.dump_button.config(state="normal")
            self.export_button.config(state="normal")
            self.output_text.insert(tk.END, f"Connected to {port}\n")
            self.stop_thread = False
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))

    def disconnect(self):
        self.stop_thread = True
        if self.ser:
            self.ser.close()
            self.ser = None
        self.connect_button.config(text="Connect")
        self.dump_button.config(state="disabled")
        self.export_button.config(state="disabled")
        self.output_text.insert(tk.END, "Disconnected\n")

    def read_serial(self):
        while not self.stop_thread and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
                if line:
                    self.master.after(0, self.append_text, line + "\n")
                    self.parse_event_line(line)
            except Exception:
                break

    def append_text(self, text):
        self.output_text.insert(tk.END, text)
        self.output_text.see(tk.END)

    def dump_flash(self):
        if self.ser and self.ser.is_open:
            # Clear previous output and event list
            self.output_text.delete('1.0', tk.END)
            self.events = []
            # Send the dumpflash command (with CRLF termination)
            self.ser.write(b"dumpflash\r\n")
        else:
            messagebox.showerror("Error", "Serial port not connected.")

    def parse_event_line(self, line):
        # Example expected flash log line format:
        # "[2025-06-24 14:30:00] ROLL_CHANGE: 0.00"
        # "[2025-06-24 14:30:05] DISPENSE: 12.50"
        pattern = r"\[([^\]]+)\]\s*(ROLL_CHANGE|DISPENSE):\s*([\d\.\-]+)"
        match = re.search(pattern, line)
        if match:
            event = {
                "datetime": match.group(1),
                "type": match.group(2),
                "revolutions": float(match.group(3))
            }
            self.events.append(event)

    def export_json(self):
        if not self.events:
            messagebox.showinfo("No Data", "No events have been captured.")
            return
        file_path = filedialog.asksaveasfilename(defaultextension=".json",
                                                 filetypes=[("JSON files", "*.json")])
        if file_path:
            try:
                with open(file_path, "w") as f:
                    json.dump(self.events, f, indent=2)
                messagebox.showinfo("Success", f"Data exported to {file_path}")
            except Exception as e:
                messagebox.showerror("Export Error", str(e))

    def on_close(self):
        self.disconnect()
        self.master.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = SmartSpindleGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()