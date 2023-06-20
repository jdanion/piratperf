import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import serial
import time
import threading
from collections import deque
import csv

# Command for Arduino
pump_command = "P:"  # Command for the roller pump


class SerialReader(threading.Thread):
    def __init__(self):
        super().__init__()
        self.serial = None
        self.datas = deque(maxlen=10)
        self.is_running = False

    def initialize_serial(self):
        # Default port for Arduino
        port = '/dev/cu.usbmodem14301'

        try:
            self.serial = serial.Serial(port, 9600, timeout=1)
            time.sleep(2)
            print("Serial connection established on port:", port)
        except serial.SerialException as e:
            print("Failed to initialize serial connection:", str(e))

    def run(self):
        self.is_running = True
        while self.is_running:
            self.read_serial_data()

    def stop(self):
        self.is_running = False

    def read_serial_data(self):
        if self.serial is not None and self.serial.is_open:
            try:
                line = self.serial.readline().decode().strip()
                data = line.split(',')
                time.sleep(0.002)
                self.serial.reset_input_buffer()
                self.datas.append(data)
            except serial.SerialException as e:
                print("Failed to read serial data:", str(e))

    def return_last_data(self):
        if self.datas:
            return self.datas[-1]
        else:
            return None


class Sensor(threading.Thread):
    def __init__(self, name, unit, serial_input, rank, y_min=None, y_max=None):
        super().__init__()
        self.name = name
        self.data = deque(maxlen=1000)
        self.unit = unit
        self.isFull = False
        self.input = serial_input
        self.rank = rank
        self.lock = threading.Lock()  # Lock for accessing sensor data
        self.is_running = False
        self.start_time = time.time()  # Start time
        self.y_min = y_min  # Minimum value for Y-axis range
        self.y_max = y_max  # Maximum value for Y-axis range

    def run(self):
        self.is_running = True
        while self.is_running:
            self.update_data()

    def stop(self):
        self.is_running = False
        self.join()

    def update_data(self):
        last_data = self.input.return_last_data()
        if last_data is not None:
            value = float(last_data[self.rank])  # Convert the value to float
            if value is not None:
                self.add_data(value)

                # Check Y-axis value range
                if self.y_min is not None and value < self.y_min:
                    self.y_min = value
                if self.y_max is not None and value > self.y_max:
                    self.y_max = value

        time.sleep(0.1)

    def add_data(self, value):
        with self.lock:
            self.data.append(value)

    def clear_data(self):
        with self.lock:
            self.data.clear()

    def start_sensing(self):
        if not self.is_running:
            self.start()

    def stop_sensing(self):
        if self.is_running:
            self.stop()


class GUI:
    def __init__(self):
        self.sensors = []
        self.window = tk.Tk()
        self.window.title("Perfusion GUI")

        self.start_button = tk.Button(self.window, text="Start Perfusion", command=self.start_perfusion)
        self.start_button.pack(side=tk.LEFT, padx=10, pady=10)

        self.save_button = tk.Button(self.window, text="Save Data", command=self.save_data_to_csv)
        self.save_button.pack(side=tk.LEFT, padx=10, pady=10)

        self.reset_button = tk.Button(self.window, text="Reset", command=self.reset_data)
        self.reset_button.pack(side=tk.LEFT, padx=10, pady=10)

        self.close_button = tk.Button(self.window, text="Close", command=self.close_gui)
        self.close_button.pack(side=tk.LEFT, padx=10, pady=10)

        # Call start_sensing() to start sensing immediately
        self.start_sensing()

    def add_sensor(self, sensor):
        self.sensors.append(sensor)

        figure = Figure(figsize=(5, 4), dpi=100)
        subplot = figure.add_subplot(111)
        canvas = FigureCanvasTkAgg(figure, master=self.window)
        canvas.get_tk_widget().pack(side=tk.LEFT, padx=10, pady=10)

        sensor.figure = figure
        sensor.subplot = subplot
        sensor.canvas = canvas

    def update_plots(self):
        for sensor in self.sensors:
            with sensor.lock:
                data = list(sensor.data)
            sensor.subplot.clear()
            sensor.subplot.plot(data)
            sensor.subplot.set_title(sensor.name)
            sensor.subplot.set_xlabel('Time')
            sensor.subplot.set_ylabel(sensor.unit)

            # Set Y-axis range
            if sensor.y_min is not None and sensor.y_max is not None:
                sensor.subplot.set_ylim(sensor.y_min, sensor.y_max)

            sensor.canvas.draw()

        self.window.after(100, self.update_plots)

    def start_sensing(self):
        for sensor in self.sensors:
            sensor.start_sensing()

    def start_perfusion(self):
        for sensor in self.sensors:
            sensor.start_sensing()

        self.start_button.configure(text="Perfusion Running", state=tk.DISABLED)

    def stop_perfusion(self):
        for sensor in self.sensors:
            sensor.stop_sensing()

        self.start_button.configure(text="Start Perfusion", state=tk.NORMAL)

    def save_data_to_csv(self):
        filename = "perfusion_data.csv"

        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Time"] + [sensor.name for sensor in self.sensors])

            start_time = min(sensor.start_time for sensor in self.sensors)
            for i, _ in enumerate(self.sensors[0].data):
                time_elapsed = time.time() - start_time
                row = [time_elapsed] + [sensor.data[i] if i < len(sensor.data) else "" for sensor in self.sensors]
                writer.writerow(row)

        print("Data saved to", filename)

    def reset_data(self):
        for sensor in self.sensors:
            sensor.clear_data()

    def close_gui(self):
        for sensor in self.sensors:
            sensor.stop_sensing()
        arduino1.stop()  # Stop the SerialReader thread
        self.window.destroy()

    def run_gui(self):
        self.start_sensing()  # Start the threads from the beginning
        self.update_plots()
        self.window.mainloop()


if __name__ == "__main__":
    gui = GUI()
    arduino1 = SerialReader()
    arduino1.initialize_serial()
    arduino1.start()

    flow_rate_sensor = Sensor("Flow Rate", "ml/min", arduino1, 0, y_min=0, y_max=300)
    pressure_sensor = Sensor("Pressure", "mmHg", arduino1, 1, y_min=20, y_max=30)
    art_oxy_sensor = Sensor("Oxygen Arterial", "mmHg", arduino1, 2, y_min=5, y_max=25)
    ven_oxy_sensor = Sensor("Oxygen Venous", "mmHg", arduino1, 3, y_min=60, y_max=150)

    gui.add_sensor(flow_rate_sensor)
    gui.add_sensor(pressure_sensor)
    gui.add_sensor(art_oxy_sensor)
    gui.add_sensor(ven_oxy_sensor)

    gui.run_gui()
