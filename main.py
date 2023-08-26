import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import serial
import time
import threading
from collections import deque
import csv

# Commande pour Arduino
pump_command = "P:"  # Commande pour la pompe péristaltique


class SerialReader(threading.Thread):
    def __init__(self, port='/dev/cu.usbmodem14301'):
        super().__init__()
        self.serial = None
        self.datas = deque(maxlen=10)
        self.is_running = False
        self.port = port

    def initialize_serial(self):
        try:
            self.serial = serial.Serial(self.port, 9600, timeout=1)
            time.sleep(2)
            print("Connexion série établie sur le port :", self.port)
        except serial.SerialException as e:
            print("Échec de l'initialisation de la connexion série :", str(e))

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
                print("Échec de la lecture des données série :", str(e))

    def return_last_data(self):
        if self.datas:
            return self.datas[-1]
        else:
            return None


class Pump:
    def __init__(self, serial, speed=0, status=False, mode="controller"):
        self.serial = serial
        self.speed = speed
        self.status = status
        self.mode = mode

    def set_speed(self, speed):
        self.speed = speed
        # Envoyer la commande d'ajustement de la vitesse à la pompe via le port série

    def set_status(self, status):
        self.status = status
        # Envoyer la commande d'allumage/arrêt à la pompe via le port série

    def set_mode(self, mode):
        self.mode = mode

    def get_speed(self):
        return self.speed

    def get_status(self):
        return self.status

    def get_mode(self):
        return self.mode


class Sensor(threading.Thread):
    def __init__(self, name, unit, serial_input, rank, y_min=None, y_max=None):
        super().__init__()
        self.name = name
        self.data = deque(maxlen=1000)
        self.unit = unit
        self.isFull = False
        self.input = serial_input
        self.rank = rank
        self.lock = threading.Lock()  # Verrou pour accéder aux données du capteur
        self.is_running = False
        self.start_time = time.time()  # Heure de début
        self.y_min = y_min  # Valeur minimale pour la plage de l'axe Y
        self.y_max = y_max  # Valeur maximale pour la plage de l'axe Y

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
            value = float(last_data[self.rank])  # Convertir la valeur en float
            if value is not None:
                self.add_data(value)

                # Vérifier la plage de valeur de l'axe Y
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

        # Appeler start_sensing() pour commencer la surveillance immédiatement
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

            # Définir la plage de valeur de l'axe Y
            if sensor.y_min is not None and sensor.y_max is not None:
                sensor.subplot.set_ylim(sensor.y_min, sensor.y_max)

            sensor.canvas.draw()

        self.window.after(100, self.update_plots)

    def start_sensing(self):
        for sensor in self.sensors:
            sensor.start()

    def start_perfusion(self):
        for sensor in self.sensors:
            sensor.start_sensing()

        self.start_button.configure(text="Perfusion Running", state=tk.DISABLED)

    def stop_perfusion(self):
        for sensor in self.sensors:
            sensor.stop()

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
        arduino1.stop()  # Arrêter le thread SerialReader
        self.window.destroy()

    def run_gui(self):
        self.start_sensing()  # Démarrer les threads dès le début
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
