#!/usr/bin/env python
# coding: utf-8

import csv
import time
import collections
from random import randint
from typing import List, Dict, Callable, Any
from matplotlib.ticker import FuncFormatter
import inspect
import matplotlib.pyplot as plt
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QToolBox,
    QVBoxLayout,
    QPushButton,
    QWidget,
    QLabel,
    QDockWidget,
    QSlider,
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

import time
import serial
import statistics
import logging
import threading



class DeviceManager:
    def __init__(self):
        self.devices = {}

    def add_device(self, device):
        self.devices[device.name] = device

    def reconnect_device(self, device_name, max_retries=5, retry_interval=5):
        for i in range(max_retries):
            try:
                device = self.devices[device_name]
                device.close()
                device.serial = serial.Serial(device.port, device.baud_rate, timeout=1)
                device.start()
                return True
            except serial.SerialException as e:
                print(f"Reconnection attempt {i + 1} failed: {e}")
                time.sleep(retry_interval)
        return False

    def check_devices(self):
        for device_name, device in self.devices.items():
            if not device.serial or not device.serial.isOpen():
                print(
                    f"Device {device_name} is disconnected. Attempting to reconnect..."
                )
                if not self.reconnect_device(device_name):
                    print(f"Failed to reconnect to device {device_name}.")


class SerialInterface:
    def __init__(self, port, name, baud_rate=9600, data_length=1):
        self.port = port
        self.name = name
        self.baud_rate = baud_rate
        self.serial = None
        self.is_running = False
        self.data_length = data_length
        self.thread = None
        self.serial_lock = threading.Lock()

        self.init_serial()

    def init_serial(self):
        try:
            self.serial = serial.Serial(self.port, self.baud_rate, timeout=1)
        except serial.SerialException as e:
            logging.error(f"Error opening serial port {self.port}: {e}")
            raise e

    def read_raw_data(self):
        data = [float(x) for x in self.serial.readline().decode().strip().split(",")]
        return data

    def start(self):
        if not self.serial:
            logging.error(f"Serial port {self.port} is not open")
            self.init_serial()
            return
        self.is_running = True
        self.thread = threading.Thread(target=self.run)
        self.thread.start()
        self.serial_lock = threading.Lock()

    def run(self):
        while self.is_running:
            data = self.read_serial_data()
            if data is None:
                break

    def stop(self):
        self.is_running = False
        if self.thread is not None:
            self.thread.join()

    def read_serial_data(self):

        if not self.serial:
            logging.error(f"Serial port {self.port} is not open")
            print("error no serial")
            self.init_serial()
            print("Tried to re-initialize serial")
            return None

        with self.serial_lock:

            try:
                data = [
                    float(x) for x in self.serial.readline().decode().strip().split(",")
                ]
                time.sleep(0.02)  # Suspend execution for 50 ms
            except Exception as e:
                logging.error(f"Error reading serial data from {self.name}: {e}")
                print(f"Exception while reading: {e}")
                return None


            if len(data) != self.data_length:
                logging.error(
                    f"Error: The number of data read ({len(data)}) is different from data_length ({self.data_length})."
                )
                return None

            logging.debug(data)
            return data

    def write_data(self, data):
        if not self.serial:
            logging.error(f"Serial port {self.port} is not open")
            return
        with self.serial_lock:
            try:
                self.serial.write(data)
            except Exception as e:
                logging.error(f"Error writing data to {self.name}: {e}")

    def close(self):
        if self.serial:
            self.serial.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        self.close()



class Sensor:
    def __init__(self, name, unit, color="b", y_range=None):
        self.name = name
        self.unit = unit
        self.data = collections.deque(maxlen=300)  # Conserve les 600 dernières valeurs
        self.sensor_type = None
        self.color = color
        self.y_range = y_range
        self.is_working = True
        self.mean_and_std = {
            "last_values": {"mean": None, "std": None},
            "all_values": {"mean": None, "std": None},
        }

    def add_data(self, data):
        """Ajoute une donnée au capteur."""
        self.data.append(data)

    def get_last_data(self):
        """Retourne la dernière donnée ajoutée au capteur."""
        return self.data[-1] if self.data else None

    def get_plot_data(self):
        """Retourne toutes les données du capteur pour l'affichage sur un graphique."""
        return list(self.data)

    def clear_data(self):
        self.data = collections.deque(maxlen=600)

    def update_statistics(self):
        """Mise à jour des statistiques pour les dernières valeurs."""
        if self.data:
            _, values = zip(*self.data)
            self.mean_and_std["last_values"]["mean"] = statistics.mean(values)
            self.mean_and_std["last_values"]["std"] = statistics.stdev(values)

    update_statistics.button_label = "Update statistics"

class SerialSensor(Sensor):
    def __init__(
        self,
        name,
        unit,
        device,
        data_column,
        y_min=None,
        y_max=None,
        color="b",
        y_range=None,
    ):
        super().__init__(name=name, unit=unit, color=color, y_range=y_range)
        self.device = device
        self.data_column = data_column
        self.y_min = y_min  # Minimum value for Y-axis range
        self.y_max = y_max  # Maximum value for Y-axis range
        self.sensor_type = "serial"

    def add_data(self, data):
        """Ajoute une donnée au capteur. La donnée est un tuple (timestamp, valeur)."""
        # Vérifie que la valeur est dans l'intervalle y_min, y_max avant d'ajouter les données
        timestamp, value = data
        if (self.y_min is not None and value < self.y_min) or (
            self.y_max is not None and value > self.y_max
        ):
            return  # Ignore silently and don't add the data if it's out of range
        super().add_data(data)

    def clear_data(self):
        super().clear_data()

class PressureSensor(SerialSensor):
    def __init__(self, name, unit, device, data_column, y_min=None, y_max=None, color="b", y_range=None):
        super().__init__(name=name, unit=unit, device=device, data_column=data_column, y_min=y_min, y_max=y_max, color=color, y_range=y_range)
        self.sensor_type = "pressure"

        self.offset = 0
        self.scale = 1

    def tare(self):
        data = "2,t," + str(randint(0, 100))  # 't' est une chaîne de caractères et chr(value) convertit la valeur en caractère
        self.device.write_data(data.encode())

    tare.button_label = "Tare"
    def calibrate(self):
       data = "2,c," + str(randint(0, 100))
       self.device.write_data(data.encode())

    calibrate.button_label = "Calibrate"

    def add_data(self, data):
        super().add_data(data)


class FlowSensor(SerialSensor):

    def __init__(self, name, unit, device, data_column, y_min=None, y_max=None, color="b", y_range=None):
        super().__init__(name=name, unit=unit, device=device, data_column=data_column, y_min=y_min, y_max=y_max, color=color, y_range=y_range)
        self.sensor_type = "flow"

    def add_data(self, data):
        super().add_data(data)


class TemperatureSensor(SerialSensor):

        def __init__(self, name, unit, device, data_column, y_min=None, y_max=None, color="b", y_range=None):
            super().__init__(name=name, unit="°C", device=device, data_column=data_column, y_min=y_min, y_max=y_max, color=color, y_range=y_range)
            self.sensor_type = "temperature"

class OxySensor(SerialSensor):

        def __init__(self, name, unit, device, data_column, y_min=None, y_max=None, color="b", y_range=None):
            super().__init__(name=name, unit="%", device=device, data_column=data_column, y_min=y_min, y_max=y_max, color=color, y_range=y_range)
            self.sensor_type = "oxygen"

        def add_data(self, data):
            super().add_data(data)

class PhSensor(SerialSensor):

            def __init__(self, name, unit, device, data_column, y_min=None, y_max=None, color="b", y_range=None):
                super().__init__(name=name, unit="pH", device=device, data_column=data_column, y_min=y_min, y_max=y_max, color=color, y_range=y_range)
                self.sensor_type = "ph"

class Pump(SerialSensor):

            def __init__(self, name, unit, device, data_column, y_min=None, y_max=None, color="b", y_range=None):
                super().__init__(name=name, unit="RPM", device=device, data_column=data_column, y_min=y_min, y_max=y_max, color=color, y_range=y_range)
                self.sensor_type = "pump"
                self.control_mode = False
                self.speed = 0
                self.control_target = 0
                self.control_sensor = None
                self.is_running = False
            def run(self):
                self.is_running = True
                data = "1,r,100"
                self.device.write_data(data.encode())

            run.button_label = "Run Pump"
            def stop(self):
                self.is_running = False
                data = "1,s,0"
                self.device.write_data(data.encode())

            stop.button_label = "Stop Pump"
            def set_speed(self, speed):
                self.control_mode = False
                data = "1,r,"+str(speed)
                self.device.write_data(data.encode())

            set_speed.button_label = "Set Speed"

            def control_mode(self):
                self.control_mode = True
                data = "1c0"
                self.device.write_data(data.encode())

            control_mode.button_label = "Control Mode"

class WeightSensor(SerialSensor):

            def __init__(self, name, unit, device, data_column, y_min=None, y_max=None, color="b", y_range=None):
                super().__init__(name=name, unit="g", device=device, data_column=data_column, y_min=y_min, y_max=y_max, color=color, y_range=y_range)
                self.sensor_type = "weight"

class Valve(SerialSensor):

            def __init__(self, name, unit, device, data_column, y_min=None, y_max=None, color="b", y_range=None):
                super().__init__(name=name, unit="°", device=device, data_column=data_column, y_min=y_min, y_max=y_max, color=color, y_range=y_range)
                self.sensor_type = "valve"




class ManualSensor(Sensor):
    def __init__(
        self, name, unit, min_value, max_value, color="b", y_range=None, **kwargs
    ):
        super().__init__(name=name, unit=unit, color=color, y_range=y_range)
        self.min_value = min_value
        self.max_value = max_value
        self.current_value = min_value
        self.sensor_type = "manual"

    def set_value(self, value):
        if not self.min_value <= value <= self.max_value:
            raise ValueError("Value out of range")
        self.current_value = value

    def get_value(self):
        return self.current_value

    def clear_data(self):
        super().clear_data()

    def add_data(self, timestamp):
        data = timestamp, self.current_value
        super().add_data(data)



class VirtualSensor(Sensor):
    def __init__(self, name, unit, color="b", y_range=None, **kwargs):
        super().__init__(name=name, unit=unit, color=color, y_range=y_range)
        self.sensor_type = "virtual"

        self.virtual_types = {
            "resistance": self.compute_resistance,
        }

        self.compute_function = self.virtual_types[kwargs["virtualtype"]]
        self.related_sensors = kwargs["related_sensors"]

    def compute_resistance(self, pressure_data, flow_data):
        if not pressure_data or not flow_data:
            return None

        pressure_value = pressure_data[-1][1]
        flow_value = flow_data[-1][1]

        if flow_value != 0:
            return pressure_value / flow_value
        else:
            return None

    def read_data(self):
        # Extract the data from the related sensors
        pressure_data = self.related_sensors[0].get_plot_data()
        flow_data = self.related_sensors[1].get_plot_data()

        return self.compute_function(pressure_data, flow_data)
    def add_data(self, timestamp):
        data = timestamp, self.read_data()
        super().add_data(data)

    def clear_data(self):
        super().clear_data()

class Animal:

    def __init__(self, race, weight, sex):

        self.race = race
        self.weight = weight
        self.sex = sex


class Organ:

    def __init__(self, surgical_procedure):

        self.type = None
        self.weight = None
        self.surgical_procedure = surgical_procedure
        self.harvesting_duration = None
        self.perfusion_start_time = None
        self.perfusion_duration = None

class SurgicalProcedure:

        def __init__(self):

            self.name = None
            self.start_time = None
            self.end_time = None
            self.duration = None
            self.surgeon = None
            self.assistant = None
            self.incident = []
            self.animal = None
            self.organs = []
            self.canulation_start_time = None
            self.canulation_end_time = None
            self.canulation_duration = None
            self.cooling_start_time = None
            self.fonctionnal_warm_ischemia_start_time = None

        def start(self):
            self.start_time = time.perf_counter()

        def end(self):
            self.end_time = time.perf_counter()
            self.duration = self.end_time - self.start_time

        def set_incident(self, incident):
            self.incident.append(incident)

        def get_incident(self):
            return self.incident

        def add_animal(self, animal):
            self.animal = animal

        def harvest_liver(self, weight):
            liver = Organ(self)
            liver.type = "liver"
            liver.weight = weight
            liver.harvesting_duration = time.perf_counter() - self.cooling_start_time
            self.organs.append(liver)
        def canulation_start(self):
            self.canulation_start_time = time.perf_counter()

        def canulation_done(self):
            self.canulation_end_time = time.perf_counter()
            self.canulation_duration = self.canulation_end_time - self.canulation_start_time

        def cooling_start(self):
            self.cooling_start_time = time.perf_counter()


class Perfusion:
    def __init__(
        self,
        device_manager,
        sensing_frequency=1,
        record_frequency=600,
        csv_file="perfusion.csv",
    ):
        self.device_manager = device_manager
        self.sensing_frequency = sensing_frequency
        self.record_frequency = record_frequency
        self.normalized_data = collections.deque(maxlen=86400)  # For 24 hours at 1Hz
        self.csv_file = csv_file  # file for recording
        self.sensors = []  # list of sensors
        self.sensor_groups = []  # list of lists
        self.reading_thread = None
        self.recording_thread = None
        self.gui = None
        self.stop_event = threading.Event()  # Event to stop recording data
        self.sensor_data_lock = threading.Lock()
        self.events = collections.deque(maxlen=300)  # Record events
        self.start_time = time.perf_counter()
        self.end_time = None
        self.duration = None
        self.organ = None
        self.auto_record = False

    def add_sensor(self, sensor, group_index=None):
        if group_index is None:
            # If no group specified, add sensor in its own group
            self.sensor_groups.append([sensor])
        else:
            # If group specified, add sensor to this group
            self.sensor_groups[group_index].append(sensor)
        self.sensors.append(sensor)

    def add_gui(self):
        self.gui = GUI(self)


    def launch_gui(self):
        if self.gui:
            self.gui.run_gui()

    def add_organ(self, organ):
        self.organ = organ

    def read_sensor_data(self):
        last_avg_time = time.perf_counter()
        with self.sensor_data_lock:
            while not self.stop_event.is_set():
                timestamp = time.perf_counter() - self.start_time
                # Read data from serial sensors
                for device_name, device in self.device_manager.devices.items():
                    try:
                        data = device.read_serial_data()
                        time.sleep(0.02)  # Suspend execution for 50 ms
                    except Exception as e:
                        print(f"Error reading data from {device_name}: {e}")
                        continue
                    if data is not None:
                        for sensor in self.sensors:
                            if hasattr(sensor, 'device') and sensor.device == device:
                                value = data[sensor.data_column]
                                sensor.add_data((timestamp, value))
                            else :
                                sensor.add_data(timestamp)



                    # Calculate and store average every second
                current_time = time.perf_counter()
                if current_time - last_avg_time >= 1:
                    avg_data = [timestamp]
                    for sensor in self.sensors:

                        if sensor.data:

                            timestamps, values = zip(*sensor.data)
                            if values:
                                cleaned_values = [x for x in values if x is not None]  # Supprime les valeurs None
                                if cleaned_values:  # Continue seulement si la liste nettoyée n'est pas vide
                                    avg_data.append(sum(cleaned_values) / len(cleaned_values))
                    if avg_data:
                        self.normalized_data.append(avg_data)
                    #print(self.normalized_data)
                    last_avg_time = current_time

    def start_sensing(self):
        self.stop_event.clear()
        #print("Trying to start sensing")
        #print(self.sensing_thread)
        if self.reading_thread is None or not self.reading_thread.is_alive():
            self.reading_thread = threading.Thread(target=self.read_sensor_data)
            self.reading_thread.start()
            print("Perfusion start_sensing called")
            if self.gui:
                self.gui.update_message_board("sensing")
            # self.gui.update_message_board('sensing')

    def clear_sensing(self):
        with self.sensor_data_lock:
            self.stop_sensing()
            for sensor in self.sensors:
                sensor.clear_data()
        print("Perfusion clear_sensing called")
        if self.gui:
            self.gui.update_message_board("data_cleared")


    def record_data(self):
        print("Starting data recording...")  # Debug message
        try:
            with open(self.csv_file, "w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(
                    ["Timestamp", "Sensor1", "Sensor2", "Sensor3", "..."]
                )  # Replace with your actual sensor names
                print("CSV header written.")  # Debug message


                print("Inside while loop.")  # Debug message

                # Ensure self.normalized_data is a list of lists where each sublist contains time and sensor data
                normalized_data_copy = list(self.normalized_data)
                print(f"Normalized data: {normalized_data_copy}")  # Debug message

                for data_row in normalized_data_copy:
                    writer.writerow(data_row)

                # Write events if needed
                timestamp = time.perf_counter()
                for event in self.events:
                    writer.writerow([timestamp, "Event", event])
    
            print("Data recording stopped.")  # Debug message
            print("Data saved to csv")  # Debug message
        except Exception as e:
            print(f"Error recording data to {self.csv_file}: {e}")


    def continuous_record_data(self):
        self.auto_record = True
        while self.auto_record:
            print("Starting data recording durint auto record")  # Debug message
            self.record_data()
            print("Waiting for next recording...")
            time.sleep(self.record_frequency*60)

    def start_recording(self):
        print("Starting auto data recording...")  # Debug message
        if self.recording_thread is None or not self.recording_thread.is_alive():
            self.recording_thread = threading.Thread(target=self.continuous_record_data)
            self.recording_thread.start()
            print("Perfusion start_recording called")
            if self.gui:
                self.gui.update_message_board("recording")

    def stop_sensing(self):
        if self.reading_thread is not None:
            self.stop_event.set()
            self.reading_thread.join()
            self.sensing_thread = None
            print("Perfusion stop_sensing called")
            if self.gui:
                self.gui.update_message_board("stop_sensing")

    def start(self):
        # Lancez vos autres threads (lecture des données du capteur, etc.) ici
        threading.Thread(target=self.read_sensor_data).start()
        #threading.Thread(target=self.record_data).start()
        # Lancez l'interface utilisateur dans le thread principal
        # self.launch_gui()

    def print_data(self):
        print(self.normalized_data)

    def print_perfusion_status(self):
        print(self.start_time)

    def canulate(self):
        self.gui.update_message_board("canulate")

    def calibrate(self):
        self.gui.update_message_board("calibrate")

    def set_oxygen(self):
        self.gui.update_message_board("oxygen_set")


class GUI(QMainWindow):
    buttons_data: list[dict[str, str | Callable[[], None]] | dict[str, str | Callable[[], None]] | dict[
        str, str | Callable[[], None]] | dict[str, str | Any] | dict[str, str | Callable[[], None]] | dict[
                           str, str | Callable[[], None]] | dict[str, str | Callable[[], None]]]

    def __init__(self, perfusion):
        super().__init__()
        self.perfusion = perfusion
        self.gui_refresh_rate = 50

        self.main_widget = QWidget(self)
        self.setCentralWidget(self.main_widget)

        self.fig, self.axs = plt.subplots(
            len(self.perfusion.sensor_groups),
            1,
            figsize=(10, 5),
            constrained_layout=True,
        )
        self.toolBox = QToolBox(self)
        self.layout = QVBoxLayout(self.main_widget)
        self.canvas = FigureCanvas(self.fig)
        self.layout.addWidget(self.canvas)
        self.dock_widget = QDockWidget("Controls", self)
        self.dock_toolbox = QToolBox(self.dock_widget)
        self.addDockWidget(Qt.RightDockWidgetArea, self.dock_widget)
        self.dock_widget_contents = QWidget()
        self.dock_widget.setWidget(self.dock_widget_contents)
        self.dock_layout = QVBoxLayout(self.dock_widget_contents)
        self.dock_widget_contents.setLayout(self.dock_layout)
        self.message_board = QLabel(self)
        self.layout.addWidget(self.message_board)

        self.messages_library = {
            "sensing": "Capteurs en fonction",
            "stop_sensing": "Sensing stopped",
            "recording": "Enregistrement en cours",
            "sensor_error": "Erreur de capteur",
            "sensor_rate": "Current sensoring rate is "
            + str(self.perfusion.sensing_frequency),
            "canulate": "Canulation done",
            "calibrate": "Calibration done",
            "oxygen_set": "Oxygen value set",
            'data_cleared': 'Data sensors cleared'
            # Ajoutez autant de messages que vous voulez
        }

        self.buttons_data = [
            dict(name="Start sensing", function=self.start_sensing, location="main", id="start_sensing"),
            dict(name="Stop sensing", function=self.stop_sensing, location="main", id="stop_sensing"),
            dict(name="Clear sensing", function=self.clear_sensing, location="main", id="clear_sensing"),
            dict(name="Auto recording", function=self.perfusion.start_recording, location="Data", attribut="checkable", id="auto_recording"),
            dict(name="Canulation", function=self.canulate, location="Surgery", id="canulate"),
            dict(name="Calibration", function=self.calibrate, location="Circuit", id="calibrate"),
            dict(name="Record", function=self.perfusion.record_data, location="Data", id="record"),
            dict(name="Oxygenation", function=self.set_oxygen, location="Circuit", id="oxygenation")
            # ... (autres boutons) ...
        ]
        self.buttons_dict = {}
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_plots)
        self.update_timer.start(1000)  # milliseconds

        self.init_sensor_groups()
        self.init_toolbox()
        self.init_sensor_buttons()

    def init_sensor_groups(self):

        for sensor_group, ax in zip(self.perfusion.sensor_groups, self.axs):
            for sensor in sensor_group:
                ax.set_title(sensor.name, fontsize=10)  # Ajoute le titre au graphique
                ax.set_ylabel(sensor.unit)  # Ajoute l'unité à l'axe y
                if sensor.y_range is not None:
                    ax.set_ylim(sensor.y_range)
                if isinstance(sensor, ManualSensor):
                    slider = QSlider(Qt.Horizontal, self)
                    slider.setRange(sensor.min_value, sensor.max_value)
                    slider.setValue(sensor.current_value)
                    slider.valueChanged.connect(sensor.set_value)
                    self.layout.addWidget(slider)


    def init_toolbox(self):
        self.toolBox = QToolBox(self.dock_widget_contents)
        self.dock_layout.addWidget(self.toolBox)
    
        toolbox_panels = {}
    
        for button_data in self.buttons_data:
            location = button_data.get("location", "main")
    
            if location == "main":
                self.layout.addWidget(self.create_button(button_data))
            else:
                panel = toolbox_panels.get(location)
                if panel is None:
                    panel = QWidget()
                    panel_layout = QVBoxLayout()
                    panel.setLayout(panel_layout)
                    toolbox_panels[location] = panel
                    self.toolBox.addItem(
                        panel, location
                    )  # Chaque location différente devient un nouvel accordeon
    
                panel.layout().addWidget(self.create_button(button_data))
    
    def init_sensor_buttons(self):
        for sensor_group in self.perfusion.sensor_groups:
            for sensor in sensor_group:
                accordion = QWidget()  # Crée un nouveau QWidget pour chaque capteur
                layout = QVBoxLayout()
        
                for name, method in inspect.getmembers(sensor, predicate=inspect.ismethod):
                    button_label = getattr(method, "button_label", None)
                    if button_label:  # Si la méthode a un bouton associé
                        button = QPushButton(button_label)
                        button.clicked.connect(method)
                        layout.addWidget(button)
        
                accordion.setLayout(layout)
                self.dock_toolbox.addItem(accordion, sensor.name)
        
        self.dock_layout.addWidget(self.dock_toolbox)

    def create_button(self, button_data):
        button = QPushButton(button_data["name"], self)
        button.clicked.connect(button_data["function"])
    
        if "attribut" in button_data and button_data["attribut"] == "checkable":
            button.setCheckable(True)
            button.setChecked(self.perfusion.auto_record)
            button.toggled.connect(self.toggle_recording)
    
        self.buttons_dict[button_data["id"]] = button
        return button

    def toggle_recording(self, checked):
        self.perfusion.auto_record = checked
        button = self.buttons_dict.get("auto_recording")
        if button:
            if checked:
                button.setText("Stop Recording")
                self.perfusion.start_recording()
            else:
                button.setText("Start Recording")

    def start_sensing(self):
        print("GUI Start sensing called")
        self.perfusion.start_sensing()
        self.update_timer.start(1000)

    def clear_sensing(self):
        print("Clear sensing called")  # Pour le débogage
        self.stop_sensing()
        self.perfusion.clear_sensing()  # Assurez-vous que cette méthode efface toutes les données des capteurs
        self.perfusion.start_time = time.perf_counter()
        self.update_plots()  # Force la mise à jour de l'affichage


    def stop_sensing(self):
        print("GUI Stop sensing called")
        self.perfusion.stop_sensing()
        self.update_timer.stop()


    def update_plots(self):

        def seconds_to_hhmmss(seconds):
            hours, remainder = divmod(seconds, 3600)
            minutes, seconds = divmod(remainder, 60)
            return f"{int(hours):02}:{int(minutes):02}:{int(seconds):02}"

        def format_func(value, tick_number):
            return seconds_to_hhmmss(value)

        for i, (sensor, ax) in enumerate(zip(self.perfusion.sensors, self.axs)):
            data = sensor.get_plot_data()
            ax.clear()
            if data:
                timestamps, values = zip(*data)
                ax.plot(timestamps, values, color=sensor.color, label=sensor.name)
                if sensor.mean_and_std is not None:
                    mean_value = sensor.mean_and_std["last_values"]["mean"]
                    std_value = sensor.mean_and_std["last_values"]["std"]
                    if mean_value is not None and std_value is not None:
                        ax.axhline(y=mean_value, color='r', linestyle='--', label='Mean',linewidth=0.8)
                        ax.axhline(y=mean_value+ std_value, color='g', linestyle='--', label='Mean + 1*STD', linewidth=0.8)
                        ax.axhline(y=mean_value - std_value, color='g', linestyle='--', label='Mean - 1*STD', linewidth=0.8)
                        ax.fill_between(
                            timestamps,
                            mean_value - std_value,
                            mean_value + std_value,
                            color=sensor.color,
                            alpha=0.3,
                        )
                # Personnaliser l'axe des x pour afficher le temps en format HH:MM:SS
                if i == len(self.axs) - 1:  # Seulement pour la dernière sous-figure
                    ax.xaxis.set_major_formatter(FuncFormatter(format_func))
            else:
                ax.text(
                    0.5,
                    0.5,
                    "No Data",
                    horizontalalignment="center",
                    verticalalignment="center",
                    transform=ax.transAxes,
                )
    
            ax.set_title(sensor.name, fontsize=8)
            ax.set_ylabel(sensor.unit, fontsize=8)
            if sensor.y_range is not None:
                ax.set_ylim(sensor.y_range)
    
            # Masquer les étiquettes de l'axe des x pour toutes les sous-figures sauf la dernière
            if i < len(self.axs) - 1:
                ax.set_xticklabels([])
                ax.set_xlabel("")
        # Activer et définir les étiquettes de l'axe des x pour la dernière sous-figure
        self.axs[-1].set_xlabel("Time (HH:MM:SS)")
    
        self.fig.set_constrained_layout_pads()
        self.fig.canvas.draw_idle()


    def run_gui(self):
        self.update_timer.start(1000)  # milliseconds

    def show_gui(self):
        self.show()

    def update_refresh_rate(self, refresh_rate=50):
        self.update_timer.stop()
        self.update_refresh_rate = refresh_rate
        self.update_timer.start(self.gui_refresh_rate)

    def update_message_board(self, key):
        # Vérifiez que la clé est dans la bibliothèque de messages
        if key in self.messages_library:
            self.message_board.setText(self.messages_library[key])
        else:
            self.message_board.setText(str(key))

    def create_event(self):
        button = self.sender()  # récupère le bouton qui a déclenché l'événement
        event_name = button.text()
        print(
            f"Event {event_name} created"
        )  # remplacez cette ligne par le code pour créer un événement
        # ...

    def canulate(self):
        self.update_message_board("canulate")

    def calibrate(self):
        self.update_message_board("calibrate")

    def set_oxygen(self):
        self.update_message_board("oxygen_set")


%matplotlib qt

app = QApplication([])

device_manager = DeviceManager()

arduino1 = SerialInterface("/dev/cu.usbmodem14601", "arduino1", 9600, 4)

device_manager.add_device(arduino1)

surgery = SurgicalProcedure()

rat = Animal("Lewis", 250, "M")

surgery.add_animal(rat)

surgery.start()

surgery.canulation_start()

surgery.canulation_done()

surgery.cooling_start()

surgery.harvest_liver(10)

perfusion = Perfusion(
    device_manager, sensing_frequency=4, record_frequency=1, csv_file="data.csv"
)
pump = Pump(
    "Pump Speed", "rpm", arduino1, 0, color="k", y_range=(0, 100)
)
flow_rate_sensor = FlowSensor(name="Flow rate", unit = "mL/min", device= arduino1, data_column=1, color="r", y_range=(0, 40))

pressure_sensor = PressureSensor(
    "Pressure arterial line", "mmHg", arduino1, 3, color="g"
)
art_oxy_sensor = OxySensor(
    "Arterial oxygen saturation", "%", arduino1, 2, color="y"
)


oxygen_flow = ManualSensor("Oxygen gaz flow", "%", 0, 100, color="c", y_range=(0, 100))

resistance_sensor = VirtualSensor(
    name="Resistance Sensor",
    unit="mmHg/mL/min",
    virtualtype="resistance",
    related_sensors=[pressure_sensor, flow_rate_sensor],
    color="m",
)

perfusion.add_sensor(flow_rate_sensor)
perfusion.add_sensor(pressure_sensor)
perfusion.add_sensor(art_oxy_sensor)
perfusion.add_sensor(pump)
perfusion.add_sensor(oxygen_flow)
perfusion.add_sensor(resistance_sensor)

perfusion.add_organ(surgery.organs[0])

# In[11]:

perfusion.add_gui()
perfusion.gui.show_gui()
app.exec_()

# In[11]:

perfusion.start()

# In[ ]:


#perfusion.add_gui()

# In[21]:


print(perfusion.normalized_data)

# In[ ]:

# In[ ]:
perfusion.launch_gui()

# In[ ]:
perfusion.gui.run_gui()

# In[ ]:
perfusion.start()

# In[ ]:
perfusion.stop_sensing()

# In[ ]:
type(perfusion.device_manager.devices["arduino1"].serial_lock)

# In[ ]:

perfusion.device_manager.devices["arduino1"].init_serial()
# In[ ]:
perfusion.record_data()
# In[ ]:
type(perfusion.sensors[0].device)
