import pyb
from pyb import Pin
from time import ticks_us, ticks_diff

class Multi_Sensor:
    def __init__(self, sensor_pins):
        """
        Initialize multiple QTR-8RC sensors.
        """
        self.sensors = [Pin(pin, Pin.OUT_PP) for pin in sensor_pins]  # Store sensor pins as output initially
        self.sensor_pins = sensor_pins  # Store pin numbers
        self.white_datum = [0] * len(sensor_pins)  # Initialize white calibration values
        self.black_datum = [4000] * len(sensor_pins)  # Initialize black calibration values

    def calibrate(self, white=True):
        """Calibrate the sensors for white or black surface."""
        readings = self.read_sensors()
        for i in range(len(readings)):
            if readings[i] is not None:
                if white:
                    self.white_datum[i] = readings[i]
                else:
                    self.black_datum[i] = readings[i]

        print(f"{'White' if white else 'Black'} calibration values: {self.white_datum if white else self.black_datum}")

    def read_sensors(self):
        """Read and return discharge times for all sensors."""
        # 1. Charge all sensor capacitors (set pins HIGH for 10µs)
        for sensor in self.sensors:
            sensor.value(1)
        
        pyb.udelay(10)  # Wait 10µs

        # 2. Change all sensors to input mode
        self.sensors = [Pin(pin, Pin.IN) for pin in self.sensor_pins]  # Reinitialize as input

        start_time = ticks_us()
        discharge_times = [None] * len(self.sensors)

        while any(sensor.value() == 1 for sensor in self.sensors):
            elapsed_time = ticks_diff(ticks_us(), start_time)
            if elapsed_time > 4000:
                return [4000] * len(self.sensors)  # Timeout safety

            for i, sensor in enumerate(self.sensors):
                if sensor.value() == 0 and discharge_times[i] is None:
                    discharge_times[i] = elapsed_time

        # Replace None values with the timeout value
        discharge_times = [time if time is not None else 4000 for time in discharge_times]

        # Reset sensors to output mode to discharge them
        self.sensors = [Pin(pin, Pin.OUT_PP) for pin in self.sensor_pins]
        for sensor in self.sensors:
            sensor.value(0)  # Set pins LOW to discharge

        return discharge_times
    
    def normalize_readings(self, readings):
        """Normalize the readings to the range [0, 1]."""
        return [(reading - white) / (black - white) for reading, white, black in zip(readings, self.white_datum, self.black_datum)]

    @staticmethod
    def calculate_centroid(readings):
        """Calculate the centroid of the sensor readings."""
        weighted_sum = sum(i * reading for i, reading in enumerate(readings))
        total = sum(readings)
        return weighted_sum / total if total != 0 else len(readings) / 2