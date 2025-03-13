from pyb import Pin
import time

class HCSR04:
    def __init__(self, trigger_pin, echo_pin, timeout):
        """
        Initializes the HC-SR04 sensor.
        :param trigger_pin: GPIO pin connected to the Trigger pin of the sensor.
        :param echo_pin: GPIO pin connected to the Echo pin of the sensor.
        :param timeout: Maximum waiting time for the echo response (in microseconds).
        """
        self.trigger = Pin(trigger_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        self.timeout = timeout
    
    def get_distance(self):
        """
        Measures the distance using the HC-SR04 sensor.
        :return: Distance in centimeters (or -1 if timeout occurs).
        """
        self.trigger.off()
        time.sleep_us(2)
        self.trigger.on()
        time.sleep_us(10)
        self.trigger.off()
        
        # Wait for echo to go HIGH (signal start)
        start_time = time.ticks_us()
        timeout_start = start_time
        while self.echo.value() == 0:
            if time.ticks_diff(time.ticks_us(), timeout_start) > self.timeout:
                return -1  # Timeout waiting for echo to start
        
        # Measure the HIGH duration (time of flight)
        start_time = time.ticks_us()
        while self.echo.value() == 1:
            if time.ticks_diff(time.ticks_us(), start_time) > self.timeout:
                return -1  # Timeout while measuring
        
        end_time = time.ticks_us()
        duration = time.ticks_diff(end_time, start_time)
        
        # Convert time to distance (Speed of sound = 343 m/s or 0.0343 cm/μs)
        distance = (duration * 0.0343) / 2  # Divide by 2 for round trip
        
        if distance < 2 or distance > 400:  # Ignore out-of-range values
            return -1
        
        return round(distance, 2)  # Round to 2 decimal places
