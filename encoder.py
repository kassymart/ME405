import pyb
from pyb import Timer, Pin
from time import ticks_us, ticks_diff
import math

class Encoder:
    '''A quadrature encoder decoding interface encapsulated in a Python class'''
    def __init__(self, timer, pin1, pin2):
        self.timer = timer
        self.timer.channel(1, mode=Timer.ENC_AB, pin=pin1)
        self.timer.channel(2, mode=Timer.ENC_AB, pin=pin2)
        self.position = 0
        self.prev_count = self.timer.counter()
        self.delta = 0
        self.dt = 0
        self.prev_time = ticks_us()
    
    def update(self):
        '''Runs one update step on the encoder's timer counter to keep
           track of the change in count and check for counter reload'''
        update_time = ticks_us()
        count = self.timer.counter() # Read the timer counter
        self.dt = ticks_diff(update_time, self.prev_time) # Calculate the change in time
        self.delta = count - self.prev_count # Calculate the change in count
        if self.delta > 32768: # Check for counter reload
            self.delta -= 65536
        elif self.delta < -32768: # Check for counter underflow
            self.delta += 65536
        self.position += self.delta # Update the position
        self.prev_time = update_time # Update the previous time  
        self.prev_count = count # Update the previous count
        
        return update_time / 1_000_000 # Convert microseconds to seconds

    def get_position(self):
        '''Returns the most recently updated value of position as determined
           within the update() method'''
        return self.position * 2 * math.pi / 1440
    
    def get_velocity(self):
        '''Returns a measure of velocity using the the most recently updated
           value of delta as determined within the update() method'''
        if self.dt == 0:
            return 0
        return self.delta * 2 * math.pi / 1440 / (self.dt / 1_000_000) # Convert microseconds to seconds

    def reset(self):
        '''Resets the encoder state to ensure correct initial values'''
        self.position = 0
        self.prev_count = self.timer.counter()
        self.delta = 0
        self.dt = 0
        self.prev_time = ticks_us()