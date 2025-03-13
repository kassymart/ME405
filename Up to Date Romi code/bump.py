from pyb import Pin, Timer

class BumpSensor:
    def __init__(self, pin, debounce_time=50):
        self.pin = Pin(pin, Pin.IN, Pin.PULL_UP)
        self.debounce_timer = Timer(2, prescaler=83, period=0xFFFF)  # Timer 2 @ 1MHz
        self.last_press_time = 0
        self.debounce_ticks = debounce_time * 1000  # Convert ms to μs
        self.last_state = False
        self.pressed = False
        
    def update(self):
        """Call regularly to update pressed state with debouncing"""
        current_state = self.pin.value() == 0
        now = self.debounce_timer.counter()
        
        if current_state != self.last_state:
            self.last_press_time = now
            
        elapsed = now - self.last_press_time
        if elapsed < 0:  # Handle timer overflow
            elapsed += 0xFFFF
            
        if elapsed > self.debounce_ticks:
            self.pressed = current_state
            
        self.last_state = current_state
        return self.pressed