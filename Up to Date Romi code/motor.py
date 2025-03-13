from pyb import Pin, Timer

class Motor:
    '''A motor driver interface encapsulated in a Python class. Works with
    motor drivers using separate PWM and direction inputs such as the DRV8838
    drivers present on the Romi chassis from Pololu.'''

    def __init__(self, PWM_pin, DIR_pin, EN_pin, timer_num, channel):
        '''Initializes a Motor object'''
        self.PWM_pin = Pin(PWM_pin)
        self.DIR_pin = Pin(DIR_pin, mode=Pin.OUT_PP)
        self.EN_pin = Pin(EN_pin, mode=Pin.OUT_PP, value=0)
        
        self.timer = Timer(timer_num, freq=20000)  # 20 kHz frequency
        self.channel = self.timer.channel(channel, Timer.PWM, pin=self.PWM_pin)

    def set_effort(self, effort):
        '''Sets the present effort requested from the motor based on an input value
            between -100 and 100'''
        if effort < 0 and effort >= -100:
            self.DIR_pin.high()
        elif effort >= 0 and effort <= 100:
            self.DIR_pin.low()
        else: 
            effort = 0
            print("Invalid effort value. Effort must be between -100 and 100.")

        self.channel.pulse_width_percent(abs(effort))

    def enable(self):
        '''Enables the motor driver by taking it out of sleep mode into brake mode'''
        self.EN_pin.high()

    def disable(self):
        '''Disables the motor driver by taking it into sleep mode'''
        self.EN_pin.low()
