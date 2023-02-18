
import machine, time
from machine import Pin

# Define all of the pins
dir_pin = Pin(19, Pin.OUT)
x1_step_pin = Pin(13, Pin.OUT)
x2_step_pin = Pin(16, Pin.OUT)
y_step_pin = Pin(17, Pin.OUT)
z_step_pin = Pin(18, Pin.OUT)

valve_pin = Pin(22, Pin.OUT)
servo_pin = Pin(21, Pin.OUT)
servo = machine.PWM(servo_pin, freq=50)

flow_pin = Pin(23, Pin.IN)
moisture1_pin = Pin(34, Pin.IN)
moisture1_pin = Pin(39, Pin.IN)
moisture1_pin = Pin(36, Pin.IN)

xp_lim_pin = Pin(4, Pin.IN, Pin.PULL_UP)
xn_lim_pin = Pin(26, Pin.IN, Pin.PULL_UP)
yp_lim_pin = Pin(25, Pin.IN, Pin.PULL_UP)
yn_lim_pin = Pin(33, Pin.IN, Pin.PULL_UP)
zp_lim_pin = Pin(32, Pin.IN, Pin.PULL_UP)
zn_lim_pin = Pin(35, Pin.IN, Pin.PULL_UP)



def main():
    motor_system = MotorSystem()
    
    # Everything inside this will run forever
    while True:
        z_step_pin.on()
        time.sleep(0.005)
        z_step_pin.off()
        time.sleep(0.005)

class MotorSystem:
    '''Class designed to abstract away the problems with our motor set up. 
    
    The most important functions here are `set_target()` and `update()`. These functions set the target position for the head, and move the motors towards that target position, respectively'''
    X = 0
    Y = 1
    Z = 2
    def __init__(self):
        '''Initialize the motors'''
        pass

    def set_target(self, x, y, z):
        '''Set the target position that the gantry will go to in the future'''
        pass

    def set_max_velocity(self, max_velocity, direction):
        '''Set the maximum velocity in a given direction (0=X, 1=Y, 2=Z)'''
        pass

    def set_max_acceleration(self, max_acceleration, direction):
        '''Set the maximum velocity in a given direction (0=X, 1=Y, 2=Z)'''
        pass

    def distance_to_steps(self, distance, direction):
        '''Converts a distance in m to a number of steps in a given direction (X=0, Y=1, Z=2). See also `steps_to_distance`. '''
        pass

    def steps_to_distance(self, steps, direction):
        '''Converts a number of steps in a given direction (X=0, Y=1, Z=2) to distance in m. See also `steps_to_distance`. '''
        pass

    def calibrate(self):
        '''calibration subroutine - not sure how this will work'''
        pass

    def update(self):
        '''Should be called every clock cycle. This function will pulse the stepper motors needed to move the target position'''
        pass

#Reference: https://icircuit.net/micropython-controlling-servo-esp32-nodemcu/2385
def set_servo_pos(duty):
    '''Limits are 27 (dispense) and 65 (collect)'''
    if (20<=duty<=70):
        servo.duty(duty) 

if __name__=='__main__':
    main()