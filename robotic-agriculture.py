
import machine, time, math
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
    print('Hello, world!')
    motor_system = MotorSystem()
    motor_system.set_max_acceleration(.1)
    motor_system.set_max_velocity(1)
    
    # Everything inside this will run forever
    motor_system.set_velocity(0, 0, 440)
    change = .01
    speed = 10
    while True:
        motor_system.run_speed()
        speed += change
        if speed > 500: change = -.002
        if speed < 10: change = .002
        motor_system.set_velocity(0, 0, speed)
        set_servo_pos(int(speed/6))

class MotorSystem:
    '''Class designed to abstract away the problems with our motor set up. 
    
    The most important functions here are `set_target()` and `update()`. These functions set the target position for the head, and move the motors towards that target position, respectively'''
    X = 0
    Y = 1
    Z = 2
    def __init__(self):
        self.x_motor = Motor(x1_step_pin, dir_pin)
        self.y_motor = Motor(x1_step_pin, dir_pin)
        self.z_motor = Motor(x1_step_pin, dir_pin)
        pass
class Motor:
    def __init__(self, step_pin, dir_pin):
        '''Initialize the motor'''
        self.min_pulse_width = 3 # Minimum width of pulse in microseconds. Minimum for DRV8825 is 1.9us, 3 gives us some wiggle room
        self.set_position(0)
        self.set_velocity(0)
        self.set_target(0)
        self.previous_step_ticks = 0
        self.set_pin(step_pin, dir_pin)
        pass

    def set_pin(self, step, dir):
        '''Set the pins to be used for stepping the motor.'''
        self.step_pin = step
        self.dir_pin = dir

    def update(self):
        '''Should be called every clock cycle. This function will pulse the stepper motor needed to move the target position'''
        self.update_vel()

    def update_vel(self):
        '''Is called by update(). Changes the stepper velocity at the rate of `self.max_accel`. This function makes sure that we don't accelerate or decelerate too quickly'''
        pass

    def set_target(self, tar):
        '''Set the target position that the motor will go to in the future'''
        self.target = tar

    def set_position(self, pos):
        '''Set the current position of the motor. Note that this is *not* the target position'''
        self.position = pos

    def set_velocity(self, v):
        '''Set the current velocity in steps/sec'''
        self.velocity = v
        # Set the interval between steps in microseconds 
        self.step_interval = (math.inf if v==0 else int(1_000_000 * 1/v))

    def set_max_velocity(self, max_velocity):
        '''Set the maximum velocity. Only needs to be called once.'''
        self.max_vel = max_velocity

    def set_max_acceleration(self, max_acceleration):
        '''Set the maximum velocity in a given direction (0=X, 1=Y, 2=Z, None=All directions)'''
        self.max_accel = max_acceleration

    def run_speed(self):
        '''Checks the timers for the motor and does a `step()` if needed.'''
        current_time = time.ticks_us()
        # print(current_time)
        if time.ticks_diff(current_time, self.previous_step_ticks) >= self.step_interval:
            self.step(False) # Todo: add functionality for reverse here
            self.previous_step_ticks = time.ticks_add(self.previous_step_ticks, self.step_interval)

    def step(self, reverse=False):
        '''Perform one step. If `reverse` is set, go the opposite way. Uses `self.min_pulse_width` to calculate length of pulse'''
        # First, set the direction pin
        if (reverse): 
            self.dir_pin.on()
        else:
            self.dir_pin.off()

        # Then, pulse the desired pin as quickly as possible
        self.step_pin.on()
        time.sleep_us(self.min_pulse_width)
        self.step_pin.off()

    def distance_to_steps(self, distance, direction):
        '''Converts a distance in m to a number of steps in a given direction (X=0, Y=1, Z=2). See also `steps_to_distance`. '''
        pass

    def steps_to_distance(self, steps, direction):
        '''Converts a number of steps in a given direction (X=0, Y=1, Z=2) to distance in m. See also `steps_to_distance`. '''
        pass

    def calibrate(self):
        '''calibration subroutine - not sure how this will work'''
        pass
        
#Reference: https://icircuit.net/micropython-controlling-servo-esp32-nodemcu/2385
def set_servo_pos(duty):
    '''Limits are 27 (dispense) and 65 (collect)'''
    if (20<=duty<=70):
        servo.duty(duty) 

if __name__=='__main__':
    main()