
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
    motor_system = MotorSystem()
    motor_system.set_max_acceleration(.1)
    motor_system.set_max_velocity(1)
    
    # Everything inside this will run forever
    while True:
        motor_system.step(MotorSystem.Z, False)
        time.sleep(1)

class MotorSystem:
    '''Class designed to abstract away the problems with our motor set up. 
    
    The most important functions here are `set_target()` and `update()`. These functions set the target position for the head, and move the motors towards that target position, respectively'''
    X = 0
    Y = 1
    Z = 2
    def __init__(self):
        '''Initialize the motors'''
        self.min_pulse_width = 3 # Minimum width of pulse in microseconds. Minimum for DRV8825 is 1.9us, 3 gives us some wiggle room
        self.set_position(0, 0, 0)
        self.set_velocity(0, 0, 0)
        self.set_target(0, 0, 0)
        self.set_pins(x1_step_pin, y_step_pin, z_step_pin, dir_pin)

    def set_target(self, x, y, z):
        '''Set the target position that the gantry will go to in the future'''
        self.x_tar = x
        self.y_tar = y
        self.z_tar = z

    def set_position(self, x, y, z):
        '''Set the current position of the system. Note that this is *not* the target position'''
        self.x_pos = x
        self.y_pos = y
        self.z_pos = z

    def set_velocity(self, xv, yv, zv):
        '''Set the current velocities in steps/sec'''
        self.x_vel = xv
        self.y_vel = yv
        self.z_vel = zv
        # Set the interval between steps in microseconds 
        self.x_step_interval = (math.inf if xv==0 else int(1_000_000 * 1/xv))
        self.y_step_interval = (math.inf if yv==0 else int(1_000_000 * 1/yv))
        self.z_step_interval = (math.inf if zv==0 else int(1_000_000 * 1/zv))

    def set_max_velocity(self, max_velocity, direction=None):
        '''Set the maximum velocity in a given direction (0=X, 1=Y, 2=Z, None=All directions). Only needs to be called once.'''
        if direction in (MotorSystem.X, None): self.max_x_vel = max_velocity
        if direction in (MotorSystem.Y, None): self.max_y_vel = max_velocity
        if direction in (MotorSystem.Z, None): self.max_z_vel = max_velocity

    def set_max_acceleration(self, max_acceleration, direction=None):
        '''Set the maximum velocity in a given direction (0=X, 1=Y, 2=Z, None=All directions)'''
        if direction in (MotorSystem.X, None): self.max_x_accel = max_acceleration
        if direction in (MotorSystem.Y, None): self.max_y_accel = max_acceleration
        if direction in (MotorSystem.Z, None): self.max_z_accel = max_acceleration
        pass

    def set_pins(self, x, y, z, dir):
        '''Set the pins to be used for stepping the motors.'''
        self.x_pin = x
        self.y_pin = y
        self.z_pin = z
        self.dir_pin = dir

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
        self.update_vel()

    def update_vel(self):
        '''Is called by update(). Changes the stepper velocities at the rate of `self.max_accel`. This function makes sure that we don't accelerate or decelerate too quickly'''
        pass

    def step(self, direction, reverse=False):
        '''Perform one step in the given direction. If `reverse` is set, go the opposite way. Uses `self.min_pulse_width` to calculate length of pulse'''
        # First, set the direction pin
        if (reverse): 
            self.dir_pin.on()
        else:
            self.dir_pin.off()

        # Then, pulse the desired pin as quickly as possible
        pulse_pin = None
        if direction == MotorSystem.X: pulse_pin = self.x_pin
        elif direction == MotorSystem.Y: pulse_pin = self.y_pin
        elif direction == MotorSystem.Z: pulse_pin = self.z_pin
        if pulse_pin:
            pulse_pin.on()
            time.sleep_us(self.min_pulse_width)
            pulse_pin.off()

    def run_speed(self):
        '''Checks the timers for each motor and does a `step()` if needed.'''
        



#Reference: https://icircuit.net/micropython-controlling-servo-esp32-nodemcu/2385
def set_servo_pos(duty):
    '''Limits are 27 (dispense) and 65 (collect)'''
    if (20<=duty<=70):
        servo.duty(duty) 

if __name__=='__main__':
    main()