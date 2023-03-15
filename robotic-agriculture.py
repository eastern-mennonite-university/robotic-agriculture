
import machine, time, math
from machine import Pin

# List of things that still need done
# - [ ] Automated go to position with stepper motors
# - [ ] Solenoid valve control
# - [ ] Flow meter reading
# - [ ] Soil sensor readings
# - [ ] Seed dispenser control
# - [ ] User interface


# Define all of the pins
dir_pin = Pin(19, Pin.OUT)
x1_step_pin = Pin(13, Pin.OUT)
x2_step_pin = Pin(16, Pin.OUT)
y_step_pin = Pin(17, Pin.OUT)
z_step_pin = Pin(18, Pin.OUT)

valve_pin = Pin(22, Pin.OUT)
servo_pin = Pin(21, Pin.OUT)


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
    print('Script started')
    motor_system = MotorSystem()
    motor_system.set_position(0, 0, 0)

    seed_dispenser = SeedDispenser(servo_pin)

    frequency = 50
    # Everything inside this will run forever
    # serial = machine.UART(1, 115200, tx=1, rx=3)
    # serial.init(115200)
    motor_system.set_target(1000, 1000, 1000)
    at_start = True
    while True:
        motor_system.update()
        if motor_system.at_target():
            at_start = not at_start
            # Dispense a seed
            seed_dispenser.collect()
            time.sleep(1)
            seed_dispenser.dispense()
            time.sleep(1)
            seed_dispenser.collect()
            if at_start:
                motor_system.set_target(0, 0, 0)
            else:
                motor_system.set_target(1000, 1000, 1000)

        # if serial.any():
        #     try:
        #         message = serial.readline().decode('utf-8')
        #         frequency = float(message.strip())
        #     except:
        #         frequency = 20
        # motor_system.z_motor.set_velocity(frequency)
        # motor_system.z_motor.run_speed()
        pass

class MotorSystem:
    '''Class designed to abstract away the problems with our motor set up. 
    
    The most important functions here are `set_target()` and `update()`. These functions set the target position for the head, and move the motors towards that target position, respectively'''
    X = 0
    Y = 1
    Z = 2
    def __init__(self):
        self.x_motor = Motor(x1_step_pin, dir_pin)
        self.y_motor = Motor(y_step_pin, dir_pin)
        self.z_motor = Motor(z_step_pin, dir_pin)
        pass

    def set_position(self, x_pos: int, y_pos: int, z_pos: int):
        '''Set positions for all of the motors at once'''
        self.x_motor.set_position(x_pos)
        self.y_motor.set_position(y_pos)
        self.z_motor.set_position(z_pos)

    def set_target(self, x_tar: int, y_tar: int, z_tar: int):
        self.x_motor.set_target(x_tar)
        self.y_motor.set_target(y_tar)
        self.z_motor.set_target(z_tar)

    def update(self):
        self.x_motor.update()
        self.y_motor.update()
        self.z_motor.update()

    def at_target(self):
        '''Returns true if all of its motors are at the target position'''
        return all([motor.at_target() for motor in [self.x_motor, self.y_motor, self.z_motor]])

class Motor:
    def __init__(self, step_pin: machine.Pin, dir_pin: machine.Pin, position: int=0, max_velocity: float=.1, max_acceleration: float=.05, min_pulse_width: int=3):
        '''Initialize the motor. Run once at start.'''
        self.min_pulse_width = min_pulse_width # Minimum width of pulse in microseconds. Minimum for DRV8825 is 1.9us, 3 gives us some wiggle room
        self.set_position(position)
        self.set_target(position)
        self.set_velocity(0)
        self.set_max_velocity(max_velocity)
        self.set_max_acceleration(max_acceleration)
        self.previous_step_ticks = 0
        self.set_pins(step_pin, dir_pin)

    def set_pins(self, step: machine.Pin, dir: machine.Pin):
        '''Set the pins to be used for stepping the motor. Call once.'''
        self.step_pin = step
        self.dir_pin = dir

    def update(self):
        '''Should be called every clock cycle. This function will pulse the stepper motor needed to move the target position'''
        self.update_vel()
        self.run_speed()

    def update_vel(self):
        '''Is called by update(). Changes the stepper velocity at the rate of `self.max_accel`. This function makes sure that we don't accelerate or decelerate too quickly'''
        # TODO: Update this to use a ramp function, rather than turning off/on
        distance_to_go = self.target - self.position
        # steps_to_stop = self.velocity**2 / (2*self.max_accel)

        # This basic code says to just run at maximum velocity until we hit our
        # target position. It should be replaced.
        if distance_to_go == 0:
            self.set_velocity(0)
        elif distance_to_go > 0:
            self.set_velocity(self.max_vel)
        else:
            self.set_velocity(-self.max_vel)
        

    def set_target(self, tar: int):
        '''Set the target position that the motor will go to in the future'''
        self.target = tar

    def set_position(self, pos: int):
        '''Set the current position of the motor. Note that this is *not* the target position'''
        self.position = pos

    def set_velocity(self, v: float):
        '''Set the current velocity in steps/sec'''
        self.velocity = v
        # Set the interval between steps in microseconds 
        self.step_interval = abs(math.inf if v==0 else int(1_000_000 * 1/v))

    def set_max_velocity(self, max_velocity: float):
        '''Set the maximum velocity. Only needs to be called once.'''
        self.max_vel = max_velocity

    def set_max_acceleration(self, max_acceleration: float):
        '''Set the maximum velocity in a given direction (0=X, 1=Y, 2=Z, None=All directions)'''
        self.max_accel = max_acceleration

    def run_speed(self):
        '''Checks the timers for the motor and does a `step()` if needed.'''
        current_time = time.ticks_us()
        # print(current_time)
        if time.ticks_diff(current_time, self.previous_step_ticks) >= self.step_interval:
            self.step(reverse=(self.velocity<0))
            # This equation is confusing because we need to make sure our previous
            # step is at most `step_interval` us behind the last step
            self.previous_step_ticks = \
            time.ticks_add(self.previous_step_ticks, math.floor(time.ticks_diff(current_time, self.previous_step_ticks)/self.step_interval)*self.step_interval)

    def step(self, reverse: bool=False):
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

    def at_target(self):
        '''Returns true if the motor is at its target position'''
        return self.target==self.position

    def distance_to_steps(self, distance: float):
        '''Converts a distance in m to a number of steps. See also `steps_to_distance`. '''
        pass

    def steps_to_distance(self, steps: int):
        '''Converts a number of steps to distance in m. See also `steps_to_distance`. '''
        pass

    def calibrate(self):
        '''calibration subroutine - not sure how this will work'''
        pass
        
class SeedDispenser:
    DISPENSE_DUTY = 27
    COLLECT_DUTY = 65
    def __init___(self, servo_pin):
        self.pos = 0
        self.servo = machine.PWM(servo_pin, freq=50)

    def dispense(self):
        self.servo.duty(SeedDispenser.DISPENSE_DUTY)

    def collect(self):
        self.servo.duty(SeedDispenser.COLLECT_DUTY)


if __name__=='__main__':
    main()