
import machine, time, math
from machine import Pin

# List of things that still need done
# - [X~] Automated go to position with stepper motors
#    - [X] Able to go to specified position
#    - [ ] Ramp velocity for stepper motors
#    - [ ] Limit switches and calibration
# - [X] Solenoid valve control
# - [X] Flow meter reading
# - [ ] Soil sensor readings
# - [X] Seed dispenser control
# - [ ] User interface
# - [ ] Program States
#    - [ ] Idle State
#    - [ ] Calibration State
#    - [ ] Watering State
#    - [ ] Planting State


# Define all of the pins
dir_pin = Pin(19, Pin.OUT)
x1_step_pin = Pin(13, Pin.OUT)
x2_step_pin = Pin(16, Pin.OUT)
y_step_pin = Pin(17, Pin.OUT)
z_step_pin = Pin(18, Pin.OUT)

valve_pin = Pin(22, Pin.OUT)
servo_pin = Pin(21, Pin.OUT)

# Water system pins
flow_pin = Pin(23, Pin.IN, Pin.PULL_DOWN)
moisture1_pin = Pin(34, Pin.IN)
moisture2_pin = Pin(39, Pin.IN)
moisture3_pin = Pin(36, Pin.IN)

# Limit switch pins
xp_lim_pin = Pin(4, Pin.IN, Pin.PULL_UP)
xn_lim_pin = Pin(26, Pin.IN, Pin.PULL_UP)
yp_lim_pin = Pin(25, Pin.IN, Pin.PULL_UP)
yn_lim_pin = Pin(33, Pin.IN, Pin.PULL_UP)
zp_lim_pin = Pin(32, Pin.IN, Pin.PULL_UP)
zn_lim_pin = Pin(35, Pin.IN, Pin.PULL_UP)

def handle_interrupt(pin):
    pass

def main():
    print('Script started')
    motor_system = MotorSystem()

    seed_dispenser = SeedDispenser(servo_pin)
    water_system = WaterSystem(valve_pin, flow_pin)

    # Set up handlers for limit switch interrupts
    xp_lim_pin.irq(trigger=Pin.IRQ_FALLING, handler=motor_system.xp_limit_hit)
    xn_lim_pin.irq(trigger=Pin.IRQ_FALLING, handler=motor_system.xn_limit_hit)
    yp_lim_pin.irq(trigger=Pin.IRQ_FALLING, handler=motor_system.yp_limit_hit)
    yn_lim_pin.irq(trigger=Pin.IRQ_FALLING, handler=motor_system.yn_limit_hit)
    # zp_lim_pin.irq(trigger=Pin.IRQ_FALLING, handler=motor_system.zp_limit_hit)
    # zn_lim_pin.irq(trigger=Pin.IRQ_FALLING, handler=motor_system.zn_limit_hit)

    flow_pin.irq(trigger=Pin.IRQ_RISING, handler=water_system.water_pulse) 

    frequency = 50
    # Everything inside this will run forever
    # serial = machine.UART(1, 115200, tx=1, rx=3)
    # serial.init(115200)
    motor_system.set_target(1000, 2000, 1000)
    at_start = True
    while True:
        motor_system.update()
        time.sleep(1)
        if motor_system.at_target():
            print('reached target')
            # Dispense a seed
            seed_dispenser.collect()
            time.sleep(1)
            seed_dispenser.dispense()
            time.sleep(1)
            seed_dispenser.collect()
            time.sleep(1)
            if at_start:
                motor_system.set_target(0, 0, 0)
            else:
                motor_system.set_target(1000, 2000, 1000)
            at_start = not at_start

        # if serial.any():
        #     try:
        #         message = serial.readline().decode('utf-8')
        #         frequency = float(message.strip())
        #     except:
        #         frequency = 20
        # motor_system.z_motor.set_velocity(frequency)
        # motor_system.z_motor.run_speed()

class MotorSystem:
    '''Class designed to abstract away the problems with our motor set up. 
    
    The most important functions here are `set_target()` and `update()`. These functions set the target position for the head, and move the motors towards that target position, respectively'''
    X = 0
    Y = 1
    Z = 2
    # TODO: Make these limits adjustable
    X_MIN = 0
    X_MAX = 1000
    Y_MIN = 0
    Y_MAX = 1000
    Z_MIN = 0
    Z_MAX = 1000
    def __init__(self):
        self.x_motor = Motor(x1_step_pin, dir_pin, max_position=MotorSystem.X_MAX)
        self.y_motor = Motor(y_step_pin, dir_pin, max_position=MotorSystem.Y_MAX)
        self.z_motor = Motor(z_step_pin, dir_pin, max_position=MotorSystem.Z_MAX)
        self.set_position(0, 0, 0)

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

    # Handlers for limit switch interrupts
    # Will update the positions of the motors
    # TODO: Maybe add more details here?
    def xp_limit_hit(self, _pin):
        self.x_motor.set_position(self.x_motor.max_position)
    def xn_limit_hit(self, _pin):
        self.x_motor.set_position(0)
    def yp_limit_hit(self, _pin):
        self.y_motor.set_position(self.y_motor.max_position)
    def yn_limit_hit(self, _pin):
        self.y_motor.set_position(0)
    def zp_limit_hit(self, _pin):
        self.z_motor.set_position(self.z_motor.max_position)
        print('zp limit')
    # TODO: This pin doesn't have an internal pullup resistor, so it
    # doesn't work right yet
    def zn_limit_hit(self, _pin):
        self.z_motor.set_position(0)
        print('zn limit')


    def at_target(self):
        '''Returns true if all of its motors are at the target position'''
        return all([motor.at_target() for motor in [self.x_motor, self.y_motor, self.z_motor]])

class Motor:
    def __init__(self, step_pin: machine.Pin, dir_pin: machine.Pin, position: int=0, max_velocity: float=200, max_acceleration: float=100, min_pulse_width: int=3, max_position: int=1000):
        '''Initialize the motor. Run once at start.'''
        self.max_position = max_position
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
        self.target = max(min(tar, self.max_position), 0)

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
        # print('runspeed')
        current_time = time.ticks_us()
        # print(current_time)
        if time.ticks_diff(current_time, self.previous_step_ticks) >= self.step_interval:
            self.step(reverse=(self.velocity<0))
            # This equation is confusing because we need to make sure our previous
            # step is at most `step_interval` us behind the last step
            # self.previous_step_ticks = current_time
            self.previous_step_ticks = \
            time.ticks_add(self.previous_step_ticks, math.floor(time.ticks_diff(current_time, self.previous_step_ticks)/self.step_interval)*self.step_interval)

    def step(self, reverse: bool=False):
        '''Perform one step. If `reverse` is set, go the opposite way. Uses `self.min_pulse_width` to calculate length of pulse'''
        # First, set the direction pin
        # print(self.position)
        if (reverse): 
            self.dir_pin.on()
            self.position -= 1
        else:
            self.dir_pin.off()
            self.position += 1

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
    # These denote the duty cycles of the collect and dispense positions
    # of the servo motor
    DISPENSE_DUTY = 27
    COLLECT_DUTY = 65
    def __init__(self, servo_pin):
        self.pos = 0
        self.servo = machine.PWM(servo_pin, freq=50)
        self.collect()

    def dispense(self):
        '''Moves servo motor to dispense position'''
        self.servo.duty(SeedDispenser.DISPENSE_DUTY)

    def collect(self):
        '''Moves servo motor to collect position'''
        self.servo.duty(SeedDispenser.COLLECT_DUTY)

class WaterSystem:
    '''Class representing the solenoid valve and the flow meter'''
    # From flow meter datasheet: f(requency) = 11*Q, where Q=L/min
    # So ratio is 660 pulses/Liter
    # or 1.515 mL per pulse
    FLOW_RATE = 1.515
    def __init__(self, valve_pin, flow_pin):
        self.valve_pin = valve_pin
        self.flow_pin = flow_pin
        self.flow = 0
        self.dispense_target_ml = 0
        self.last_pulse = time.ticks_ms()

    def update(self):
        '''Opens/closes valve based on what is needed'''
        if self.flow < self.dispense_target_ml:
            self.valve_pin.on()
        else:
            self.valve_pin.off()
            self.flow = 0
            self.dispense_target_ml = 0

    def dispense(self, dispense_target_ml):
        '''Set the target amount of water to dispense, in mL'''
        self.dispense_target_ml = dispense_target_ml

    def water_pulse(self, _pin):
        '''Called by interrupt - increases flow by set rate'''
        current_ticks_ms = time.ticks_ms()
        ms_diff = time.ticks_diff(current_ticks_ms, self.last_pulse)
        # Debounce signal (signal will never be above 400Hz)
        if ms_diff >= 2:
            self.flow += WaterSystem.FLOW_RATE
            self.last_pulse = current_ticks_ms
            # print(f'{self.flow} mL')
    
class UserInterface:
    '''Class designed to get user input. TODO: implementation'''
    def __init__(self) -> None:
        pass

    def get_input(self):
        return None

# --------------------------------------------------------------------------
# Program States

class ProgramState:
    '''General class representing a state. For any subclass of ProgramState,
    you can pass an instance of a state to create a new state. Each state has a
    run() function that should be called on every program cycle that does whatever 
    it is supposed to do. run() should always return a subclass of ProgramState'''
    motor_system: MotorSystem
    water_system: WaterSystem
    dispenser: SeedDispenser
    user_interface: UserInterface
    last_calibration: float

    def __init__(self, previous_state: 'ProgramState'=None):
        if previous_state:
            self.motor_system = previous_state.motor_system
            self.water_system = previous_state.water_system
            self.dispenser = previous_state.dispenser
            self.user_interface = previous_state.user_interface
        else:
            self.motor_system = MotorSystem()
            self.water_system = WaterSystem(valve_pin, flow_pin)
            self.dispenser = SeedDispenser(servo_pin)
            self.user_interface = UserInterface()

    def run(self) -> 'ProgramState':
        '''Called every program cycle, ideally'''
        raise NotImplementedError
    
class IdleState(ProgramState):
    '''State representing when the machine is doing nothing'''
    def run(self):
        input = self.user_interface.get_input()
        if input is None:
            time.sleep(1)
            return self
        else:
            return WateringState(self)

class CalibrationState(ProgramState):
    x_cal_dir: str
    y_cal_dir: str
    z_cal_dir: str
    min_x: int
    min_y: int
    min_z: int
    max_x: int
    max_y: int
    max_z: int
    def __init__(self, previous_state: 'ProgramState' = None):
        super().__init__(previous_state)
        self.x_cal_dir = 'positive'
        self.y_cal_dir = 'positive'
        self.z_cal_dir = 'positive'
    def run(self):
        # Check for limit switch hits (remember, False=hit)
        if self.x_cal_dir=='positive' and xp_lim_pin.value() == False:
            self.x_cal_dir = 'negative'
            self.motor_system.x_motor.max_position = self.motor_system.x_motor.position
        if self.y_cal_dir=='positive' and yp_lim_pin.value() == False:
            self.y_cal_dir = 'negative'
            self.motor_system.y_motor.max_position = self.motor_system.y_motor.position
        if self.z_cal_dir=='positive' and zp_lim_pin.value() == False:
            self.z_cal_dir = 'negative'
            self.motor_system.z_motor.max_position = self.motor_system.z_motor.position

        if self.x_cal_dir=='negative' and xn_lim_pin.value() == False:
            self.x_cal_dir = 'done'
            self.motor_system.x_motor.min_position = self.motor_system.x_motor.position
        if self.y_cal_dir=='negative' and yn_lim_pin.value() == False:
            self.y_cal_dir = 'done'
            self.motor_system.y_motor.min_position = self.motor_system.y_motor.position
        if self.z_cal_dir=='negative' and zn_lim_pin.value() == False:
            self.z_cal_dir = 'done'
            self.motor_system.z_motor.min_position = self.motor_system.z_motor.position

        # Check which point we are at for each of the stepper motors, and set the
        # velocities appropriately
        # TODO: this needs to update motor_system with our bounds
        match self.x_cal_dir:
            case 'positive':
                self.motor_system.x_motor.set_velocity(self.motor_system.x_motor.max_vel)
            case 'negative':
                self.motor_system.x_motor.set_velocity(-self.motor_system.x_motor.max_vel)
            case 'done':
                self.motor_system.x_motor.set_velocity(0)
        match self.y_cal_dir:
            case 'positive':
                self.motor_system.y_motor.set_velocity(self.motor_system.y_motor.max_vel)
            case 'negative':
                self.motor_system.y_motor.set_velocity(-self.motor_system.y_motor.max_vel)
            case 'done':
                self.motor_system.y_motor.set_velocity(0)
        match self.z_cal_dir:
            case 'positive':
                self.motor_system.z_motor.set_velocity(self.motor_system.z_motor.max_vel)
            case 'negative':
                self.motor_system.z_motor.set_velocity(-self.motor_system.z_motor.max_vel)
            case 'done':
                self.motor_system.z_motor.set_velocity(0)

        # Once all the speeds are set, call our run_speed function.
        # Normally we would use motor_system.update(), but this is an exception,
        # because we need to go outside of normal behavior to calibrate
        self.motor_system.x_motor.run_speed()
        self.motor_system.y_motor.run_speed()
        self.motor_system.z_motor.run_speed()

        # If we are done calibrating, we need to calibrate the motors 
        # and return an IdleState
        if self.x_cal_dir=='done' and self.y_cal_dir=='done' and self.z_cal_dir=='done':
            self.last_calibration = time.time()
            return IdleState(self)
        else:
            return self


class WateringState(ProgramState):
    pass

class PlantingState(ProgramState):
    pass

if __name__=='__main__':
    main()