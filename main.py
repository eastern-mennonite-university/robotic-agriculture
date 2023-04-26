
import machine, time, math, json
from machine import Pin
import network
# from umqttsimple import MQTTClient
# import ubinascii

# List of things that still need done
# - [X~] Automated go to position with stepper motors
#    - [X] Able to go to specified position
#    - [ ] Ramp velocity for stepper motors
#    - [X] Limit switches and calibration
# - [X] Solenoid valve control
# - [X] Flow meter reading
# - [ ] Soil sensor readings
# - [X] Seed dispenser control
# - [X] User interface
# - [ ] Program States
#    - [X] Idle State
#    - [X] Calibration State
#    - [ ] Watering State
#    - [X] Planting State


# Define all of the pins
dir_pin = Pin(19, Pin.OUT)
x1_step_pin = Pin(13, Pin.OUT)
x2_step_pin = Pin(16, Pin.OUT)
y_step_pin = Pin(17, Pin.OUT)
z_step_pin = Pin(18, Pin.OUT)

valve_pin = Pin(22, Pin.OUT)
servo_pin = Pin(21, Pin.OUT)

# Water system pins
flow_pin = Pin(23, Pin.IN)
moisture1_pin = Pin(34)
moisture2_pin = Pin(39)
moisture3_pin = Pin(36)

# Limit switch pins
xp_lim_pin = Pin(4, Pin.IN, Pin.PULL_UP)
xn_lim_pin = Pin(26, Pin.IN, Pin.PULL_UP)
yp_lim_pin = Pin(25, Pin.IN, Pin.PULL_UP)
yn_lim_pin = Pin(33, Pin.IN, Pin.PULL_UP)
zp_lim_pin = Pin(32, Pin.IN, Pin.PULL_UP)
zn_lim_pin = Pin(35, Pin.IN, Pin.PULL_UP)
limit_pins = [xp_lim_pin,xn_lim_pin,yp_lim_pin,yn_lim_pin,zp_lim_pin,zn_lim_pin]

current_state = None
# uart = machine.UART(1, 115200, tx=1, rx=3)

sta_if = None 
def main():
    global current_state
    print('Script started')
    
    current_state = IdleState()
    motor_system = current_state.motor_system
    seed_dispenser = current_state.dispenser
    water_system = current_state.water_system

    # Set up handlers for limit switch interrupts
    for pin in limit_pins:
        pin.irq(trigger=Pin.IRQ_FALLING, handler=motor_system.limit_handler)
    flow_pin.irq(trigger=Pin.IRQ_RISING, handler=water_system.water_pulse) 

    sta_if = do_connect()

    last_update = time.time()

    while True:
        cmd = current_state.user_interface.get_input_line()
        if cmd is not None:
            if cmd == 'p down':
                current_state.user_interface.output(str(water_system.total_flow))
                if not water_system.dispensing:
                    water_system.dispense(100)
            elif cmd == 'q up':
                water_system.dispense(0)
            elif cmd == 'f down':
                seed_dispenser.dispense()
            elif cmd == 'f up':
                seed_dispenser.collect()
            elif cmd == 'm down':
                current_state.user_interface.output(str(current_state.moisture_system.get_moisture()))
            elif cmd == '1 down':
                current_state = IdleState(current_state)
                current_state.user_interface.output('switched to idle \n')
            elif cmd == '2 down':
                current_state = CalibrationState(current_state)
                current_state.user_interface.output('starting calibration \n')
            elif cmd == '3 down':
                current_state = WateringState(current_state)
                current_state.user_interface.output('starting watering routine \n')
            elif cmd == '4 down':
                current_state.user_interface.output('starting planting routine \n')
                current_state = PlantingState(current_state, 6, 3, 0, 2)

        current_state = current_state.run()
        water_system.update()

        # if not sta_if.isconnected():
        #     current_state.user_interface.output('Disconnected. Reconnecting...')
        #     sta_if.disconnect()
        #     sta_if = do_connect()

        # if (time.time() - last_update) >= 15:
        #     bot_data_string = json.dumps(current_state.bot_data())
        #     current_state.user_interface.mqtt_client.publish('emuagrobot22802/botdata', bot_data_string)
        #     last_update = time.time()



# Copied from Micropython documentation
# https://docs.micropython.org/en/latest/esp8266/tutorial/network_basics.html
def do_connect():
    global current_state
    current_state.user_interface.output('connecting')
    sta_if = network.WLAN(network.STA_IF)
    current_state.user_interface.output(str(sta_if.config('mac')))
    if not sta_if.isconnected():
        current_state.user_interface.output('connecting to network...')
        sta_if.active(True)
        sta_if.connect('EMU-iot', '3BirdsOrCats')
        while not sta_if.isconnected():
            pass
    current_state.user_interface.output('network config:', sta_if.ifconfig())
    return sta_if




def restart_and_reconnect():
  current_state.user_interface.output('Failed to connect to MQTT broker. Reconnecting...')
  time.sleep(10)
  machine.reset()



class MotorSystem:
    '''Class designed to abstract away the problems with our motor set up. 
    
    The most important functions here are `set_target()` and `update()`. These functions set the target position for the head, and move the motors towards that target position, respectively'''
    global current_state
    last_lim: int
    def __init__(self):
        self.x_motor = DoubleMotor(x1_step_pin, x2_step_pin, dir_pin, max_position=10000, min_position=0)
        self.y_motor = Motor(y_step_pin, dir_pin, max_position=10000, min_position=0)
        self.z_motor = Motor(z_step_pin, dir_pin, max_position=10000, min_position=0)
        self.set_position(0, 0, 0)
        self.last_lim = time.ticks_us()

    def set_position(self, x_pos: int=None, y_pos: int=None, z_pos: int=None):
        '''Set positions for all of the motors at once'''
        if x_pos is not None: self.x_motor.set_position(x_pos)
        if y_pos is not None: self.y_motor.set_position(y_pos)
        if z_pos is not None: self.z_motor.set_position(z_pos)

    def set_target(self, x_tar: int=None, y_tar: int=None, z_tar: int=None):
        if x_tar is not None: self.x_motor.set_target(x_tar)
        if y_tar is not None: self.y_motor.set_target(y_tar)
        if z_tar is not None: self.z_motor.set_target(z_tar)

    def update(self):
        self.x_motor.update()
        self.y_motor.update()
        self.z_motor.update()

    # Handler for limit switch interrupts
    # Will update the positions of the motors
    def limit_handler(self, pin):
        global current_state
        curr_time = time.ticks_us()
        # Debounce the switch, and make sure we aren't calibrating right now
        if time.ticks_diff(curr_time, self.last_lim)>100_000 and not isinstance(current_state, CalibrationState):
            if str(pin)==str(xp_lim_pin) and not xn_lim_pin.value()==0:
                self.x_motor.set_position(self.x_motor.max_position)
            elif str(pin)==str(xn_lim_pin) and not xp_lim_pin.value()==0:
                self.x_motor.set_position(self.x_motor.min_position)
            elif str(pin)==str(yp_lim_pin) and not yn_lim_pin.value()==0:
                self.y_motor.set_position(self.y_motor.max_position)
            elif str(pin)==str(yn_lim_pin) and not yp_lim_pin.value()==0:
                self.y_motor.set_position(self.y_motor.min_position)
            elif str(pin)==str(zp_lim_pin) and not zn_lim_pin.value()==0:
                self.z_motor.set_position(self.z_motor.max_position)
            elif str(pin)==str(zn_lim_pin) and not zp_lim_pin.value()==0:
                self.z_motor.set_position(self.z_motor.min_position)
            current_state.user_interface.output('limit switch: ' + str(pin))
        self.last_lim = curr_time

    def at_target(self):
        '''Returns true if all of its motors are at the target position'''
        return all([motor.at_target() for motor in [self.x_motor, self.y_motor, self.z_motor]])

    def normalize_positions(self):
        '''Normalize the positions of all the motors. Only really needs to be
        called after calibration.'''
        self.x_motor.normalize_position()
        self.y_motor.normalize_position()
        self.z_motor.normalize_position()

    def dimensions(self):
        dimensions = []
        dimensions.append(self.x_motor.max_position - self.x_motor.min_position)
        dimensions.append(self.y_motor.max_position - self.y_motor.min_position)
        dimensions.append(self.z_motor.max_position - self.z_motor.min_position)
        return dimensions


class Motor:
    # Type hints for attriutes
    position: int
    max_position: int
    min_position: int
    velocity: float
    max_vel: float
    step_interval: int
    previous_step_ticks: int
    min_pulse_width: int
    step_pin: Pin
    dir_pin: Pin
    def __init__(self, step_pin: machine.Pin, dir_pin: machine.Pin, position: int=0, max_velocity: float=200, max_acceleration: float=100, min_pulse_width: int=3, max_position: int=100, min_position: int=0):
        '''Initialize the motor. Run once at start.'''
        self.max_position = max_position
        self.min_position = min_position
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
        self.target = max(min(tar, self.max_position), self.min_position)

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

    def normalize_position(self):
        '''Makes it so that min_position is zero, and adjusts the current 
        position accordingly'''
        lower = self.min_position
        higher = self.max_position
        width = higher-lower
        pos = self.position
        self.min_position = 0
        self.max_position = width
        self.set_position(pos-lower)

class DoubleMotor(Motor):
    def __init__(self, step_pin: machine.Pin, step_pin_2: machine.Pin, dir_pin: machine.Pin, position: int = 0, max_velocity: float = 200, max_acceleration: float = 100, min_pulse_width: int = 3, max_position: int = 100, min_position: int = 0):
        super().__init__(step_pin, dir_pin, position, max_velocity, max_acceleration, min_pulse_width, max_position, min_position)
        self.step_pin_2 = step_pin_2

    def step(self, reverse: bool = False):
        if (reverse): 
            self.dir_pin.on()
            self.position -= 1
        else:
            self.dir_pin.off()
            self.position += 1

        # Then, pulse the desired pin as quickly as possible
        self.step_pin.on()
        self.step_pin_2.on()
        time.sleep_us(self.min_pulse_width)
        self.step_pin.off() 
        self.step_pin_2.off()
    
class SeedDispenser:
    # These denote the duty cycles of the collect and dispense positions
    # of the servo motor
    DISPENSE_DUTY = 27
    COLLECT_DUTY = 66
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
    global current_state
    FLOW_RATE = 3.5 # Originally was 1.515
    def __init__(self, valve_pin, flow_pin):
        self.valve_pin = valve_pin
        self.flow_pin = flow_pin
        self.flow = 0
        self.total_flow = 0
        self.dispense_target_ml = 0
        self.last_pulse = time.ticks_ms()
        self.dispensing = False
        self.last_water = None

    def update(self):
        '''Opens/closes valve based on what is needed. Returns True if it is
        open, False if it is now closed'''
        if self.dispensing:
            if self.flow < self.dispense_target_ml:
                self.valve_pin.on()
                return True
            else:
                self.valve_pin.off()
                self.flow = 0
                self.dispense_target_ml = 0
                current_state.user_interface.output('flow limit reached. Total flow: ' + str(self.total_flow))
                self.dispensing = False
                return False
        else:
            self.flow = 0
            self.valve_pin.off()

    def dispense(self, dispense_target_ml):
        '''Set the target amount of water to dispense, in mL. Does not reset flow'''
        self.dispense_target_ml = dispense_target_ml
        if dispense_target_ml > 0:
            self.dispensing = True
        else:
            self.dispensing = False

    def water_pulse(self, _pin):
        '''Called by interrupt - increases flow by set rate'''
        current_ticks_ms = time.ticks_ms()
        ms_diff = time.ticks_diff(current_ticks_ms, self.last_pulse)
        # Debounce signal (signal will never be above 400Hz)
        self.total_flow += WaterSystem.FLOW_RATE
        if ms_diff >= 0.001 or True:
            self.flow += WaterSystem.FLOW_RATE
            if round(self.flow/100) != round((self.flow-WaterSystem.FLOW_RATE)/100):
                current_state.user_interface.output(self.flow)
                # current_state.user_interface.output(str(self))
            self.last_pulse = current_ticks_ms
        # current_state.user_interface.output(str(self.flow) + '\n')
        # current_state.user_interface.output(str(self))
    
class UserInterface:
    '''Class designed to get user input.'''
    def __init__(self) -> None:
        self.uart = machine.UART(1, 115200, tx=1, rx=3)
        self.uart.init()

        # Code for connecting to MQTT broker
        # Copied from https://randomnerdtutorials.com/micropython-mqtt-esp32-esp8266/
        # topic_sub = 'emuagrobot22802/control'
        # self.mqtt_client = MQTTClient(ubinascii.hexlify(machine.unique_id()), 'broker.hivemq.com')
        # self.mqtt_client.set_callback(self.mqtt_callback)
        # self.mqtt_client.connect()
        # self.mqtt_client.subscribe(topic_sub)
        # self.output('Connected to %s MQTT broker, subscribed to %s topic' % ('broker.hivemq.com', topic_sub))
        pass

    def get_input_line(self):
        if self.uart.any():
            cmd = self.uart.readline().decode('utf-8').strip()
            return cmd
        return None
    
    def output(self, out_string):
        self.uart.write(out_string.strip() + '\n')
        # self.mqtt_client.publish('emuagrobot22802/message', out_string)

    def mqtt_callback(self, topic, msg):
        '''Handler for MQTT callback'''
        self.output(f'{topic}: {msg}')
        
class MoistureSystem:
    def __init__(self) -> None:
        self.moisture1_pin = machine.ADC(moisture1_pin)
        self.moisture2_pin = machine.ADC(moisture2_pin)
        self.moisture1_pin.atten(machine.ADC.ATTN_11DB)
        self.moisture2_pin.atten(machine.ADC.ATTN_11DB)

    def get_moisture(self):
        sum_1 = 0
        sum_2 = 0
        for _ in range(100):
            sum_1 += self.moisture1_pin.read()
            sum_2 += self.moisture2_pin.read()
            time.sleep(.0001)
        return [int(sum_1/100), int(sum_2/100)]



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
            self.moisture_system = previous_state.moisture_system
        else:
            self.motor_system = MotorSystem()
            self.water_system = WaterSystem(valve_pin, flow_pin)
            self.dispenser = SeedDispenser(servo_pin)
            self.user_interface = UserInterface()
            self.moisture_system = MoistureSystem()
    def run(self) -> 'ProgramState':
        '''Called every program cycle, ideally'''
        raise NotImplementedError
    
    def bot_data(self) -> dict:
        data = dict()
        data['current_state'] = self.state_name()
        data['current_time'] = time.time()
        data['last_water'] = self.water_system.last_water
        data['total_water_ml'] = self.water_system.total_flow
        # TODO: Add moisture data here

    def state_name(self):
        return 'none'
    
class IdleState(ProgramState):
    '''State representing when the machine is doing nothing'''
    def __init__(self, previous_state: 'ProgramState' = None):
        super().__init__(previous_state)

    def run(self):
        return self

    def state_name(self):
        return 'idle'

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
        # Do the Z calibration first so it doesn't drag through the dirt
        self.x_cal_dir = 'none'
        self.y_cal_dir = 'none'
        self.z_cal_dir = 'negative'
    def run(self):
        # Check for limit switch hits (remember, False=hit)
        if self.x_cal_dir=='positive' and xp_lim_pin.value() == False:
            self.x_cal_dir = 'negative'
            self.motor_system.x_motor.max_position = self.motor_system.x_motor.position
        if self.y_cal_dir=='positive' and yp_lim_pin.value() == False:
            self.y_cal_dir = 'negative'
            self.motor_system.y_motor.max_position = self.motor_system.y_motor.position
        if self.z_cal_dir=='positive' and zp_lim_pin.value() == False:
            self.x_cal_dir = 'positive'
            self.y_cal_dir = 'positive'
            self.z_cal_dir = 'done'
            self.motor_system.z_motor.max_position = self.motor_system.z_motor.position

        if self.x_cal_dir=='negative' and xn_lim_pin.value() == False:
            self.x_cal_dir = 'done'
            self.motor_system.x_motor.min_position = self.motor_system.x_motor.position
        if self.y_cal_dir=='negative' and yn_lim_pin.value() == False:
            self.y_cal_dir = 'done'
            self.motor_system.y_motor.min_position = self.motor_system.y_motor.position
        if self.z_cal_dir=='negative' and zn_lim_pin.value() == False:
            self.z_cal_dir = 'positive'
            self.motor_system.z_motor.min_position = self.motor_system.z_motor.position

        # Check which point we are at for each of the stepper motors, and set the
        # velocities appropriately
        if self.x_cal_dir == 'positive':
            self.motor_system.x_motor.set_velocity(self.motor_system.x_motor.max_vel)
        elif self.x_cal_dir == 'negative':
            self.motor_system.x_motor.set_velocity(-self.motor_system.x_motor.max_vel)
        else:
            self.motor_system.x_motor.set_velocity(0)
        if self.y_cal_dir == 'positive':
            self.motor_system.y_motor.set_velocity(self.motor_system.y_motor.max_vel)
        elif self.y_cal_dir == 'negative':
            self.motor_system.y_motor.set_velocity(-self.motor_system.y_motor.max_vel)
        else:
            self.motor_system.y_motor.set_velocity(0)
        if self.z_cal_dir == 'positive':
            self.motor_system.z_motor.set_velocity(self.motor_system.z_motor.max_vel)
        elif self.z_cal_dir == 'negative':
            self.motor_system.z_motor.set_velocity(-self.motor_system.z_motor.max_vel)
        else:
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
            self.motor_system.normalize_positions()
            # current_state.user_interface.output(f'X: {-current_state.motor_system.x_motor.min_position + current_state.motor_system.x_motor.max_position}')
            # current_state.user_interface.output(f'Y: {-current_state.motor_system.y_motor.min_position + current_state.motor_system.y_motor.max_position}')
            # current_state.user_interface.output(f'Z: {-current_state.motor_system.z_motor.min_position + current_state.motor_system.z_motor.max_position}')
            return IdleState(self)
        else:
            return self
        
    def state_name(self):
        return 'calibration'


class WateringState(ProgramState):
    def __init__(self, previous_state: 'ProgramState' = None):
        super().__init__(previous_state)
        self.sub_state = 'initial'
        # TODO: Figure out ml per step.
        self.ml_per_segment = 100
        self.water_system.last_water = time.time()

        zones = 6
        zone_width = math.floor(self.motor_system.dimensions()[0]/zones)
        self.target_positions = [zone_width*i for i in range(zones+1)]
        self.target_i = 0

    def run(self):
        # Position the gantry at the starting position
        if self.sub_state == 'initial':
            self.motor_system.set_target(self.target_positions[0], None, self.motor_system.z_motor.max_position)
            self.motor_system.update()
            if self.motor_system.at_target():
                self.user_interface.output('reached starting position, now watering \n')
                self.pulse_start_time = time.ticks_ms()
                self.water_system.dispense(100)
                self.sub_state = 'watering'
        # Run the actual watering process
        if self.sub_state == 'watering':
            self.water_system.update()
            # Our target x position should be f/r, where f is the total flow in
            # mL, and r is the number of mL per step
            zone_progress = time.ticks_diff(time.ticks_ms(), self.pulse_start_time)/20000
            target_x_position = int(zone_progress*(self.target_positions[self.target_i+1]-self.target_positions[self.target_i]) + self.target_positions[self.target_i])
            self.motor_system.set_target(target_x_position, None, None)
            self.motor_system.update()
            if zone_progress >= 1:
                if self.target_i + 1 == 6:
                    return IdleState(self)
                else:
                    self.target_i += 1
                    self.water_system.dispense(100)
                    self.pulse_start_time = time.ticks_ms()
        return self

    def state_name(self):
        return 'watering'

class PlantingState(ProgramState):
    def __init__(self, previous_state: 'ProgramState' = None, nx=0, ny=0, min_steps_between=0, segment=0):
        super().__init__(previous_state)
        self.current_seed_index = 0
        self.sub_state = 'up'
        self.generate_seed_coords(nx, ny, min_steps_between, segment)

    def generate_seed_coords(self, nx: int, ny: int, min_steps_between=0, segment=0):
        min_length = nx*min_steps_between
        min_width = ny*min_steps_between

        y_range = (self.motor_system.y_motor.max_position-self.motor_system.y_motor.min_position)
        if segment==0:
            x_range = (self.motor_system.x_motor.max_position-self.motor_system.x_motor.min_position)
        else:
            x_range = (self.motor_system.x_motor.max_position-self.motor_system.x_motor.min_position)//2
        if min_length > x_range:
            self.user_interface.output('too long!')
            self.seeds = []
        elif min_width > y_range:
            self.user_interface.output('too wide!')
            self.seeds = []
        else:
            if segment==0 or segment==1:
                starting_x = self.motor_system.x_motor.min_position
            else:
                starting_x = self.motor_system.x_motor.min_position + x_range - 1

            seeds = []
            for x in range(nx):
                for y in range(ny):
                    x_pos = starting_x + round((x+.5)*(x_range/nx))
                    y_pos = self.motor_system.x_motor.min_position + round((y+.5)*(y_range/ny))
                    seeds.append((x_pos, y_pos))
            self.seeds = seeds
        return seeds


    def run(self):
        # 'up' is for raising the planter up to maximum height
        # Do this before index check so we move up after planting last one
        if self.sub_state == 'up':
            self.motor_system.set_target(None, None, self.motor_system.z_motor.max_position)
            self.motor_system.update()
            if self.motor_system.at_target():
                self.sub_state = 'move'
        # Check to see if we are at the end of the list of seeds
        elif self.current_seed_index >= len(self.seeds):
            return IdleState(self)
        # 'move' is for moving to the next seed location
        elif self.sub_state == 'move':
            self.motor_system.set_target(*(self.seeds[self.current_seed_index]), None)
            self.motor_system.update()
            if self.motor_system.at_target():
                self.sub_state = 'down'
        # 'down' is for moving down to plant the seed
        elif self.sub_state == 'down':
            self.motor_system.set_target(None, None, self.motor_system.z_motor.min_position)
            self.motor_system.update()
            if self.motor_system.at_target():
                self.sub_state = 'plant'
        # 'plant' is for actually dispensing the seed
        elif self.sub_state == 'plant':
            self.dispenser.collect()
            time.sleep(1)
            self.dispenser.dispense()
            time.sleep(1)
            self.dispenser.collect()
            time.sleep(1)
            self.current_seed_index += 1
            self.sub_state = 'up'
        return self
    
    def state_name(self):
        return 'planting'

if __name__=='__main__':
    print('starting up')
    main()