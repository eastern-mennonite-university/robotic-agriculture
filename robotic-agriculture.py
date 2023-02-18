
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

# Everything inside this will run forever
while True:
    z_step_pin.on()
    time.sleep(0.005)
    z_step_pin.off()
    time.sleep(0.005)


#Reference: https://icircuit.net/micropython-controlling-servo-esp32-nodemcu/2385
def servo_motor():
    p4 = machine.Pin(4) 
    servo = machine.PWM(p4, freq = 50) #calls the pin and the frequency to send to the pin.
    servo.duty(10) #Tells the servo motor to go to said degree (10 degrees).
    servo.duty(80) #Tells servo to move to other said degree (80 degreees).
