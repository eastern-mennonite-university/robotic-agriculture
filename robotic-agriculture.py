from machine import Pin

ledPin = Pin()




# I wasnt sure where I should add the code to cntrol the servo so I put it here. 
# It appears to be a very simple code with the ESP32.
#Referance Website https://icircuit.net/micropython-controlling-servo-esp32-nodemcu/2385

import machine

def servo_motor():
    p4 = machine.Pin(4) #assignes the pin 4 output to the servo motor.
    servo = machine.PWM(p4, freq = 50) #calls the pin and the frequency to send to the pin.
    servo.duty(10) #Tells the servo motor to go to said degree (10 degrees).
    servo.duty(80) #Tells servo to move to other said degree (80 degreees).
