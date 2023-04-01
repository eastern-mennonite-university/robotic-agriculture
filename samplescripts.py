
# Move and dispense
motor_system.set_target(100, 100, 100)
at_start = True
while True:
    motor_system.update()
    if motor_system.at_target():
        print('reached target')
        print(f'{motor_system.x_motor.position}, {motor_system.y_motor.position}, {motor_system.z_motor.position}')
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

# Serial stuff
# Everything inside this will run forever
    # serial = machine.UART(1, 115200, tx=1, rx=3)
    # serial.init(115200)

        # if serial.any():
        #     try:
        #         message = serial.readline().decode('utf-8')
        #         frequency = float(message.strip())
        #     except:
        #         frequency = 20
        # motor_system.z_motor.set_velocity(frequency)
        # motor_system.z_motor.run_speed()


# This script dispenses 100ml of water (ideally)
    print('3...')
    time.sleep(1)
    print('2...')
    time.sleep(1)
    print('1...')
    time.sleep(1)
    water_system.dispense(100)
    while water_system.update():
        print(water_system.flow)
        time.sleep(.333)
    print('done')