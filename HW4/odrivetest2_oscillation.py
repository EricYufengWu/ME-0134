import odrive
import time

odrv0 = odrive.find_any()
my_motor = odrv0.axis0

# print bus voltage
print("vbus voltage: ", odrv0.vbus_voltage)

# setup parameters
odrv0.config.brake_resistance = 0.05 #(if using a power supply, consider adding in a brake resistor)
odrv0.axis0.motor.config.pole_pairs = 11
odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis0.encoder.config.cpr = 2000
odrv0.axis0.controller.config.vel_limit = 20

# run motor calibration sequence
my_motor.requested_state = 3
time.sleep(10)

# setup close_loop control
my_motor.requested_state = 8 # AXIS_STATE_CLOSED_LOOP_CONTROL

# run motor at velocity mode (incremental speed)
my_motor.controller.config.control_mode = 2
while True:
    try: 
        for speed in range(20):
            my_motor.controller.input_vel = speed
            time.sleep(0.2)
        for speed in range(20):
            my_motor.controller.input_vel = 20 - speed
            time.sleep(0.2)
        for speed in range(20):
            my_motor.controller.input_vel = -speed
            time.sleep(0.2)
        for speed in range(20):
            my_motor.controller.input_vel = -20 + speed
            time.sleep(0.2)
    except KeyboardInterrupt:
        my_motor.controller.input_vel = 0
        my_motor.requested_state = 1  #A XIS_STATE_IDLE
        break

