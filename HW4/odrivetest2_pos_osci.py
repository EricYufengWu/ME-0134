import odrive
import time

odrv0 = odrive.find_any()
my_motor = odrv0.axis0

# print bus voltage
print("vbus voltage: ", odrv0.vbus_voltage)

# setup parameters
# odrv0.config.brake_resistance = 0.05 #(if using a power supply, consider adding in a brake resistor)
# odrv0.axis0.motor.config.pole_pairs = 11
# odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
# odrv0.axis0.encoder.config.cpr = 2000
# odrv0.axis0.controller.config.vel_limit = 20

# run motor calibration sequence
my_motor.requested_state = 3
time.sleep(15)

# setup close_loop control
my_motor.requested_state = 8 # AXIS_STATE_CLOSED_LOOP_CONTROL

# run motor at velocity mode (incremental speed)
my_motor.controller.config.control_mode = 3 # position control

# first slowly move the motor back to zero pos
my_motor.controller.config.vel_limit = 1
my_motor.controller.input_pos = 0
time.sleep(5)
my_motor.controller.config.vel_limit = 3

# loop oscillation
while True:
    try: 
        my_motor.controller.input_pos = 0.02
        time.sleep(0.1)
        print(my_motor.encoder.shadow_count)
        my_motor.controller.input_pos = 0
        time.sleep(0.1)
        print(my_motor.encoder.shadow_count)
    except KeyboardInterrupt:
        my_motor.controller.input_pos = 0
        my_motor.requested_state = 1  #A XIS_STATE_IDLE
        break


