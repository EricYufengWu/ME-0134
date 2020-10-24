import odrive
import time

odrv0 = odrive.find_any()
my_motor = odrv0.axis0

# print bus voltage
print("vbus voltage: ", odrv0.vbus_voltage)

# setup parameters, only needed if using a fresh new odrive
# odrv0.config.brake_resistance = 0.05 #(if using a power supply, consider adding in a brake resistor)
# odrv0.axis0.motor.config.pole_pairs = 11 # for Tmotor MN5212
# odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
# odrv0.axis0.encoder.config.cpr = 2000
# odrv0.axis0.controller.config.vel_limit = 10
# odrv0.save_configuration()

# run motor calibration sequence
my_motor.requested_state = 3
time.sleep(15)

# setup close_loop control
my_motor.requested_state = 8 # AXIS_STATE_CLOSED_LOOP_CONTROL

# first slowly move the motor back to zero pos
my_motor.controller.config.control_mode = 3 # position control
my_motor.controller.config.vel_limit = 1
my_motor.controller.input_pos = 0
time.sleep(3)
my_motor.controller.config.vel_limit = 10

my_motor.controller.config.control_mode = 1 # torque control
# loop
while True:
    try: 
        my_motor.controller.input_torque = 0.1  # unit is in Nm
        time.sleep(1)
        print(my_motor.encoder.shadow_count)  # unit is in Nm
        my_motor.controller.input_pos = -0.1
        time.sleep(1)
        print(my_motor.encoder.shadow_count)
    except KeyboardInterrupt:
        my_motor.controller.input_torque = 0
        my_motor.requested_state = 1  #AXIS_STATE_IDLE
        break



