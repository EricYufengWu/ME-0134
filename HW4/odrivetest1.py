import odrive
import time

odrv0 = odrive.find_any()
my_motor = odrv0.axis0

# print bus voltage
print("vbus voltage: ", odrv0.vbus_voltage)

# run motor calibration sequence
my_motor.requested_state = 3
time.sleep(15)

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
