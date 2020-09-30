import pigpio
import time
from bluedot.btcomm import BluetoothServer
from signal import pause
import threading

max_value = 1900
min_spin = 1200
min_value = 1000
ESC_pin = 4
pwm_val = 1000
pi = pigpio.pi();
pi.set_servo_pulsewidth(ESC_pin, 0) 

# The brushless motor ESC needs to be calibrated before use.
def calibrate():
	pi.set_servo_pulsewidth(ESC_pin, min_value) 
	time.sleep(1)
	pi.set_servo_pulsewidth(ESC_pin, min_spin)
	time.sleep(1)
	pi.set_servo_pulsewidth(ESC_pin, min_value)
	print("calibration done")

# Timer callback which is executed avery 0.1 second. This updates the signal to the ESC.
def update():
	global pwm_val
	try:
		threading.Timer(0.1,update).start()
		print(pwm_val)
		pi.set_servo_pulsewidth(ESC_pin, pwm_val)
	except:
		return

# updating the PWM signal from command received via bluetooth. 
def control(cmd):
	global pwm_val
	val = float(cmd[2])
	if cmd[1] == 'RIGHT-Y' and val >= 0.0:
		pwm_val = int(val * 900 + 1000)

# Callback which handles bluetooth communication from the client Raspberry Pi.
def data_received(data):
    cmd = data.split()[:3]
    # print(cmd)
    control(cmd)
    # s.send("server received: " + cmd)

calibrate()
update()
s = BluetoothServer(data_received)
pause()
