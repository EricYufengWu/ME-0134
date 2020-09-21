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

def calibrate():   #This is the auto calibration procedure of a normal ESC
	pi.set_servo_pulsewidth(ESC_pin, min_value) 
	time.sleep(1)
	pi.set_servo_pulsewidth(ESC_pin, min_spin)
	time.sleep(1)
	pi.set_servo_pulsewidth(ESC_pin, min_value)
	print("calibration done")

def update():
	global pwm_val
	try:
		threading.Timer(0.1,update).start()
		print(pwm_val)
		pi.set_servo_pulsewidth(ESC_pin, pwm_val)
	except:
		return

def control(cmd):
	global pwm_val
	val = float(cmd[2])
	if cmd[1] == 'RIGHT-Y' and val >= 0.0:
		pwm_val = int(val * 900 + 1000)

def data_received(data):
    cmd = data.split()[:3]
    # print(cmd)
    control(cmd)
    # s.send("server received: " + cmd)

calibrate()
update()
s = BluetoothServer(data_received)
pause()
