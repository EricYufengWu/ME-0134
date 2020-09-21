import pigpio
import time

max_value = 1900
min_spin = 1200
min_value = 1000
ESC_pin = 4
pi = pigpio.pi();
pi.set_servo_pulsewidth(ESC_pin, 0) 

def calibrate():   #This is the auto calibration procedure of a normal ESC
	pi.set_servo_pulsewidth(ESC_pin, min_value) 
	time.sleep(1)
	pi.set_servo_pulsewidth(ESC_pin, min_spin)
	time.sleep(1)
	pi.set_servo_pulsewidth(ESC_pin, min_value)
	print("calibration done")

calibrate()

while True:
	try:
		duty = int(input('duty cycle: '))
		time.sleep(0.1)
		pi.set_servo_pulsewidth(ESC_pin, duty)
	except KeyboardInterrupt:
		pi.set_servo_pulsewidth(ESC_pin, 0)
		break
	except:
		continue
