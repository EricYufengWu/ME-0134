import RPi.GPIO as GPIO
from time import sleep

pin = 12				# PWM pin connected to LED
GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BOARD)		#set pin numbering system
GPIO.setup(pin,GPIO.OUT)
pi_pwm = GPIO.PWM(pin,8000)		#create PWM instance with frequency
pi_pwm.start(0)				#start PWM of required Duty Cycle 

while True:
	try:
		duty = int(input('duty cycle: '))
		pi_pwm.ChangeDutyCycle(duty)
		sleep(0.1)
	except KeyboardInterrupt:
		pi_pwm.ChangeDutyCycle(0)
		break
	except:
		continue