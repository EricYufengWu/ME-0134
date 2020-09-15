from bluedot.btcomm import BluetoothClient
from signal import pause
import time

def data_received(data):
    print(data)

c = BluetoothClient("raspberrypi", data_received)

while True:
	try:
		msg = input('message to server: ')
		c.send(msg)
		time.sleep(0.1)
	except KeyboardInterrupt:
		break

pause()