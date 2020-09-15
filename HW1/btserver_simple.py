from bluedot.btcomm import BluetoothServer
from signal import pause

def data_received(data):
    print(data)
    s.send("server received: " + data)

s = BluetoothServer(data_received)

pause()