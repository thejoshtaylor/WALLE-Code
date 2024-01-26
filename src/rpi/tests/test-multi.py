from pyPS4Controller.controller import Controller
import serial
from time import sleep
import struct

ser = serial.Serial("/dev/ttyS0", 9600)
# while True:
	# received_data = ser.read()
	# sleep(0.03)
	# data_left = ser.inWaiting()
	# received_data += ser.read(data_left)
	# print(received_data)
	# ser.write(received_data)

class MyController(Controller):
	def __init__(self, **kwargs):
		Controller.__init__(self, **kwargs)
		
	def on_x_press(self):
		msg = struct.pack('>B', 250)
		print('L', msg)
		ser.write(b'L' + msg)
		
	def on_x_release(self):
		msg = struct.pack('>B', 0)
		print('L', msg)
		ser.write(b'L' + msg)
		
	def on_L2_press(self, value):
		msg = struct.pack('>B', round((value + 32767) / 257))
		print('L', msg)
		ser.write(b'L' + msg)
		
	def on_L2_release(self, ):
		msg = struct.pack('>B', 0)
		print('L', msg)
		ser.write(b'L' + msg)
		
	def on_R2_press(self, value):
		msg = struct.pack('>B', round((value + 32767) / 257))
		print('R', msg)
		ser.write(b'R' + msg)
		
	def on_R2_release(self, ):
		msg = struct.pack('>B', 0)
		print('R', msg)
		ser.write(b'R' + msg)
		
controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen(timeout=60)
print("Let's go!")
