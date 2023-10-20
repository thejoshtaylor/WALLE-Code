from pyPS4Controller.controller import Controller
import serial
from time import sleep

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
		print("Hello world")
		ser.write('3'.encode('utf-8'));
		
	def on_x_release(self):
		print("Goodbye world")
		
	def on_R2_press(self, value):
		ser.write(round((value + 32767) / 6537));
		
controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen(timeout=60)
print("Let's go!")
