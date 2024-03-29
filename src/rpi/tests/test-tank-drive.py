from pyPS4Controller.controller import Controller
import serial
from time import sleep
import struct
from math import atan2, sqrt, pi

ser = serial.Serial("/dev/ttyS0", 115200)



# Message variables
intro = [0x55, 0x4f, 0x02, 0x39, 0x81, 0xc4]
terminator = [0x0c, 0xf7, 0x13, 0x85, 0x3f, 0x5a]

# Default message parameters
leftSpeed = 0
rightSpeed = 0

leftArmLength = 0
rightArmLength = 0

leftArmAngle = 0
rightArmAngle = 0

leftWingAngle = 0
rightWingAngle = 0

leftHandAngle = 0
rightHandAngle = 0

bigFaceLatch = True

def send_default_packet():

	# Intro
	for i in intro:
		ser.write(struct.pack('>B', i))

	# Type
	ser.write(b'\x01')
	
	def prep(val):
		temp = round(val)
		if temp > 32767:
			temp = 32767
		elif temp < -32768:
			temp = -32768
		return temp

	# Message
	data = b''
	# >h = big endian short
	data += struct.pack('>h', prep(leftSpeed))
	data += struct.pack('>h', prep(rightSpeed))

	data += struct.pack('>h', prep(leftArmLength))
	data += struct.pack('>h', prep(rightArmLength))

	data += struct.pack('>h', prep(leftArmAngle))
	data += struct.pack('>h', prep(rightArmAngle))

	data += struct.pack('>h', prep(leftWingAngle))
	data += struct.pack('>h', prep(rightWingAngle))

	data += struct.pack('>h', prep(leftHandAngle))
	data += struct.pack('>h', prep(rightHandAngle))

	data += struct.pack('>B', bigFaceLatch)

	# fill in the rest of the 32 bytes
	for i in range(0, 32 - len(data)):
		data += b'\x00'
	
	ser.write(data)

	# Checksum
	checksum = 0
	for i in data:
		checksum += i

	checksum = checksum % 256
	ser.write(struct.pack('>B', checksum))

	# Terminator
	for i in terminator:
		ser.write(struct.pack('>B', i))
		

def update_drive(x, y):
	global leftSpeed
	global rightSpeed

	if x is not None:
		last_drive_x = x
	if y is not None:
		last_drive_y = y

	if last_drive_x == 0:
		if last_drive_y >= 0:
			drive_angle = pi/2
		else:
			drive_angle = 3*pi/2
	else:
		# Get the angle between zero and 2pi
		drive_angle = atan2(last_drive_y, last_drive_x)
		if drive_angle < 0:
			drive_angle += 2*pi

	drive_speed = sqrt(last_drive_x * last_drive_x + last_drive_y * last_drive_y)

	# Set left speed
	if drive_angle >= 0 and drive_angle <= pi / 2:
		leftSpeed = drive_speed
	elif drive_angle >= pi and drive_angle <= 3 * pi / 2:
		leftSpeed = -drive_speed
	elif drive_angle > pi / 2 and drive_angle < pi:
		# Put scale between -1 and 1
		scale = drive_angle - (3 * pi / 4)
		scale = -scale / (pi / 4)
		# Linearly scale speed
		leftSpeed = scale * drive_speed
	elif drive_angle > 3 * pi / 2 and drive_angle < 2 * pi:
		# Put scale between -1 and 1
		scale = drive_angle - (7 * pi / 4)
		scale = scale / (pi / 4)
		# Linearly scale speed
		leftSpeed = scale * drive_speed

	# Set right speed
	if drive_angle >= pi / 2 and drive_angle <= pi:
		rightSpeed = drive_speed
	elif drive_angle == 0 or drive_angle >= 3 * pi / 2 and drive_angle <= 2 * pi:
		rightSpeed = -drive_speed
	elif drive_angle > 0 and drive_angle < pi / 2:
		# Put scale between -1 and 1
		scale = drive_angle - (pi / 4)
		scale = scale / (pi / 4)
		# Linearly scale speed
		rightSpeed = scale * drive_speed
	elif drive_angle > pi and drive_angle < 3 * pi / 2:
		# Put scale between -1 and 1
		scale = drive_angle - (5 * pi / 4)
		scale = -scale / (pi / 4)
		# Linearly scale speed
		rightSpeed = scale * drive_speed

	print('Angle', drive_angle, 'Speed', drive_speed, 'Right', rightSpeed, 'Left', leftSpeed)

	send_default_packet()

class MyController(Controller):
	def __init__(self, **kwargs):
		Controller.__init__(self, **kwargs)
		self.last_drive_x = 0
		self.last_drive_y = 0

	# Left stick (tank drive)
	def on_L3_up(self, value):
		self.last_drive_y = -value
		update_drive(x=self.last_drive_x, y=-value)
	
	def on_L3_down(self, value):
		self.last_drive_y = -value
		update_drive(x=self.last_drive_x, y=-value)
	
	def on_L3_left(self, value):
		self.last_drive_x = value
		update_drive(x=value, y=self.last_drive_y)
	
	def on_L3_right(self, value):
		self.last_drive_x = value
		update_drive(x=value, y=self.last_drive_y)
	
	def on_L3_x_at_rest(self, ):
		self.last_drive_x = 0
		update_drive(x=0, y=self.last_drive_y)
		
	def on_L3_y_at_rest(self, ):
		self.last_drive_y = 0
		update_drive(x=self.last_drive_x, y=0)
	
		
controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen(timeout=60)
print("Let's go!")
