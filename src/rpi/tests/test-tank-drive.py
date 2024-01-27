from pyPS4Controller.controller import Controller
import serial
from time import sleep
import struct
import math

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
		print(struct.pack('>B', i))
		ser.write(struct.pack('>B', i))

	# Type
	print(b'\x01')
	ser.write(b'\x01')
	
	# Message
	data = b''
	# >h = big endian short
	data += struct.pack('>h', leftSpeed)
	data += struct.pack('>h', rightSpeed)

	data += struct.pack('>h', leftArmLength)
	data += struct.pack('>h', rightArmLength)

	data += struct.pack('>h', leftArmAngle)
	data += struct.pack('>h', rightArmAngle)

	data += struct.pack('>h', leftWingAngle)
	data += struct.pack('>h', rightWingAngle)

	data += struct.pack('>h', leftHandAngle)
	data += struct.pack('>h', rightHandAngle)

	data += struct.pack('>B', bigFaceLatch)

	# fill in the rest of the 32 bytes
	for i in range(0, 32 - len(data)):
		data += b'\x00'
	
	print(data)
	ser.write(data)

	# Checksum
	checksum = 0
	for i in data:
		checksum += i

	checksum = checksum % 256
	print(struct.pack('>B', checksum))
	ser.write(struct.pack('>B', checksum))

	# Terminator
	for i in terminator:
		print(struct.pack('>B', i))
		ser.write(struct.pack('>B', i))

# while True:
# 	send_default_packet()
# 	sleep(0.001)
# 	leftSpeed += 1
# 	rightSpeed -= 1

drive_angle = 0
drive_speed = 0

last_drive_x = 0
last_drive_y = 0

def update_drive(x=None, y=None):
	global drive_angle
	global drive_speed

	global last_drive_x
	global last_drive_y

	global leftSpeed
	global rightSpeed

	if x is not None:
		last_drive_x = x
	if y is not None:
		last_drive_y = y

	drive_angle = math.atan(last_drive_y / last_drive_x)
	drive_speed = math.sqrt(last_drive_x * last_drive_x + last_drive_y * last_drive_y)

	# Set left speed
	if drive_angle >= 0 and drive_angle <= pi / 2:
		leftSpeed = drive_speed
	elif drive_angle >= pi and drive_angle <= 3 * pi / 2:
		leftSpeed = -drive_speed
	elif drive_angle > pi / 2 and drive_angle < pi:
		# Put scale between -1 and 1
		scale = drive_angle - (3 * pi / 4)
		scale = scale / (pi / 4)
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
		scale = scale / (pi / 4)
		# Linearly scale speed
		rightSpeed = scale * drive_speed

	send_default_packet()

class MyController(Controller):
	def __init__(self, **kwargs):
		Controller.__init__(self, **kwargs)

	# Left stick (tank drive)
	def on_L3_up(self, value):
		update_drive(y=value)
	
	def on_L3_down(self, value):
		update_drive(y=value)
	
	def on_L3_left(self, value):
		update_drive(x=value)
	
	def on_L3_right(self, value):
		update_drive(x=value)
	
	def on_L3_x_at_rest(self, ):
		update_drive(s=value)
		
	def on_L3_y_at_rest(self, ):
		update_drive(y=value)
	
		
controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen(timeout=60)
print("Let's go!")
