from pyPS4Controller.controller import Controller
import serial
from time import sleep
import struct
from math import atan2, sqrt, pi
import numpy as np
import time

ser = serial.Serial("/dev/ttyS0", 115200)

import statistics
import time

start_time = time.time()  # Global time variable
# CONSTANT variables
AVG_POINTS = 10

l3_vert_old = 0 # vertical component of L3 toggle, related to forward drive velocity
l3_horz_old = 0 # horizontal component of L3 toggle, related to sideways drive velocity
r3_vert_old = 0 # vertical component of R3 toggle, related to upper arm/shoulder flexion
r3_horz_old = 0 # horizontal component of R3 toggle, related to upper arm abduction (ie. lateral raise)
wristbro_old = 0 # represents wrist angle for both wrists (just testing functionality)
r2_old = -32768
l2_old = -32768


l3_vert_pre = 0 # pre means pre-filtered values
l3_horz_pre = 0
r3_vert_pre = 0
r3_horz_pre = 0
r2_pre = -32768
l2_pre = -32768


l3_vert = 0
l3_horz = 0
r3_vert = 0
r3_horz = 0
wristbro = 0
r2 = -32768
l2 = -32768

filter_objects = {
    'l3_vert': [l3_vert_pre] * AVG_POINTS,
    'l3_horz': [l3_horz_pre] * AVG_POINTS,

    'r3_vert': [r3_vert_pre] * AVG_POINTS,
    'r3_horz': [r3_horz_pre] * AVG_POINTS,

    'r2': [r2_pre] * AVG_POINTS,
    'l2': [l2_pre] * AVG_POINTS,

}

#
# INPUT FILTERING
#
def updateFilters():
    global l3_vert_pre, l3_horz_pre, r3_vert_pre, r3_horz_pre, r2_pre, l2_pre
    global l3_vert, l3_horz, r3_vert, r3_horz, r2, l2
    global filter_objects

    for key in filter_objects:
        filter_objects[key].pop(0)
        value = globals()[key + '_pre']
        if abs(value) < 800: # stops whining :o, high-frequency mini-corrections for the position of the actuator
            value = 0
        filter_objects[key].append(value)
		
    l3_vert = statistics.fmean(filter_objects['l3_vert'])
    l3_horz = statistics.fmean(filter_objects['l3_horz'])
    r3_vert = statistics.fmean(filter_objects['r3_vert'])
    r3_horz = statistics.fmean(filter_objects['r3_horz'])
    r2 = statistics.fmean(filter_objects['r2'])
    l2 = statistics.fmean(filter_objects['l2'])
	
    triggerUpdate()

def triggerUpdate():
	global l3_vert_old, l3_horz_old, r3_vert_old, r3_horz_old, r2_old, l2_old, wristbro_old
	global l3_vert, l3_horz, r3_vert, r3_horz, r2, l2, wristbro
	global leftArmLength, leftArmAngle, leftWingAngle, rightArmLength, rightArmAngle, rightWingAngle, leftHandAngle, rightHandAngle
	# Left stick (tank drive)
	if l3_vert != l3_vert_old or l3_horz != l3_horz_old:
		update_drive(x=l3_horz, y=l3_vert)
		l3_vert_old = l3_vert
		l3_horz_old = l3_horz

	# Right stick (arm elevation)
	if r3_vert != r3_vert_old or r3_horz != r3_horz_old or r2 != r2_old:
		leftArmLength = r2
		rightArmLength = r2
		leftArmAngle = r3_vert
		rightArmAngle = r3_vert
		leftWingAngle = r3_horz
		rightWingAngle = r3_horz
		
		# update wristAngle
		# ps4 controller library to understand more about controller input formatting
		send_default_packet() # sends updated variables to teensy
		r3_vert_old = r3_vert
		r3_horz_old = r3_horz
		r2_old = r2

	# circle button (wrist control)
	if wristbro != wristbro_old:
		leftHandAngle = wristbro
		rightHandAngle = wristbro
		send_default_packet()
		wristbro_old = wristbro


	global shredder_good, shredder_value

	# Left trigger (shredder control [TEMPORARY])
	if l2 != l2_old:
		if True:#shredder_good:
			pass#shredder_value = l2
		else:
			shredder_value = -32768

		# send_shredder_packet()
		l2_old = l2

		
# Update PIDs on a regular interval
def filterThread():
    while True:
        updateFilters()
        time.sleep(0.025)


# Message variables
intro = [0x55, 0x4f, 0x02, 0x39, 0x81, 0xc4]
terminator = [0x0c, 0xf7, 0x13, 0x85, 0x3f, 0x5a]

# Default message parameters
leftSpeed = 0 # leftSpeed is the leftward tank drive speed
rightSpeed = 0 # rightSpeed is the rightward tank drive speed

leftArmLength = 0 # this is related to forearm length
rightArmLength = 0 

leftArmAngle = 0 # this is related to upper arm angle
rightArmAngle = 0

leftWingAngle = 0 # this is the arm abduction angle
rightWingAngle = 0

leftHandAngle = 0 # this is the wrist servo
rightHandAngle = 0

bigFaceLatch = True

shredder_running = False
shredder_good = False
shredder_value = -32768
shredder_direction = 0

def send_default_packet():
	global shredder_good
	shredder_good = False

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


	data += struct.pack('>B', bigFaceLatch) # sending 32 max bytes of data, bigFaceLatch is an on-off (1) byte, but other inputs are 2-4 bytes long

	

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

#
# SHREDDER
#
def send_shredder_packet():
	global shredder_running
	global shredder_good
	global shredder_value
	global shredder_direction

	# Intro
	for i in intro:
		ser.write(struct.pack('>B', i))

	# Type
	ser.write(b'\x02')

	# Message
	data = b''

	# Insert double check 0xdeadbeef
	data += struct.pack('>B', 0xde)
	# one and two are empty
	for i in range(0, 2):
		data += struct.pack('>B', 0x00)
	# three is be
	data += struct.pack('>B', 0xbe)
	# four and five are the shredder value
	
	# Make sure we're good to go
	if True:#shredder_good and shredder_running:
		data += struct.pack('>h', shredder_value)
	else:
		data += struct.pack('>h', -32768)

	# six is the direction
	data += struct.pack('>B', shredder_direction)

	# seven is empty
	for i in range(6, 7):
		data += struct.pack('>B', 0x00)
	# eight is ad
	data += struct.pack('>B', 0xad)
	# nine through eighteen are empty
	for i in range(8, 18):
		data += struct.pack('>B', 0x00)
	# nineteen is ef
	data += struct.pack('>B', 0xef)


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

#		
# TANK DRIVE
#
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
		# self.last_drive_x = 0
		# self.last_drive_y = 0


	# Left stick (tank drive)
	def on_L3_up(self, value):
		global l3_vert_pre
		l3_vert_pre = -value
		# self.last_drive_y = -value
		# update_drive(x=self.last_drive_x, y=-value)
	
	def on_L3_down(self, value):
		global l3_vert_pre
		l3_vert_pre = -value
		# self.last_drive_y = -value
		# update_drive(x=self.last_drive_x, y=-value)
	
	def on_L3_left(self, value):
		global l3_horz_pre
		l3_horz_pre = value
		# self.last_drive_x = value
		# update_drive(x=value, y=self.last_drive_y)
	
	def on_L3_right(self, value):
		global l3_horz_pre
		l3_horz_pre = value
		# self.last_drive_x = value
		# update_drive(x=value, y=self.last_drive_y)
	
	def on_L3_x_at_rest(self, ):
		global l3_horz_pre
		l3_horz_pre = 0
		# self.last_drive_x = 0
		# update_drive(x=0, y=self.last_drive_y)
		
	def on_L3_y_at_rest(self, ):
		global l3_vert_pre
		l3_vert_pre = 0
		# self.last_drive_y = 0
		# update_drive(x=self.last_drive_x, y=0)

	# Right stick (arm elevation)
	def on_R3_up(self, value):
		global r3_vert_pre
		# global leftArmAngle
		# leftArmAngle = value
		r3_vert_pre = -value
		# send_default_packet()

	def on_R3_down(self, value):
		global r3_vert_pre
		# global leftArmAngle
		# leftArmAngle = value
		r3_vert_pre = -value
		# send_default_packet()

	def on_R3_y_at_rest(self, ):
		global r3_vert_pre # this is related to the value of L1, or shoulder flexion
		global r3_horiz_pre # this is related to the value of L3, or elbow flexion
		overall_time = time.time() - start_time  # Continuously increasing global time
    	local_time = overall_time % 10  # Wrap every 10 seconds
		t = local_time
		x = 20 + 5 * np.sin(2 * np.pi * t /10)  # x-position input (in inches)
		y = -20  # y-position input (in inches)

		# The program should find L1 and L2 to satisfy these requirements
		r = 10  # r is the distance from the base of actuator 1 to the base of actuator 2 (in inches)
		x0 = r * np.cos(np.deg2rad(80))  # x0 is the horizontal component of r, assuming its angle is 80 degrees (in inches)
		y0 = r * np.sin(np.deg2rad(80))  # y0 is the vertical component of r, assuming its angle is 80 degrees (in inches)
		L2 = 2  # L2 is the length of the link that connects the end of actuator 1 and the base of actuator 2 (in inches)

		L3 = np.sqrt(x**2 + y**2)  # L3 is the distance from the base of actuator 2 to Wall-E's hands, or the end of actuator 2 (in inches)
		theta3 = np.rad2deg(np.arctan(y / x))  # theta3 is the angle of actuator 2 relative to horizontal. Returns between -90 degrees and 90 degrees
		theta2 = 200 - theta3  # theta2 is the angle of the link connecting actuator 1 and 2
		L1 = np.sqrt(r**2 - 2 * L2 * (x0 * np.cos(np.deg2rad(theta2)) + y0 * np.sin(np.deg2rad(theta2))) + L2**2)  # L1 is the length of actuator 1 (in inches)
		
		forearm = L1 - 3 # this is the length of the forearm
		upperArm = L3 - 3 # this is the length of the upper arm

		# global leftArmAngle
		# leftArmAngle = 0
		r3_horiz_pre = (65536 / 18) * forearm - 32768
		r3_vert_pre = (65536 / 18) * upperArm - 32768 # we want to return a pre-filtered value even though we can calculate the post-filtered one. This simplifies the code in that we're already 
		# filtering the value from the 2 other R3 functions, so we can maintain consistency here.
		# send_default_packet()

	def on_R3_left(self, value):
		global r3_horz_pre
		# global leftWingAngle
		# leftWingAngle = value
		r3_horz_pre = value
		# send_default_packet()

	def on_R3_right(self, value):
		global r3_horz_pre
		# global leftWingAngle
		# leftWingAngle = value
		r3_horz_pre = value
		# send_default_packet()

	#def on_R3_x_at_rest(self, ):
	#	global r3_horz_pre
		# global leftWingAngle
		# leftWingAngle = 0
	#	r3_horz_pre = 0
		# send_default_packet()


	# Right trigger (arm extension)
	def on_R2_press(self, value):
		global r2_pre
		# global leftArmLength
		r2_pre = value
		# leftArmLength = value
		# send_default_packet()

	def on_R2_release(self, ):
		global r2_pre
		# global leftArmLength
		r2_pre = -32768
		# leftArmLength = -32768
		# send_default_packet()

	# Left trigger (shredder control [TEMPORARY])
	def on_L2_press(self, value):
		global shredder_good
		global shredder_running
		global shredder_value
		# global l2_pre

		shredder_running = True

		# l2_pre = value

		if shredder_good:
			shredder_value = value
			send_shredder_packet()

	def on_L2_release(self, ):
		global shredder_running
		global shredder_value
		shredder_running = False
		shredder_value = -32768
		# global l2_pre

		# l2_pre = -32768

		send_shredder_packet()

	# X button (shredder good and forward)
	def on_x_press(self, ):
		global shredder_running
		global shredder_good
		global shredder_direction

		if not shredder_running and not shredder_good:
			shredder_direction = 1
			shredder_good = True
		else:
			shredder_good = False
		
		send_shredder_packet()
		
	# release
	def on_x_release(self, ):
		global shredder_running
		global shredder_good
		global shredder_direction

		shredder_good = False

		send_shredder_packet()

	# triangle button (shredder good and reverse)
	def on_triangle_press(self, ):
		global shredder_running
		global shredder_good
		global shredder_direction

		if not shredder_running and not shredder_good:
			shredder_direction = 0
			shredder_good = True
		else:
			shredder_good = False
		
		send_shredder_packet()

	# release
	def on_triangle_release(self, ):
		global shredder_running
		global shredder_good
		global shredder_direction

		shredder_good = False

		send_shredder_packet()

	def on_circle_press(self, ):
		global wristbro
		wristbro = 90.0
		

	def on_circle_release(self, ):
		global wristbro
		wristbro = 0.0

# Start the thread
import threading
thread = threading.Thread(target=filterThread)
thread.start()
		
controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
print(controller.listen(timeout=60))
print("Let's go!")
