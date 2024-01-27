import serial
from time import sleep
import struct

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

while True:
	send_default_packet()
	sleep(0.001)
	leftSpeed += 1
	rightSpeed -= 1