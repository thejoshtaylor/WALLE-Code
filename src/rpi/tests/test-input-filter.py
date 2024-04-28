from pyPS4Controller.controller import Controller
from simple_pid import PID

import statistics
import time

# CONSTANT variables
AVG_POINTS = 10

l3_vert_pre = 0
l3_horz_pre = 0
r3_vert_pre = 0
r3_horz_pre = 0
r2_pre = -32768
l2_pre = -32768

l3_vert = 0
l3_horz = 0
r3_vert = 0
r3_horz = 0
r2 = -32768
l2 = -32768

pid_objects = {
    'l3_vert': [l3_vert_pre] * AVG_POINTS,
    'l3_horz': [l3_horz_pre] * AVG_POINTS,

    'r3_vert': [r3_vert_pre] * AVG_POINTS,
    'r3_horz': [r3_horz_pre] * AVG_POINTS,

    'r2': [r2_pre] * AVG_POINTS,
    'l2': [l2_pre] * AVG_POINTS,
}

def printTable():
    print('|', '-'*10, '|', '-'*10, '|', '-'*10, '|', '-'*10, '|', '-'*10, '|', '-'*10, '|', '-'*10, '|', sep='-')
    print('|', ' '*10, '| {0:10} | {1:10} | {2:10} | {3:10} | {4:10} | {5:10} |'.format('L3_vert', 'L3_horz', 'R3_vert', 'R3_horz', 'R2', 'L2'))
    print('|', '-'*10, '|', '-'*10, '|', '-'*10, '|', '-'*10, '|', '-'*10, '|', '-'*10, '|', '-'*10, '|', sep='-')
    print('|', '   raw    ', '| {0:10.2f} | {1:10.2f} | {2:10.2f} | {3:10.2f} | {4:10.2f} | {5:10.2f} |'.format(l3_vert_pre, l3_horz_pre, r3_vert_pre, r3_horz_pre, r2_pre, l2_pre))
    print('|', '   filt   ', '| {0:10.2f} | {1:10.2f} | {2:10.2f} | {3:10.2f} | {4:10.2f} | {5:10.2f} |'.format(l3_vert, l3_horz, r3_vert, r3_horz, r2, l2))
    print('|', '-'*10, '|', '-'*10, '|', '-'*10, '|', '-'*10, '|', '-'*10, '|', '-'*10, '|', '-'*10, '|', sep='-')
	
def retractCursor():
    for i in range(0, 6):
        print("\033[A                             \033[A")

def updatePIDs():
    global l3_vert_pre, l3_horz_pre, r3_vert_pre, r3_horz_pre, r2_pre, l2_pre
    global l3_vert, l3_horz, r3_vert, r3_horz, r2, l2
    global pid_objects

    for key in pid_objects:
        pid_objects[key].pop(0)
        value = globals()[key + '_pre']
        if abs(value) < 800:
            value = 0
        pid_objects[key].append(value)
		
    l3_vert = statistics.fmean(pid_objects['l3_vert'])
    l3_horz = statistics.fmean(pid_objects['l3_horz'])
    r3_vert = statistics.fmean(pid_objects['r3_vert'])
    r3_horz = statistics.fmean(pid_objects['r3_horz'])
    r2 = statistics.fmean(pid_objects['r2'])
    l2 = statistics.fmean(pid_objects['l2'])
	
def threadPrint():
    while True:
        updatePIDs()
        printTable()
        time.sleep(0.04)
        retractCursor()

class MyController(Controller):
	def __init__(self, **kwargs):
		Controller.__init__(self, **kwargs)
		self.last_drive_x = 0
		self.last_drive_y = 0

	def input_filter(self, value, name):
		if name not in self.pid_objects:
			return value
		
		return self.pid_objects[name](value)

	# Left stick (tank drive)
	def on_L3_up(self, value):
		global l3_vert_pre
		l3_vert_pre = value
	
	def on_L3_down(self, value):
		global l3_vert_pre
		l3_vert_pre = value
		
	def on_L3_y_at_rest(self, ):
		global l3_vert_pre
		l3_vert_pre = 0

	def on_L3_left(self, value):
		global l3_horz_pre
		l3_horz_pre = value
		
	def on_L3_right(self, value):
		global l3_horz_pre
		l3_horz_pre = value
	
	def on_L3_x_at_rest(self, ):
		global l3_horz_pre
		l3_horz_pre = 0

	# Right stick (arm elevation)
	def on_R3_up(self, value):
		global r3_vert_pre
		r3_vert_pre = value
	
	def on_R3_down(self, value):
		global r3_vert_pre
		r3_vert_pre = value
		
	def on_R3_y_at_rest(self, ):
		global r3_vert_pre
		r3_vert_pre = 0

	def on_R3_left(self, value):
		global r3_horz_pre
		r3_horz_pre = value
		
	def on_R3_right(self, value):
		global r3_horz_pre
		r3_horz_pre = value
	
	def on_R3_x_at_rest(self, ):
		global r3_horz_pre
		r3_horz_pre = 0

	# Right trigger (arm extension)
	def on_R2_press(self, value):
		global r2_pre
		r2_pre = value

	def on_R2_release(self, ):
		global r2_pre
		r2_pre = -32768

	# Left stick (shredder control [TEMPORARY])
	def on_L2_press(self, value):
		global l2_pre
		l2_pre = value

	def on_L2_release(self, ):
		global l2_pre
		l2_pre = -32768
		

print()

# Start the thread
import threading
thread = threading.Thread(target=threadPrint)
thread.start()

controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
print(controller.listen(timeout=60))
print("Let's go!")