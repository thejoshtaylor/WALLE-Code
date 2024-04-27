from pyPS4Controller.controller import Controller
from simple_pid import PID

class MyController(Controller):
	def __init__(self, **kwargs):
		Controller.__init__(self, **kwargs)
		self.last_drive_x = 0
		self.last_drive_y = 0
		
		self.pid_objects = {
			'l3_vert': PID(0.9, 0.001, 0.1),
			'l3_horz': PID(0.9, 0.001, 0.1),

			'r3_vert': PID(0.9, 0.001, 0.1),
			'r3_horz': PID(0.9, 0.001, 0.1),

			'r2': PID(0.9, 0.001, 0.1),
			'l2': PID(0.9, 0.001, 0.1),
		}

	def input_filter(self, value, name):
		if name not in self.pid_objects:
			return value
		
		return self.pid_objects[name](value)

	# Left stick (tank drive)
	def on_L3_up(self, value):
		print('L3_vert', value, self.input_filter(value, 'l3_vert'))
	
	def on_L3_down(self, value):
		print('L3_vert', value, self.input_filter(value, 'l3_vert'))
		
	def on_L3_y_at_rest(self, ):
		print('L3_vert', 0, self.input_filter(0, 'l3_vert'))

	def on_L3_left(self, value):
		print('L3_horz', value, self.input_filter(value, 'l3_horz'))
		
	def on_L3_right(self, value):
		print('L3_horz', value, self.input_filter(value, 'l3_horz'))
	
	def on_L3_x_at_rest(self, ):
		print('L3_horz', 0, self.input_filter(0, 'l3_horz'))

	# Right stick (arm elevation)
	def on_R3_up(self, value):
		print('R3_vert', value, self.input_filter(value, 'r3_vert'))
	
	def on_R3_down(self, value):
		print('R3_vert', value, self.input_filter(value, 'r3_vert'))
		
	def on_R3_y_at_rest(self, ):
		print('R3_vert', 0, self.input_filter(0, 'r3_vert'))

	def on_R3_left(self, value):
		print('R3_horz', value, self.input_filter(value, 'r3_horz'))
		
	def on_R3_right(self, value):
		print('R3_horz', value, self.input_filter(value, 'r3_horz'))
	
	def on_R3_x_at_rest(self, ):
		print('R3_horz', 0, self.input_filter(0, 'r3_horz'))

	# Right trigger (arm extension)
	def on_R2_press(self, value):
		print('R2', value, self.input_filter(value, 'r2'))

	def on_R2_release(self, ):
		print('R2', 0, self.input_filter(0, 'r2'))

	# Left stick (shredder control [TEMPORARY])
	def on_L2_press(self, value):
		print('L2', value, self.input_filter(value, 'l2'))

	def on_L2_release(self, ):
		print('L2', 0, self.input_filter(0, 'l2'))
		

controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
print(controller.listen(timeout=60))
print("Let's go!")