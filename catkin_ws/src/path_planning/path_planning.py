import numpy as np

class Vector:
	def __init__(self, x, y, z):
		self.x = x
		self.y = y 
		self.z = z

	def mean(*vectors):
		x = sum(v.x for v in vectors) / len(vectors)
		y = sum(v.y for v in vectors) / len(vectors)
		z = sum(v.z for v in vectors) / len(vectors)
		return Vector(x, y, z)

class Submarine:
	def __init__(self, position, direction, velocity, angular_velocity):
		self.position = position
		self.direction = direction
		self.velocity = velocity
		self.angular_velocity = angular_velocity

class Gate:
	def __init__(self, top_corners, bottom_corners):
		self.top_corners = top_corners
		self.bottom_corners = bottom_corners
		self.position = Vector.mean(top_corners[0], top_corners[1], bottom_corners[0], bottom_corners[1])

class PID:
	off_state =  {'x' : None, 'y' : None, 'z' : None, 'r' : None, 'p' : None, 'y' : None}
	def __init__(self):
		self.desired_state = PID.off_state

	def set_desired_state(self, mode, values):
		# TODO: add ROS code to subscribe to a topic and call this function
		# Called whenever there is a new desired state for the PID to aim for
		self.desired_state = PID.off_state
		for i in range(len(mode)):
			self.desired_state[mode[i]] = values[i]

	def process_data(sub):
		# TODO: add ROS code to subscribe to a topic and call this function
		# Called whenever there is updated location data from localization and mapping.
		pass

	def apply_thrust():
		# TODO: add ROS code to publish to a topic when calling this function
		pass

class GatePathPlanner:
	def __init__(self):
		self.sub = None
		self.gate = None

	def process_data(sub, gate):
		"""
		# TODO: add ROS code to subscribe to a topic and call this function
		Called whenever there is updated location data from localization and mapping.
		"""
		pass

	def send_pid_command(mode, values):
		"""
		# TODO: add ROS code to publish to a topic when calling this function
		"""
		pass
