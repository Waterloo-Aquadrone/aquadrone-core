import numpy as np


class Vector:
	def __init__(self, x, y, z):
		self.x = x
		self.y = y
		self.z = z


	def magnitude(self):
		return np.linalg.norm([x_i for x_i in [self.x, self.y, self.z] if x_i is not None])

	
	def mean(*vectors):
		x = sum(v.x for v in vectors) / len(vectors)
		y = sum(v.y for v in vectors) / len(vectors)
		z = sum(v.z for v in vectors) / len(vectors)
		return Vector(x, y, z)


	def std_mean(*vectors):
		"""
		:@return: the component-wise standard deviation of the mean of a set of vectors with the given standard deviations
		"""
		x = (sum(v.x ** 2 for v in vectors) ** 0.5) / len(vectors)
		y = (sum(v.y ** 2 for v in vectors) ** 0.5) / len(vectors)
		z = (sum(v.z ** 2 for v in vectors) ** 0.5) / len(vectors)
		return Vector(x, y, z)


class Submarine:
	def __init__(self, position, direction, velocity, angular_velocity):
		self.position = position
		self.direction = direction
		self.velocity = velocity
		self.angular_velocity = angular_velocity
		self.position_std = None
		self.direction_std = None
		self.velocity_std = None
		self.angular_velocity_std = None


	def set_uncertainties(self, position, direction, velocity, angular_velocity):
		self.position_std = position
		self.direction_std = direction
		self.velocity_std = velocity
		self.angular_velocity_std = angular_velocity


class Gate:
	def __init__(self, top_corners, bottom_corners):
		self.top_corners = top_corners
		self.bottom_corners = bottom_corners
		self.position = Vector.mean(top_corners[0], top_corners[1], bottom_corners[0], bottom_corners[1])


	def set_uncertainties(self, top_corners, bottom_corners):
		self.top_corners_std = top_corners
		self.bottom_corners_std = bottom_corners
		self.position_std = Vector.std_mean(top_corners[0], top_corners[1], bottom_corners[0], bottom_corners[1])


class Pole:
	def __init__(self, x, y):
		self.postition = Vector(x, y, None)
		self.position_std = None


	def set_uncertainties(x, y):
		self.position_std = Vector(x, y, None)


class Wall:
	def __init__(self, m, b):
		# y = mx + b
		self.m = m
		self.b = b
		self.m_std = None
		self.b_std = None


	def set_uncertainties(self, m, b):
		self.m_std = None
		self.b_std = None


class CompetitionMap:
	def __init__(self, sub):
		self.sub = sub
		self.gate = None
		self.walls = []


	def set_gate(gate):
		self.gate = gate


	def add_wall(wall):
		self.walls.append(wall)


class PID:
	off_state =  {'x' : None, 'y' : None, 'z' : None, 'r' : None, 'p' : None, 'y' : None}


	def __init__(self):
		self.desired_state = PID.off_state


	def set_desired_state(self, mode, values):
		# TODO: add ROS code to subscribe to a topic and call this function
		# Called whenever there is a new desired state for the PID to aim for
		self.desired_state = PID.off_state
		for m, val in zip(mode, values):
			self.desired_state[m] = val


	def process_data(sub):
		# TODO: add ROS code to subscribe to a topic and call this function
		# Called whenever there is updated location data from localization and mapping.
		pass


	def apply_thrust():
		# TODO: add ROS code to publish to a topic when calling this function
		pass


class PathPlanner:
	def __init__(self):
		self.state = 'GATE'
		self.searching_tolerance = 5  # the maximum standard deviation in position of an object before it is considered lost
		self.searched_areas = []


	def process_business_command(self, state):
		"""
		TODO: add ROS code to subscribe to a topic and call this function
		Called whenever there is an updated high level task from the business/competition logic
		"""
		self.state = state


	def process_data(self, competition_map):
		"""
		# TODO: add ROS code to subscribe to a topic and call this function
		Called whenever there is updated location data from localization and mapping.
		"""
		if self.state == 'GATE':
			if competition_map.gate is None or competition_map.gate.position_std.magnitude() > self.searching_tolerance:
				search_for_gate(competition_map)
			else:
				go_through_gate(sub, gate)
		else if self.state == 'POLE':
			if competition_map.pole is None or competition_map.pole.position_std.magnitude() > self.searching_tolerance:
				search_for_pole(competition_map)
			else:
				go_around_pole(sub, pole)
		else:
			raise Exception('Unknown state: ' + self.state)

	
	def search_for_gate(competition_map):
		# Example code:
		sub = competition_map.sub
		current_x = sub.position.x
		deisred_x = current_x + 5 ## meters
		send_pid_command('xyzrp', [deisred_x, sub.position.y, sub.position.z, 0, 0])


	def go_through_gate(sub, gate):
		"""
		Given sub's current 
		"""
		pass


	def search_for_pole(competition_map):
		pass


	def go_around_pole(sub, pole):
		pass


	def send_pid_command(mode, values):
		"""
		# TODO: add ROS code to publish to a topic when calling this function
		"""
		pass
