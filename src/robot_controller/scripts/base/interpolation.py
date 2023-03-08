import numpy

class interpolationCubic:
	def __init__(self, T, x0, x1, v0, v1):
		self.T = T
		self.D = x0
		self.C = v0
		self.A = (v1 * self.T + self.C * self.T - 2 * x1 + 2 * self.D) / self.T / self.T / self.T
		self.B = (v1 * self.T - self.C * self.T - 3 * self.A * self.T * self.T * self.T) / self.T / self.T / 2
		return

	def get_position(self, t):
		return self.A * t * t * t + self.B * t * t + self.C * t + self.D

	def get_velocity(self, t):
		return self.A * t * t * 3 + self.B * t * 2 + self.C

class interpolation_1_cos:
	def __init__(self, T, x0, x1):
		self.T = T
		self.X0 = x0
		self.X1 = x1
		return

	def get_position(self, t):
	    return self.X0 + (self.X1 - self.X0) * (1.0 - numpy.cos(numpy.pi * t / self.T)) * 0.5

	def get_velocity(self, t):
		return (self.X1 - self.X0) * numpy.sin(numpy.pi * t / self.T) * 0.5 * numpy.pi / self.T