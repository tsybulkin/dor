#!/usr/bin/python
#
# dor-bug robot
#  
#
####################################################### 

from math import sin,cos,radians,pi
import numpy as np
from random import random,seed
from tools import choose_randomly, secant

alpha_max = pi/2
beta_max = pi
phi_max = pi/3


Angle_changes = [-0.05, -0.02, -0.01, 0.0, 0.01, 0.02, 0.05 ]




class Dor():
	def __init__(self,X=0,Y=0):
		self.Ox = X
		self.Oy = Y
		self.Oz = 0
		self.R = 10
		self.legs = [ Leg(i,pi/2*i) for i in range(4) ]
		self.legs[0].alpha += -0.01
		self.legs[2].alpha += -0.01
		self.legs[0].raised = True
		self.feet_distances = self.get_dist_between_feet()

	
	
	def get_foot(self, leg_index, (da,db,dp)):
		Leg = self.legs[leg_index].get_copy()
		Leg.alpha += da
		Leg.beta += db
		Leg.phi += dp

		return np.array([self.R * cos(Leg.aa) + Leg.get_foot()[0] * cos(Leg.aa+Leg.phi),
						self.R * sin(Leg.aa) + Leg.get_foot()[0] * sin(Leg.aa+Leg.phi),
						Leg.get_foot()[1]])


	def get_feet(self):
		"""returns a list of foot coordinates in its own reference system
		""" 
		return [ np.array([ self.R * cos(Leg.aa) + Leg.get_foot()[0] * cos(Leg.aa+Leg.phi),
							self.R * sin(Leg.aa) + Leg.get_foot()[0] * sin(Leg.aa+Leg.phi),
							Leg.get_foot()[1] ]) for Leg in self.legs]

	
	def get_dist_between_feet(self):
		distances = {}
		feet = self.get_feet()

		for i in range(len(feet)):
			for j in range(len(feet)-1):
				d = np.linalg.norm(feet[i]-feet[j])
				distances[(i,j)] = d
				distances[(j,i)] = d
		return distances


	def find_phi(self,leg1_move,leg2_move):
		first_leg = self.first_leg.index
		second_leg = self.second_leg.index
		xyz1 = self.get_foot(first_leg,leg1_move)
		(da,db) = leg2_move
		p = self.legs[second_leg].phi
		A = -pi/4 - p
		B = pi/4 - p
		
		def fun(dp):
			xyz2 = self.get_foot(second_leg,(da,db,dp))
			return np.linalg.norm(xyz2-xyz1) - self.feet_distances[(first_leg,second_leg)]
		
		return secant(A,B,fun(A),fun(B),fun)
		
		

	def get_random_action(self):
		Action = {}
		cur_leg = 1
		for leg in self.legs:
			print "Leg:",leg.index
			if leg.raised:
				da = choose_randomly(Angle_changes)
				while not leg.alpha_is_legal(da):
					da = choose_randomly(Angle_changes)	
				
				db = choose_randomly(Angle_changes)
				while not leg.beta_is_legal(db):
					db = choose_randomly(Angle_changes)	
							
				dp = choose_randomly(Angle_changes)
				while not leg.phi_is_legal(dp):
					dp = choose_randomly(Angle_changes)	
				
				Action[0] = (da,db,dp)
				print "Action0:",Action[0]
			
			elif cur_leg == 1:
				self.first_leg = leg
				da = choose_randomly(Angle_changes)
				while not leg.alpha_is_legal(da):
					da = choose_randomly(Angle_changes)	
					print "da:", da, leg.alpha_is_legal(da)
				
				db = choose_randomly(Angle_changes)
				while not leg.beta_is_legal(db):
					db = choose_randomly(Angle_changes)	
					print "db:", db, leg.beta+db
							
				dp = choose_randomly(Angle_changes)
				while not leg.phi_is_legal(dp):
					dp = choose_randomly(Angle_changes)	
					print "dp:", dp, leg.phi+dp
				
				Action[1] = (da,db,dp)
				print "Action1:",Action[1]
				cur_leg = 2

			elif cur_leg == 2:
				self.second_leg = leg
				da = choose_randomly(Angle_changes)
				while not leg.alpha_is_legal(da):
					da = choose_randomly(Angle_changes)	
				
				db = choose_randomly(Angle_changes)
				while not leg.beta_is_legal(db):
					db = choose_randomly(Angle_changes)	
				
				# find phi
				dp = self.find_phi(Action[1],(da,db))
				if not leg.phi_is_legal(dp):
					print "This random action is illegal\n"
					cur_leg = 1
					continue

				Action[2] = (da,db,dp)
				print "Action2:", Action[2]
				cur_leg = 3

			elif cur_leg == 3:
				self.third_leg = leg
				d1 = self.feet_distances[(leg.index,self.second_leg.index)]
				d2 = self.feet_distances[(leg.index,self.first_leg.index)]
				d3 = self.feet_distances[(self.second_leg.index,self.first_leg.index)]
				print "d1,d2,d3:",d1,d2,d3
				q1 = (d1**2 - d2**2 + d3**2) / d3 / 2
				
				# the center of the circle which the foot of the last leg lays on
				foot1 = self.get_foot(self.first_leg.index,Action[1])
				foot2 = self.get_foot(self.second_leg.index,Action[2])
				r0 = (q1 * foot1 + (d3-q1) * foot2 ) / d3
				d0_square = d1**2 - q1**2
				v0 = foot2 - foot1
				print "r0:", r0, d0_square

				attepts = 5
				while attepts > 0:
					dp = choose_randomly(Angle_changes)
					
					# leg's plane
					v3 = np.array([-sin(leg.aa+leg.phi+dp), 
									cos(leg.aa+leg.phi+dp),
									0])
					r3 = np.array([self.R * cos(leg.aa),
									self.R * sin(leg.aa),
									0])
					print "r3:", r3, v3
					v_line = np.cross(v0,v3)
					print "v_line:",v_line
					break	

				#while not leg.phi_is_legal(dp):
					#dp = choose_randomly(Angle_changes)	
				
				# find alpha and beta
				da = 0
				db = 0
				Action[3] = (da,db,dp)
				print "Action3:",Action[3]

		return Action


	def get_raised_leg(self):
		feet = self.get_feet()
		self.feet = feet

		v1 = feet[-1]-feet[0]
		v2 = feet[1]-feet[0]
		v3 = feet[2]-feet[0]
		dot = np.dot(v3, np.cross(v1,v2) )
		if dot == 0:
			print "all legs touch the surface\n"
			return []
		elif dot > 0:
			print "1st and 3rd legs can be raised\n"
			v1 = feet[0]-feet[1]
			v2 = feet[2]-feet[1]
			ez1 = np.cross(v2,v1) / (np.linalg.norm(v2) * np.linalg.norm(v1) )
			#ez1 = np.cross(v1,v2) / (np.linalg.norm(v2) * np.linalg.norm(v1) )
			
			print "Oxy projection to the ground plane =", ez1 * np.dot(ez1,feet[0])
			
			X0,Y0 = (ez1 * np.dot(ez1,feet[0]))[:2]
			print "X0,Y0",X0,Y0
			X1,Y1 = feet[0][0],feet[0][1]
			X2,Y2 = feet[1][0],feet[1][1]
			X3,Y3 = feet[2][0],feet[2][1]
			print "Feet:", feet[0][:2],feet[1][:2],feet[2][:2]

			TX0 = (X0-X3)/(X1-X3)
			TX2 = (X2-X3)/(X1-X3)
			
			q2 = ( TX0 - (Y0-Y3)/(Y1-Y3) )/( TX2 - (Y2-Y3)/(Y1-Y3) )
			q1 =  TX0 - TX2 * q2
			q3 = 1 - (q1 + q2)

			print "q =", q1,q2,q3 

			

			v1 = feet[0]-feet[3]
			v2 = feet[2]-feet[3]
			ez1 = np.cross(v1,v2) / (np.linalg.norm(v2) * np.linalg.norm(v1) )
			print "ez1 =", ez1

			return [1, 3]
		else:
			print "0th and 2nd legs can be raised\n"
			return [0, 2]
	


class Leg():
	def __init__(self,index,attach_angle,alpha=radians(20),beta=radians(30),phi=0,L=10):
		self.index = index
		self.aa = attach_angle
		self.alpha = alpha
		self.beta = beta
		self.phi = phi
		self.L = L
		self.raised = False

	
	def get_copy(self):
		copy = Leg(self.index,self.aa, self.alpha, self.beta, self.phi, self.L)
		copy.raised = self.raised
		return copy


	def get_foot(self):
		"""returns a xz coordinate of a foot in leg's own reference system
		"""
		return np.array([ self.L * ( sin(self.alpha) + sin(self.beta-self.alpha) ),
						self.L * ( cos(self.alpha) - cos(self.beta-self.alpha) ) ])
	
	def alpha_is_legal(self,da):
		a = self.alpha + da
		return a >= pi/15 and a < pi/3

	def beta_is_legal(self,db):
		b = self.beta + db
		return b >= pi/12 and b < 2 * self.alpha

	def phi_is_legal(self,dp):
		p = self.phi + dp
		return abs(p) < pi/4



if __name__ == '__main__':
	seed()
	D = Dor()
	#D.legs[0].alpha = 0.35
	#D.legs[0].phi = 0.2
	D.get_raised_leg()
	D.get_random_action()

	








