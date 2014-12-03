#!/usr/bin/python
#
# dor-bug robot
#  
#
####################################################### 

from math import sin,cos,radians,degrees,pi,asin,atan,sqrt
import numpy as np
from random import random,seed
from tools import choose_randomly, secant, TOLERANCE

alpha_max = pi/2
beta_max = pi
phi_max = pi/3


small_angles = [-0.05, -0.02, -0.01, 0.0, 0.01, 0.02, 0.05 ]
big_angles = [-0.2, -0.1, -0.05, 0, 0.05, 0.1, 0.2]




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
		i = 0
		while i < len(self.legs):
			leg = self.legs[i]
			#print "Leg:",leg.index
			if leg.raised:
				da = choose_randomly(big_angles)
				while not leg.alpha_is_legal(da):
					da = choose_randomly(big_angles)	
				
				db = choose_randomly(big_angles)
				while not leg.beta_is_legal(db):
					db = choose_randomly(big_angles)	
							
				dp = choose_randomly(big_angles)
				while not leg.phi_is_legal(dp):
					dp = choose_randomly(big_angles)	
				
				Action[0] = (da,db,dp)
				#print "Action0:",Action[0]
			
			elif cur_leg == 1:
				self.first_leg = leg
				da = choose_randomly(small_angles)
				while not leg.alpha_is_legal(da):
					da = choose_randomly(small_angles)	
					#print "da:", da, leg.alpha_is_legal(da)
				
				db = choose_randomly(small_angles)
				while not leg.beta_is_legal(db):
					db = choose_randomly(small_angles)	
					#print "db:", db, leg.beta+db
							
				dp = choose_randomly(small_angles)
				while not leg.phi_is_legal(dp):
					dp = choose_randomly(small_angles)	
					#print "dp:", dp, leg.phi+dp
				
				Action[1] = (da,db,dp)
				#print "Action1:",Action[1]
				cur_leg = 2

			elif cur_leg == 2:
				self.second_leg = leg
				da = choose_randomly(small_angles)
				while not leg.alpha_is_legal(da):
					da = choose_randomly(small_angles)	
				
				db = choose_randomly(small_angles)
				while not leg.beta_is_legal(db):
					db = choose_randomly(small_angles)	
				
				# find phi
				dp = self.find_phi(Action[1],(da,db))
				if not leg.phi_is_legal(dp):
					print "This random action is illegal\n"
					cur_leg = 1
					i = 0
					continue

				Action[2] = (da,db,dp)
				#print "Action2:", Action[2]
				cur_leg = 3

			elif cur_leg == 3:
				self.third_leg = leg
				d1 = self.feet_distances[(leg.index,self.second_leg.index)]
				d2 = self.feet_distances[(leg.index,self.first_leg.index)]
				d3 = self.feet_distances[(self.second_leg.index,self.first_leg.index)]
				#print "d1,d2,d3:",d1,d2,d3
				q1 = (d1**2 - d2**2 + d3**2) / d3 / 2
				
				# the center of the circle which the foot of the last leg lays on
				foot1 = self.get_foot(self.first_leg.index,Action[1])
				foot2 = self.get_foot(self.second_leg.index,Action[2])
				#print "foot1:",foot1
				#print "foot2:",foot2
				r0 = (q1 * foot1 + (d3-q1) * foot2 ) / d3
				#d0_square = d1**2 - q1**2
				v0 = foot2 - foot1
				#print "r0, v0:", r0, v0 

				attempts = 5
				while attempts > 0:
					dp = choose_randomly(small_angles)
					#print "dp:",dp
					#print "aa,phi:", leg.aa, leg.phi
					
					# leg's plane
					v3 = np.array([-sin(leg.aa+leg.phi+dp), 
									cos(leg.aa+leg.phi+dp),
									0])
					r3 = np.array([self.R * cos(leg.aa),
									self.R * sin(leg.aa),
									0])
					#print "r3:", r3, v3
					v_line = np.cross(v0,v3)
					x_line = (np.dot(r3,v3)*v0[1] - np.dot(r0,v0)*v3[1])/(v3[0]*v0[1] - v0[0]*v3[1])
					r_line = np.array([ x_line,
										(np.dot(r0,v0) - x_line*v0[0])/v0[1],
										0 ])
					#print "r_line, v_line:", r_line, v_line
					z1 = r_line[2]+v_line[2]
					if z1 > 0:
						t1 = - leg.L/z1/2
					else: 
						t1 = leg.L/z1/2

					def fun(t):
						dist = np.linalg.norm(r_line + v_line * t - foot2) - d1
						return dist

					A = 0
					B = t1
					Fa = fun(A)
					Fb = fun(B)
					if Fa*Fb > 0:
						B = B/2
						Fb = fun(B)
						if Fa*Fb > 0:
							if abs(Fa) < TOLERANCE: t3 = A
							elif abs(Fb) < TOLERANCE: t3 = B
							else:
								print "Solution not found\nAttempts remained:", attempts-1
								attempts -= 1
								continue
						else:
							t3 = secant(A,B,Fa,Fb,fun)
					else:
						t3 = secant(A,B,Fa,Fb,fun)

					if t3 == None:
						print "Solution not found\nAttempts remained:",attempts-1
						attempts -= 1
						continue
					else:
						foot3 = r_line + v_line*t3
						#print 'foot3:', foot3
						break

				else: # no solution found
					print "This random action is illegal\n"
					cur_leg = 1
					i = 0
					continue

				# find alpha and beta
				joint3 = np.array([self.R * cos(leg.aa),
									self.R * sin(leg.aa),
									0])
				delta = atan(-foot3[2]/sqrt((joint3[0]-foot3[0])**2 + (joint3[1]-foot3[1])**2))
				#print "delta:", degrees(delta)
				beta = 2 * asin(np.linalg.norm(foot3-joint3)/leg.L/2)
				#print "beta:",degrees(beta)
				alpha = delta + beta/2
				#print "alpha:",degrees(alpha)
				da = alpha - leg.alpha
				db = beta - leg.beta
				if leg.alpha_is_legal(da) and leg.beta_is_legal(db):
					Action[3] = (da,db,dp)
					#print "Action3:",Action[3]
				else:
					i = 1
					cur_leg = 1
					continue
			i += 1
					
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
	def __init__(self,index,attach_angle,alpha=radians(30),beta=radians(45),phi=0,L=10):
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
	print D.get_random_action()

	








