#!/usr/bin/python
#
# dor-bug robot
#  
#
####################################################### 

from math import sin,cos,radians,degrees,pi,asin,atan,sqrt
import numpy as np
from tools import *


alpha_max = pi/2
beta_max = pi
phi_max = pi/3


small_angles = [-0.04, -0.02, -0.01, 0.0, 0.01, 0.02, 0.04 ]
big_angles = [-0.2, -0.1, -0.05, 0, 0.05, 0.1, 0.2]

MAX_ATTEMPTS = 20

class Dor():
	def __init__(self,X=0,Y=0):
		self.Ox = X
		self.Oy = Y
		self.Oz = 0
		self.R = 5
		self.legs = [ Leg(i,pi/2*i) for i in range(4) ]

		self.raised_leg = self.legs[0]
		self.legs[0].alpha += -0.01
		self.legs[2].alpha += -0.01

		self.feet_distances = self.get_dist_between_feet()
		
		self.legs[1].xy = self.get_foot(1,(0,0,0))[:2]
		self.legs[3].xy = self.get_foot(3,(0,0,0))[:2]
		self.legs[2].xy = self.get_ground_xy(2)
		

		
		self.CoM = np.array([0,0])
		_, self.CoM, _q = self.is_stable()
		print self.CoM
		
		
	
	
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

	
	def get_ground_xy(self,leg_index):
		"""returns xy coordinates for the foot having given foot_index
		in the global frame of references
		"""
		raised = self.raised_leg.index
		legs = [ i for i in range(4) if not i in[raised,leg_index] ]
		leg1 = legs[0]
		leg2 = legs[1]
		print "Legs:", leg1,leg2
		feet = self.get_feet()

		v1 = feet[leg2]-feet[leg1]
		v2 = feet[leg_index]-feet[leg1]

		d1 = self.feet_distances[(leg1,leg2)]
		d2 = self.feet_distances[(leg1,leg_index)]
		d3 = self.feet_distances[(leg_index,leg2)]
		q = (d1**2+d2**2-d3**2)/d1**2/2
		h = sqrt(d2**2-(q*d1)**2)
		#print "q,h:",q,h
		#print "v1,v2:",v1,v2
		C = q*v1
		#print "C:",C

		sign = np.cross(v1,v2)[2] > 0
		
		v1 = self.legs[leg2].xy - self.legs[leg1].xy
		#print "v1:",v1
		if sign: res = np.array([[0, -1],[1, 0]]).dot(v1)/np.linalg.norm(v1)*h
		else: res = np.array([[0, 1],[-1, 0]]).dot(v1)/np.linalg.norm(v1)*h
  		
  		print "xy:",res

		return res




	def get_dist_between_feet(self):
		distances = {}
		feet = self.get_feet()

		for i in range(len(feet)):
			for j in range(len(feet)-1):
				d = np.linalg.norm(feet[i]-feet[j])
				distances[(i,j)] = d
				distances[(j,i)] = d
		return distances


	def find_phi(self,Action):
		i = self.raised_leg.index
		xyz1 = self.get_foot((i+1)%4,Action[1])
		(da,db,_) = Action[2]
		print "\nLeg2: da,db:",da,db
		p = self.legs[(i+2)%4].phi
		A = -pi/4 - p
		B = pi/4 - p
		print "A,B:",A,B

		def fun(dp):
			xyz2 = self.get_foot((i+2)%4,(da,db,dp))
			return np.linalg.norm(xyz2-xyz1) - self.feet_distances[((i+1)%4,(i+2)%4)]
		
		if fun(A)*fun(B) > 0:
			print "Leg2: Phi cannot be found. Fa=%.1g, Fb=%.1g" % (fun(A),fun(B))
			return None
		else:
			return secant(A,B,fun(A),fun(B),fun)
		
	
	def take_action(self, action):
		"""changes state taking the given action
		Assumed that action is legal
		returns a change of CoM projection
		"""
		self.raised_leg.move_leg(action[0])
		i = self.raised_leg.index
		
		print action

		[ self.legs[(i+k)%4].move_leg(action[k]) for k in range(1,4) ]
		
		
		if self.raised_leg.index in self.get_raised_leg():
			# check CoM
			stable,NewCoM,qs = self.is_stable()
			if stable:
				# no changes
				dV0, self.CoM = NewCoM - self.CoM, NewCoM 
			else:
				# raised goes to the opposite leg
				new_raised_index = (self.raised_leg.index + 2) % 4
				print "Opposite leg is raised:", new_raised_index
				
				self.raised_leg  = self.legs[new_raised_index]
				self.legs[i].xy = self.get_ground_xy(i)
		
				
				stable,NewCoM,_ = self.is_stable()
				assert stable
				
				dV0, self.CoM = NewCoM - self.CoM, NewCoM 				
		else:
			# raised goes to the next leg
			
			new_raised1 = (self.raised_leg.index + 1) % 4
			old_raised_leg_index = self.raised_leg.index				
					
			self.raised_leg = self.legs[ new_raised1  ]
			stable,CoM1,_ = self.is_stable()
			if not stable:
				# the opposit leg is raised
				new_raised2 = (old_raised_leg_index + 3) % 4
				
				self.raised_leg = self.legs[new_raised2]
				self.legs[i].xy = self.get_ground_xy(i)
				
				stable,CoM2,_ = self.is_stable()
				assert stable, "CoM2=%s"%str(CoM2)

				dV0, self.CoM = CoM2 - self.CoM, CoM2 
			else:
				# both could be stable
				self.raised_leg = self.legs[ (old_raised_leg_index + 3) % 4  ]
				stable,CoM2,_ = self.is_stable()

				if not stable:
					# the first option is true
					self.raised_leg = self.legs[ (old_raised_leg_index + 1) % 4   ]
					self.legs[i].xy = self.get_ground_xy(i)
		
					dV0, self.CoM = CoM1 - self.CoM, CoM1 
				else:
					# both stable
					if np.linalg.norm(CoM1 - self.CoM) < np.linalg.norm(CoM2 - self.CoM):
						self.raised_leg = self.legs[ (old_raised_leg_index + 1) % 4  ]
						self.legs[i].xy = self.get_ground_xy(i)
		
						dV0, self.CoM = CoM1 - self.CoM, CoM1 
					else:
						self.raised_leg = self.legs[ (old_raised_leg_index + 3) % 4  ]
						self.legs[i].xy = self.get_ground_xy(i)
		
						dV0, self.CoM = CoM2 - self.CoM, CoM2 
		
		self.feet_distances = self.get_dist_between_feet()
		return dV0	

				
	

	def get_random_action(self):
		Action = {}
		
		i = self.raised_leg.index
		Action[0] = get_random_action(self.legs[i],big_angles)		
		
		N = 0
		while True:
			if N > MAX_ATTEMPTS:
				print "\n\tCannot move any standing leg at the current state\nFeet:", self.get_feet()
				Action[1] = (0,0,0)
				Action[2] = (0,0,0)
				Action[3] = (0,0,0)
				return Action

			Action[1] = get_random_action(self.legs[(i+1)%4],small_angles)
			Action[2] = get_random_action(self.legs[(i+2)%4],small_angles)
					
			# find phi
			dp = self.find_phi(Action)
			if not self.legs[(i+2)%4].phi_is_legal(dp):
				print "dPhi =",dp
				print "Phi found is illegal"
				N += 1
				continue
			da,db,_ = Action[2]
			Action[2] = (da,db,dp)
				
			d1 = self.feet_distances[((i+3)%4, (i+2)%4)]
			d2 = self.feet_distances[((i+3)%4, (i+1)%4)]
			d3 = self.feet_distances[((i+2)%4, (i+1)%4)]
			print "d1,d2,d3:",d1,d2,d3
			q1 = (d1**2 - d2**2 + d3**2) / d3 / 2
			
			# the center of the circle which the foot of the last leg lays on
			foot1 = self.get_foot((i+1)%4, Action[1])
			foot2 = self.get_foot((i+2)%4, Action[2])
			
			r0 = (q1 * foot1 + (d3-q1) * foot2 ) / d3
			#d0_square = d1**2 - q1**2
			v0 = foot2 - foot1
			#print "r0, v0:", r0, v0 

			attempts = 5
			leg = self.legs[(i+3)%4]
				
			while attempts > 0:

				dp = choose_randomly(big_angles)
				
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
							print "Line is too far or too close\nAttempts remained:", attempts-1
							attempts -= 1
							continue
					else:
						t3 = secant(A,B,Fa,Fb,fun)
				else:
					t3 = secant(A,B,Fa,Fb,fun)

				if t3 == None:
					print "Phi not found\nAttempts remained:",attempts-1
					attempts -= 1
					continue
				else:
					foot3 = r_line + v_line*t3
					#print 'foot3:', foot3
					break

			else: # no solution found
				print "This random action is illegal\n"
				N += 1
				continue

			# find alpha and beta
			joint3 = np.array([self.R * cos(leg.aa),
								self.R * sin(leg.aa),
								0])
			delta = atan(-foot3[2]/sqrt((joint3[0]-foot3[0])**2 + (joint3[1]-foot3[1])**2))
			#print "delta:", degrees(delta)
			foot_distance = np.linalg.norm(foot3-joint3)
			if foot_distance > 2*leg.L:
				print "legal da or db cannot be found\nda,da:",da,db
				print "leg3: a, b, phi:", leg.alpha, leg.beta, leg.phi
				N += 1
				continue
			else:
				beta = 2 * asin(foot_distance/leg.L/2)

			#print "beta:",degrees(beta)
			alpha = delta + beta/2
			#print "alpha:",degrees(alpha)
			da = alpha - leg.alpha
			db = beta - leg.beta
			if leg.alpha_is_legal(da) and leg.beta_is_legal(db):
				Action[3] = (da,db,dp)
				break
			else:
				print "legal da or db cannot be found\nda,da:",da,db
				print "leg3: a, b, phi:", leg.alpha, leg.beta, leg.phi
				N += 1
				continue
		
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
			
			return [1, 3]
		else:
			print "0th and 2nd legs can be raised\n"
			return [0, 2]
	

	def is_stable(self):
		"""returns tuple. First element is True or Flase if robot can stand on its tree legs or not
		the second element is a projection od centre of mass onto the plane of three feet
		the third is a tuple of three q - a load to each leg
		"""
		feet = self.get_feet()
		f1,f2,f3 = tuple([ feet[i] for i in range(4) if i != self.raised_leg.index])
		#f1 = self.get_foot(self.first_leg.index, (0,0,0))
		#f2 = self.get_foot(self.second_leg.index, (0,0,0))
		#f3 = self.get_foot(self.third_leg.index, (0,0,0))

		v1 = f1-f2
		v2 = f3-f2
		ez1 = np.cross(v2,v1) / (np.linalg.norm(v2) * np.linalg.norm(v1) )
		X0,Y0 = (ez1 * np.dot(ez1,f1))[:2]
		print "X0,Y0",X0,Y0
		X1,Y1 = f1[0],f1[1]
		X2,Y2 = f2[0],f2[1]
		X3,Y3 = f3[0],f3[1]
		print "Feet:", f1[:2],f2[:2],f3[:2]

		TX0 = (X0-X3)/(X1-X3)
		TX2 = (X2-X3)/(X1-X3)
		
		q2 = ( TX0 - (Y0-Y3)/(Y1-Y3) )/( TX2 - (Y2-Y3)/(Y1-Y3) )
		q1 =  TX0 - TX2 * q2
		q3 = 1 - (q1 + q2)

		print "q =", q1,q2,q3 

		if q1>0 and q2>0 and q3>0: return (True, np.array([X0,Y0]), (q1,q2,q3))
		else: return (False,np.array([X0,Y0]),(q1,q2,q3))





class Leg():
	def __init__(self,index,attach_angle,alpha=radians(30),beta=radians(45),phi=0,L=10):
		self.index = index
		self.aa = attach_angle
		self.alpha = alpha
		self.beta = beta
		self.phi = phi
		self.L = L
		
	
	def get_copy(self):
		copy = Leg(self.index,self.aa, self.alpha, self.beta, self.phi, self.L)
		return copy


	def move_leg(self,(da,db,dp)):
		self.alpha += da
		self.beta += db
		self.phi += dp


	def get_foot(self):
		"""returns a xz coordinate of a foot in leg's own reference system
		"""
		return np.array([ self.L * ( sin(self.alpha) + sin(self.beta-self.alpha) ),
						self.L * ( cos(self.alpha) - cos(self.beta-self.alpha) ) ])
	
	def alpha_is_legal(self,da):
		a = self.alpha + da
		return a >= pi/15 and a < pi/1.5

	def beta_is_legal(self,db):
		b = self.beta + db
		return b >= pi/12 and b < 2 * self.alpha

	def phi_is_legal(self,dp):
		if dp == None:
			return False

		p = self.phi + dp
		return abs(p) < pi/4

	


