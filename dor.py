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

		self.raised_leg = 0
		self.legs[0].alpha += -0.01
		self.legs[2].alpha += -0.01

		self.feet_distances = self.get_dist_between_feet()
		
		self.legs[1].xy = self.get_foot(1,(0,0,0))[:2]
		self.legs[3].xy = self.get_foot(3,(0,0,0))[:2]
		self.legs[2].xy = self.get_ground_xy(2)
		
		self.CoM = np.array([0,0])
		self.orientation = 0.0
		_, self.CoM, _q = self.is_stable()
		
		
	
	
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
		Matrix = np.array([[0, 1],[-1, 0]])
		#print "leg:",leg_index,
		raised = self.raised_leg
		
		leg1 = (leg_index+3)%4
		leg2 = (leg_index+1)%4
		#print "Legs:", leg1,leg2
		feet = self.get_feet()

		v1 = feet[leg2]-feet[leg1]
		e1 = v1 / np.linalg.norm(v1)
		#e2 = Matrix.dot(e1)
		#v2 = feet[leg_index]-feet[leg1]

		d3 = self.feet_distances[(leg1,leg2)]
		d1 = self.feet_distances[(leg1,leg_index)]
		d2 = self.feet_distances[(leg_index,leg2)]
		q = (d3**2+d1**2-d2**2)/d3**2/2
		h = sqrt(d1**2-(q*d3)**2)
		#print "q,h:",q,h
		#print "v1,v2:",v1,v2
		#print q*v1+h*e2, "=", feet[leg_index]
		#print "C:",C

		v11 = self.legs[leg2].xy - self.legs[leg1].xy
		e11 = v11/np.linalg.norm(v11)
		e22 = Matrix.dot(e11)
		#print "e11,e22:",e11,e22
		
		res = self.legs[leg1].xy + q*v11 + h*e22
  		#print "xy =",res
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
		i = self.raised_leg
		xyz1 = self.get_foot((i+1)%4,Action[1])
		(da,db,_) = Action[2]
		#print "\nLeg2: da,db:",da,db
		p = self.legs[(i+2)%4].phi
		A = -pi/4 - p
		B = pi/4 - p
		#print "A,B:",A,B

		def fun(dp):
			xyz2 = self.get_foot((i+2)%4,(da,db,dp))
			return np.linalg.norm(xyz2-xyz1) - self.feet_distances[((i+1)%4,(i+2)%4)]
		
		if fun(A)*fun(B) > 0:
			#print "Leg2: Phi cannot be found. Fa=%.1g, Fb=%.1g" % (fun(A),fun(B))
			return None
		else:
			return secant(A,B,fun(A),fun(B),fun)
		
	
	def take_action(self, action):
		"""changes state taking the given action
		Assumed that action is legal
		"""
		raised = self.raised_leg
		self.legs[raised].move_leg(action[0])
		
		#print action

		[ self.legs[(raised+k)%4].move_leg(action[k]) for k in range(1,4) ]
		
		
		if self.raised_leg in self.get_raised_leg():
			# check CoM
			stable,CoM,qs = self.is_stable()
			if stable:
				# no changes
				self.CoM = CoM 
			else:
				# raised goes to the opposite leg
				self.raised_leg  = (raised + 2) % 4
				#print "Opposite leg is raised:", self.raised_leg 
				self.legs[raised].xy = self.get_ground_xy(raised)
				
				stable,CoM,qs = self.is_stable()
				assert stable, "qs:%s" % str(qs) 
				
				self.CoM = CoM 				
		else:
			# raised goes to the next leg
			
			self.raised_leg = (raised + 1) % 4
			self.legs[raised].xy = self.get_ground_xy(raised)
		
			stable,CoM1,_ = self.is_stable()
			if not stable:
				# the opposit leg is raised
				
				self.raised_leg = (raised + 3) % 4
				#self.legs[i].xy = self.get_ground_xy(i)
				
				stable,CoM2,qs = self.is_stable()
				assert stable, "feet: %s" % str(self.get_feet())

				self.CoM = CoM2 
			else:
				# both could be stable
				self.raised_leg = (raised + 3) % 4 
				#self.legs[i].xy = self.get_ground_xy(i)
		
				stable,CoM2,_ = self.is_stable()

				if not stable:
					# the first option is true
					self.raised_leg = (raised + 1) % 4  
					#self.legs[i].xy = self.get_ground_xy(i)
		
					self.CoM = CoM1 
				else:
					# both stable
					if np.linalg.norm(CoM1 - self.CoM) < np.linalg.norm(CoM2 - self.CoM):
						self.raised_leg = (raised + 1) % 4 
						#self.legs[i].xy = self.get_ground_xy(i)
		
						self.CoM = CoM1 
					else:
						self.raised_leg = (raised + 3) % 4 
						#self.legs[i].xy = self.get_ground_xy(i)
		
						self.CoM = CoM2 
		
		self.update_orientation()
		self.feet_distances = self.get_dist_between_feet()
		
				
	def get_move(self, Action):
		i = self.raised_leg
		
		Res = {0:Action[0], 1:Action[1], 2:Action[2]+(0,)}
		dp = self.find_phi(Res)
		if dp == None: return self.get_random_action()

		Res[2] = Action[2]+(dp,)

		foot3 = find_foot3(self,i,Action[3],Res)
		if foot3 == None:
			return self.get_random_action()

		a_b = find_alpha_beta(self,i,foot3)
		if a_b == None:
			return self.get_random_action()
		
		alpha, beta = a_b
		leg = self.legs[(i+3)%4]
		da = alpha - leg.alpha
		db = beta - leg.beta
		if leg.alpha_is_legal(da) and leg.beta_is_legal(db):
			Res[3] = (da,db,dp)
		else:
			return self.get_random_action()

		return Res
		


	def get_random_action(self):
		Action = {}
		
		i = self.raised_leg
		Action[0] = get_random_action(self.legs[i],big_angles)		
		
		N = 0
		while True:
			if N > MAX_ATTEMPTS:
				#print "Cannot move any standing leg at the current state"
				Action[1] = (0,0,0)
				Action[2] = (0,0,0)
				Action[3] = (0,0,0)
				return Action

			Action[1] = get_random_action(self.legs[(i+1)%4],small_angles)
			Action[2] = get_random_action(self.legs[(i+2)%4],small_angles)
					
			# find phi
			dp = self.find_phi(Action)
			if not self.legs[(i+2)%4].phi_is_legal(dp):
				#print "dPhi =",dp
				#print "Phi found is illegal"
				N += 1
				continue
			da,db,_ = Action[2]
			Action[2] = (da,db,dp)
				
			attempts = 5
			while attempts > 0:
				dp = choose_randomly(big_angles)
				
				foot3 = find_foot3(self, i, dp, Action)	
				if foot3 != None: break
				else: attempts -= 1

			else: # no solution found
				#print "This random action is illegal\n"
				N += 1
				continue

			# find alpha and beta
			a_b = find_alpha_beta(self,i,foot3)
			if a_b == None:
				N += 1
				continue
			else:
				alpha, beta = a_b
			
			leg = self.legs[(i+3)%4]
			da = alpha - leg.alpha
			db = beta - leg.beta
			if leg.alpha_is_legal(da) and leg.beta_is_legal(db):
				Action[3] = (da,db,dp)
				break
			else:
				#print "legal da or db cannot be found\nda,da:",da,db
				#print "leg3: a, b, phi:", leg.alpha, leg.beta, leg.phi
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
			#print "all legs touch the surface\n"
			raise
			return []
		elif dot > 0:
			#print "1st and 3rd legs can be raised\n"
			
			return [1, 3]
		else:
			#print "0th and 2nd legs can be raised\n"
			return [0, 2]
	

	def is_stable(self):
		"""returns tuple. First element is True or Flase if robot can stand on its tree legs respectively
		the second element is a projection of centre of mass onto the plane of three feet
		the third is a tuple of three q - a load factor on each leg
		"""
		feet = self.get_feet()
		f1,f2,f3 = tuple([ feet[i] for i in range(4) if i != self.raised_leg])
		
		v1 = f1-f2
		v2 = f3-f2
		ez1 = np.cross(v2,v1) 
		ez1 = ez1 / np.linalg.norm(ez1) 
		#print "sin:",ez1
		X0,Y0 = (ez1 * np.dot(ez1,f1))[:2]
		#print "X0,Y0",X0,Y0
		X1,Y1 = f1[0],f1[1]
		X2,Y2 = f2[0],f2[1]
		X3,Y3 = f3[0],f3[1]
		#print "Feet:", f1[:2],f2[:2],f3[:2]

		TX0 = (X0-X3)/(X1-X3)
		TX2 = (X2-X3)/(X1-X3)
		
		q2 = ( TX0 - (Y0-Y3)/(Y1-Y3) )/( TX2 - (Y2-Y3)/(Y1-Y3) )
		q1 =  TX0 - TX2 * q2
		q3 = 1 - (q1 + q2)

		xy = [ self.legs[i].xy for i in range(4) if self.raised_leg != i] 
		CoM = xy[0]*q1+xy[1]*q2+xy[2]*q3
		
		if q1>0 and q2>0 and q3>0: return (True, CoM, (q1,q2,q3))
		else: return (False,CoM,(q1,q2,q3))


	def update_orientation(self):
		#print "Raised leg:", self.raised_leg
		#print "CoM:",self.CoM
		if self.raised_leg != 0:
			f0 = self.get_foot(0,(0,0,0))
			niu = atan(f0[1]/f0[0])
			#print "niu:",niu,self.legs[0].xy
			self.orientation = atan((self.legs[0].xy[1]-self.CoM[1])/(self.legs[0].xy[0]-self.CoM[0])) - niu
		else:
			f2 = self.get_foot(2,(0,0,0))
			niu = atan(f2[1]/f2[0])
			#print "niu3:", niu,self.legs[2].xy
			self.orientation = atan((self.legs[2].xy[1]-self.CoM[1])/(self.legs[2].xy[0]-self.CoM[0])) - niu

		#print "orientation:",self.orientation		
		#if abs(self.orientation)>1: 
		#	raise


	def measure_output(self,mode):
		if mode == "forward":
			return self.CoM[:]
		elif mode == "right" or mode == "left":
			return float(self.orientation)
		else:
			return None




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

	


