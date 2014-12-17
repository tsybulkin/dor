#!/usr/bin/python
#
# dor-bug robot
#  
#
####################################################### 
import os

import numpy as np
import pickle
from random import random, seed
from math import sin, cos, asin, atan, sqrt


def choose_randomly(Ls):
	N = len(Ls)
	return Ls[int(N*random())]



TOLERANCE = 0.5
MAX_ITER = 10

def secant(A,B,Fa,Fb,fun,Iter=1):
	#print "secant:",Iter
	if Iter > MAX_ITER:
		return None

	C = (A*Fb-B*Fa)/(Fb-Fa)
	#print "secant: C =", C
	Fc = fun(C)
	if Fc > TOLERANCE:
		return secant(A,C,Fa,Fc,fun,Iter+1)
	elif Fc < -TOLERANCE:
		return secant(C,B,Fc,Fb,fun,Iter+1)
	else:
		return C


def get_random_action(leg, angles, attempts=7):
	da = choose_randomly(angles)
	t1 = attempts
	while not leg.alpha_is_legal(da) and t1 > 0:
		da = choose_randomly(angles)
		t1 -= 1 	
	if t1<0: return None

	t2 = attempts
	db = choose_randomly(angles)
	while not leg.beta_is_legal(db) and t2 > 0:
		db = choose_randomly(angles)
		t2 -= 1	
	if t2<0: return None
				
	t3 = attempts
	dp = choose_randomly(angles)
	while not leg.phi_is_legal(dp) and t3 > 0:
		dp = choose_randomly(angles)
		t3 -= 1	
	if t3<0: return None
	#print "random action selected"
	
	return (da,db,dp)


def d(angle):
	"""discretize the angle
	"""
	discr = 0.2
	return int(round(angle/discr))


def coin(p): return random()<p


def read_data(my_file):
	if not os.path.isfile(my_file):
		return {}

	with open(my_file, 'rb') as handle:
  		data = pickle.loads(handle.read())
  	handle.close()

  	return data


def write_data(data, my_file):
	with open(my_file, 'wb') as handle:
		pickle.dump(data, handle)
	handle.close()



def find_foot3(robot, i, dp, Action):
	""" returns a foot of the leg for a given dp
	i is an index of the raised leg
	if there is no legal solution returns None
	"""
	d1 = robot.feet_distances[((i+3)%4, (i+2)%4)]
	d2 = robot.feet_distances[((i+3)%4, (i+1)%4)]
	d3 = robot.feet_distances[((i+2)%4, (i+1)%4)]
	#print "d1,d2,d3:",d1,d2,d3
	q1 = (d1**2 - d2**2 + d3**2) / d3 / 2
	
	# the center of the circle which the foot of the last leg lays on
	foot1 = robot.get_foot((i+1)%4, Action[1])
	foot2 = robot.get_foot((i+2)%4, Action[2])
	
	r0 = (q1 * foot1 + (d3-q1) * foot2 ) / d3
	#d0_square = d1**2 - q1**2
	v0 = foot2 - foot1
	#print "r0, v0:", r0, v0 

	leg = robot.legs[(i+3)%4]
			
	# leg's plane
	v3 = np.array([-sin(leg.aa+leg.phi+dp), 
					cos(leg.aa+leg.phi+dp),
					0])
	r3 = np.array([robot.R * cos(leg.aa),
					robot.R * sin(leg.aa),
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
			else: return None
		else:
			t3 = secant(A,B,Fa,Fb,fun)
	else:
		t3 = secant(A,B,Fa,Fb,fun)

	if t3 == None: return None
	else: return r_line + v_line*t3 	


def find_alpha_beta(robot, i, foot3):
	leg = robot.legs[(i+3)%4]
	joint3 = np.array([robot.R * cos(leg.aa),
						robot.R * sin(leg.aa),
						0])
	delta = atan(-foot3[2]/sqrt((joint3[0]-foot3[0])**2 + (joint3[1]-foot3[1])**2))
	#print "delta:", degrees(delta)
	foot_distance = np.linalg.norm(foot3-joint3)
	if foot_distance > 2*leg.L:
		return None
	else:
		beta = 2 * asin(foot_distance/leg.L/2)
		alpha = delta + beta/2
		return (alpha,beta)
	

def clean_q(Q):
	"""removes worst records in Q dictionary
	"""
	States = set([ S for (S,A) in Q])
	print "There is %i unique states" % len(States)

	for State in States:
		actions = [ (Q[(S,A)], A) for (S,A) in Q if S == State]
		if len(actions) < 20: continue
		
		actions.sort()
		new_records = actions[-10:]
		
		for (V,A) in actions:
			del Q[(State,A)] 


		for (V,A) in new_records:
			Q[(State,A)] = V

	



