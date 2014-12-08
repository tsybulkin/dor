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


def get_random_action(leg, angles, attempts=10):
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
	print "random action selected"
	
	return (da,db,dp)


def d(angle):
	"""discretize the angle
	"""
	discr = 0.1
	return round(angle/discr)


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










