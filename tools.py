#!/usr/bin/python
#
# dor-bug robot
#  
#
####################################################### 

import numpy as np
from random import random, seed

def choose_randomly(Ls):
	N = len(Ls)
	return Ls[int(N*random())]



TOLERANCE = 0.1
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


