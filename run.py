#!/usr/bin/python
#
# dor-bug robot
#  
#
####################################################### 

import sys
from random import random,seed
from dor import Dor
from agent import get_policy, get_state
from tools import read_data, write_data



def run(moves_nbr, mode):
	if mode == "forward": Q = read_data("data/forward.dat")
	elif mode == "right": Q = read_data("data/right.dat")
	elif mode == "left": Q = read_data("data/left.dat")
	else: 
		print "Unknown mode:", mode
		return None


	seed()
	D = Dor()

	run_episodes(moves_nbr,D,Q)
	write_data(Q,"data/"+mode+".dat")
	


def run_episodes(N, robot, Q):
	for i in range(N):
		Move = get_policy(get_state(robot),Q)
		if Move == None:
			Move = robot.get_random_action()
		
		robot.take_action(Move)
		



if __name__ == '__main__':
	Args = sys.argv[:]
	if len(Args) != 3:
		print "Wrong syntax\nUSAGE: python run.py <number of moves> <movement>"
		print "Available movements:\nforward\nright\nleft"
	else:
		run(int(Args[1]), Args[2])


	