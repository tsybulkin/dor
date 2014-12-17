#!/usr/bin/python
#
# dor-bug robot
#  
#
####################################################### 

import sys
from random import random,seed
from dor import Dor
from agent import get_policy, get_state, get_reward, learn
from tools import read_data, write_data, clean_q
from math import degrees

import matplotlib.pyplot as plt

episode_len = 40

def run(nbr_episodes, mode):
	if mode == "forward": Q = read_data("data/forward.dat")
	elif mode == "right": Q = read_data("data/right.dat")
	elif mode == "left": Q = read_data("data/left.dat")
	else: 
		print "Unknown mode:", mode
		return None


	seed()
	
	for i in range(nbr_episodes):
		D = Dor()
		run_episodes(mode,episode_len,D,Q)
		print "Dor coordinates:",D.CoM, "orientation:%i%%" % degrees(D.orientation)

	for action in D.actions:
		print action

	#clean_q(Q)
	write_data(Q,"data/"+mode+".dat")
	


def run_episodes(mode, N, robot, Q):
	old_output = robot.measure_output(mode)
	State = get_state(robot)
	
	plt.axes()
	rectangle = plt.Rectangle((-25, -25), 60, 60, fc='w')
	plt.gca().add_patch(rectangle)
	plt.axis('scaled')
	plt.ion()
	plt.show()
			
	for i in range(N):
		Action = get_policy(State,Q)

		if Action == None:
			Move = robot.get_random_action()
			Action = (Move[0],Move[1],Move[2][:2],Move[3][2])
		else:
			Move = robot.get_move(Action)
		
		robot_fell = not robot.take_action(Move)
		#robot.draw(plt)
		NewState = get_state(robot)

		new_output = robot.measure_output(mode)
		if robot_fell:
			reward = -10
		elif Move[1] == Move[2] == Move[3] == (0,0,0):
			reward = --0.5
			#print "Reward= -1"
		else:
			reward = get_reward(mode, old_output, new_output, Move)
		
		#print "Reward:", reward
		old_output = new_output

		learn(NewState, State, Action, reward, Q)
		State = NewState

		if robot_fell: break



		



if __name__ == '__main__':
	Args = sys.argv[:]
	if len(Args) != 3:
		print "Wrong syntax\nUSAGE: python run.py <nbr_training_episodes> <movement>"
		print "Available movements:\nforward\nright\nleft"
	else:
		run(int(Args[1]), Args[2])


	