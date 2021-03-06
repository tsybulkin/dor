#!/usr/bin/python
#
# dor-bug robot
#  
#
####################################################### 

from tools import d, coin
from math import pi

gama = 0.9
eps = 0.05
lr = 0.5 # learning rate

di = [-1,0,1]
dS = [ (s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12)
		for s1 in di for s2 in di for s3 in di 
		for s4 in di for s5 in di for s6 in di
		for s7 in di for s8 in di for s9 in di
		for s10 in di for s11 in di for s12 in di ]



#def run_episodes(State,N):

def get_policy(State,Q):
	return get_eps_policy(State,Q)


def get_eps_policy(State,Q):
	if coin(eps):
		return None
	else:
		return get_greedy_policy(State,Q)


def get_explore_policy(State,Q):
	value_action_list = [ (Q[(S,A)], A) for (S,A) in Q if S == State]
	value_action_list.sort()
	
	if len(value_action_list) != 0:
		Value, Action = value_action_list.pop()
		if Value < 0 or len(value_action_list)<3:
			return None
		else:
			return Action
	else:
		return None


def get_greedy_policy(State, Q):
	"""returns the best action for the State
	"""
	value_action_list = [ (Q[(S,A)], A) for (S,A) in Q if S == State]
	
	if len(value_action_list) != 0:
		value_action_list.sort()
		Value, Action = value_action_list.pop()
		print "Value:%.2g, actions: %i" %(Value,len(value_action_list)+1)
		#print value_action_list
		return Action
	else:
		return None



def get_reward(Mode, Output_old, Output_new, Action):
	"""returns the next state and reward
	"""
	if Mode == "forward":
		dV = Output_new - Output_old
		return dV[0] + dV[1] - abs(dV[0]-dV[1])
	
	elif Mode == 'right':
		dV = Output_old - Output_new
		if dV > 1.5 * pi: dV = -2*pi + dV
		elif dV < -1.5*pi: dV = 2*pi + dV
		return dV

	elif Mode == "left":
		dV = Output_new - Output_old
		if dV > 1.5 * pi: dV = -2*pi + dV
		elif dV < -1.5*pi: dV = 2*pi + dV
		return dV
	else:
		return None


def learn(NewState, State, Action, Reward, Q):
	V1 = get_value(NewState,Q)
	V = get_value(State,Q)
	#Action = (Move[0], Move[1], Move[2][:2],Move[3][2])
	#print "Action:", Action

	V_new = V + lr * (Reward + gama * V1 - V)
	if V_new > V/3:
		Q[(State,Action)] = V_new


def get_close_values(State,Q):
	values = [ Q(S+dS,A) for (S,A) in Q for dS in variations ]



def get_value(State,Q):
	values = [ Q[(S,A)] for (S,A) in Q if S == State]
	if len(values) == 0:

		return get_close_values(State,Q)
	else:
		return max(values)



def get_state(dor):
	"""returns a discrete state for particular dor position defined by 12 angles
	"""
	State = (dor.raised_leg,)
	for leg in dor.legs:
		State +=  ( d(leg.alpha),d(leg.beta),d(leg.phi) ) 

	return State


