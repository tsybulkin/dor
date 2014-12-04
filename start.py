#!/usr/bin/python
#
# dor-bug robot
#  
#
####################################################### 

from random import random,seed
from dor import Dor




def main():
	seed()
	D = Dor()
	#D.legs[0].alpha = 0.35
	#D.legs[0].phi = 0.2
	Action = D.get_random_action()
	print Action
	print D.take_action(Action)




if __name__ == '__main__':
	main()
	