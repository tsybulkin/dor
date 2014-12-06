#!/usr/bin/python
#
# dor-bug robot
#  
#
####################################################### 

from random import random,seed
from dor import Dor

from time import clock


def main():
	seed()
	D = Dor()
	
	T1 = clock()
	Moves_nbr = 50
	for k in range(Moves_nbr):
		print "\n\tMove: %i\n" %k

		Action = D.get_random_action()
		print Action
		print D.take_action(Action)
		print "CoM:",D.CoM

	print "%i actions took %.1gsec\n" % (Moves_nbr,clock()-T1) 




if __name__ == '__main__':
	main()
	