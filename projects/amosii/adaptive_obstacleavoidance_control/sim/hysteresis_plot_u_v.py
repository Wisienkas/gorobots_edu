#!/usr/bin/env python
# -*- coding: utf-8 -*-
#2013 Eduard Grinke, eduard.grinke@gmail.com

#needed libraries for my plot
from matplotlib.pyplot import *
import matplotlib.pyplot as plt
from pylab import *
from numpy import *
import sys, os, time 
import matplotlib.patches as patches
import matplotlib as mpl


def main():
	#ion is to animate the stuff in realtime
	ion()  
	#sets on plot environment
	f, axs = subplots(1, sharex=False,figsize=(8,8))

	
	time=1

	#i=0
	while 	time<2:
	#runsteps<u1 v1 weight_neuron1 ANN::getWeight(2,2) u2 v2  weight_neuron2 ANN::getWeight(3,3) u3 v3 weight_neuron3 ANN::getWeight(3,2) u4 v4  weight_neuron4 ANN::getWeight(2,3) i1 i2 i1_refl<<" "<<i2_refl<<endl;
	#old   t,u1, v1, w1, u2, v2 , w2, u3, v3, w3, u4, v4 ,w4,wsum,i1,i2 = loadtxt('outTezinhib.txt',unpack=True, usecols=(0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15))
		#loads the colums in the following vectors on the left
		t,v1,v2,i1,i2 = loadtxt('outTezinhib.txt',unpack=True, usecols=(0,1,6,17,18))


		#has to be set here, cause window refreshes each plot
		axs.grid(True)
		axs.set_xlim((-1.1,1.1)) 
		axs.set_ylim((-1.1,1.1)) 
		axs.set_xlabel('u1',fontsize=30)
		axs.set_ylabel('v1',fontsize=30)
		#actual plot
		draw()
		time=time+1
	hold(False)  #Next plot command will remove old data (during next loop)

	savefig("uv.pdf",dpi=150,bbox_inches='tight')


if __name__ == '__main__':
	main()
