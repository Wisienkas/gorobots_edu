#!/usr/bin/env python
# -*- coding: utf-8 -*-
from matplotlib.pyplot import *
import matplotlib.pyplot as plt
from pylab import *
from numpy import *
import sys, os, time 
import matplotlib.patches as patches
import matplotlib as mpl


def main():
	 
	f, axs = subplots(1, sharex=True,figsize=(8,8))
	
	axs.grid(True)
	axs.set_xlim((-2,2)) 
	axs.set_ylim((-1.1,1.1)) 

	axs.set_xlabel('u1',fontsize=20)
	axs.set_ylabel('v1',fontsize=20)
	#axs.set_xlim(0,9000)
	
	ion() 
	
	

	#i=0
	#while i<1:
	t,u1, v1, w1, u2, v2 , w2, u3, v3, w3, u4, v4 ,w4,wsum,i1,i2 = loadtxt('outTezinhib.txt',unpack=True, usecols=(0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15))
		#plot(2*u1-1,2*v1-1,color="#CC5050")
	plot(i1-1,v1*2-1,color="red")
	plot(i2+1,v2*2-1,color="blue")
	draw()
	#i=i+1
		#show()
		#hold(False) #Next plot command will remove old data (during next loop)
		
		
	show()
	savefig("uv.eps",dpi=300,bbox_inches='tight')


if __name__ == '__main__':
	main()
