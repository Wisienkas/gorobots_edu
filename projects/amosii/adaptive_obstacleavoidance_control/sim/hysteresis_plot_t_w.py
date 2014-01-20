#!/usr/bin/env python
# -*- coding: utf-8 -*-
#2013 Eduard Grinke, eduard.grinke@gmail.com

#some libraries for plotting and data handling
from matplotlib.pyplot import *
import matplotlib.pyplot as plt
from pylab import *
from numpy import *
import sys, os, time 
import matplotlib.patches as patches
import matplotlib as mpl


def main():
	#defines one plot environment
	f, axs = subplots(1, sharex=True,figsize=(8,9)) 
	
	#sets a grid here	
	axs.grid(True)

	axs.set_ylim((-7,7)) 
	
	#begin and end timesteps for plotting my synaptic weights
	begin=0
	end=99000
	axs.set_xlim(begin,end)
	
	#x y labels
	axs.set_xlabel('timesteps',fontsize=20)
	axs.set_ylabel('magnitude',fontsize=20)
	

	#sets the mode my adaptive mrc is working!
	mode=3
	#t,u1,v1,w1,w1n,u2,v2,w2,w2n,u3,v3,w3,w3n,u4,v4,w4,w4n,i1,i2,i1r,i2r = loadtxt('outTezinhib.txt',unpack=True, usecols=(0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20))
	t,u1,v1,w1,w1n,u2,v2,w2,w2n,u3,v3,w3,w3n,u4,v4,w4,w4n,i1,i2,i1r,i2r = loadtxt('outTezinhib.txt',unpack=True,comments='#' )

#here are different plots for the different modes in which my obstacle avoidance can run in. mode 3 is best!
	if int(mode) ==7:
		axhline(y=0, xmin=0, xmax=1 ,color="green",label="c1")
		axhline(y=0, xmin=0, xmax=1 ,color="purple",label="c1")
		axhline(y=-0, xmin=0, xmax=1 ,color="red",label="b1")
		axhline(y=-0, xmin=0, xmax=1 ,color="blue",label="b2")
		
	if int(mode) ==6:
		axhline(y=-3.5, xmin=0, xmax=1 ,color="green",label="c1")
		axhline(y=-3.52, xmin=0, xmax=1 ,color="purple",label="c1")
		axhline(y=2.4, xmin=0, xmax=1 ,color="red",label="b1")
		axhline(y=2.42, xmin=0, xmax=1 ,color="blue",label="b2")
	if int(mode) ==5:
		plot(t[begin:end],-w3[begin:end],color="green",label="c1")
		plot(t[begin:end],-w4[begin:end],color="purple",label="c2")
		axhline(y=2.4, xmin=0, xmax=1 ,color="red",label="b1")
		axhline(y=2.42, xmin=0, xmax=1 ,color="blue",label="b2")
		
	if int(mode) ==4:
		axhline(y=-3.5, xmin=0, xmax=1 ,color="green",label="c1")
		axhline(y=-3.52, xmin=0, xmax=1 ,color="purple",label="c1")
		plot(t[begin:end],w1[begin:end],color='red',label="b1")
		plot(t[begin:end],w2[begin:end],color="blue",label="b2")
		
	if int(mode) ==3:
		plot(t[begin:end],-w3[begin:end],color="green",label="c1")
		plot(t[begin:end],-w4[begin:end],color="purple",label="c2")
		plot(t[begin:end],(-w4[begin:end]-w3[begin:end])*0.5,color="white",label=" ")
		plot(t[begin:end],(-w4[begin:end]-w3[begin:end])*0.5,color="lime",label="c")
		plot(t[begin:end],w1[begin:end],color='red',label="b1")
		plot(t[begin:end],w2[begin:end],color="blue",label="b2")
		
	if int(mode) ==2:
		plot(t[begin:end],-w3[begin:end],color="green",label="c1")
		plot(t[begin:end],-w4[begin:end],color="purple",label="c2")
		plot(t[begin:end],w1[begin:end],color='red',label="b1")
		plot(t[begin:end],w2[begin:end],color="blue",label="b2")
		plot(t[begin:end],(w2[begin:end]+w1[begin:end])*0.5,color="white",label=" ")
		plot(t[begin:end],(w2[begin:end]+w1[begin:end])*0.5,color="pink",label="b")


	if int(mode) ==1:
		plot(t[begin:end],-w3[begin:end],color="green",label="c1")
		plot(t[begin:end],-w4[begin:end],color="purple",label="c2")
		plot(t[begin:end],(-w4[begin:end]-w3[begin:end])*0.5,color="lime",label="c")
		plot(t[begin:end],w1[begin:end],color='red',label="b1")
		plot(t[begin:end],w2[begin:end],color="blue",label="b2")
		plot(t[begin:end],(w2[begin:end]+w1[begin:end])*0.5,color="pink",label="b")
		
	if int(mode) ==0:
		plot(t[begin:end],-w3[begin:end],color="green",label="c1")
		plot(t[begin:end],-w4[begin:end],color="purple",label="c2")
		plot(t[begin:end],w1[begin:end],color='red',label="b1")
		plot(t[begin:end],w2[begin:end],color="blue",label="b2")
		
	plot(t[begin:end],v1[begin:end],color="black",label="output left")

	
	if int(mode)==3 or int(mode)==2 or int(mode)==1:
		axs.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,ncol=4, mode="expand", borderaxespad=0.)
	else:
		axs.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,ncol=3, mode="expand", borderaxespad=0.)

	savefig("synaptic_magnitudes.pdf",dpi=80)
	

if __name__ == '__main__':
	main()
