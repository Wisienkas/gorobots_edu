#!/usr/bin/env python
# -*- coding: utf-8 -*-
#2013 Eduard Grinke eduard.grinke@gmail.com

#needed for using some matplot lib functions
from matplotlib.pyplot import *
from numpy import *
import sys, os 
import matplotlib.patches as patches
import matplotlib as mpl


def main():

	#here i define one plot environment, check matplot lib documentation for another examples. you can add more plot windows for example...
	f, axs = subplots(1, sharex=True,figsize=(8,8))
	
	#here you need to specify your tracking data file. tracking has to be activated in the main.cpp
	data = loadtxt('AmosII_track__2013-12-09_12-50-08.log')

	#create 2 vectors with the x y for storing the data
	x = data[:,1]
	y = data[:,2] 

	#here i shift the grid and drawing of the plot, so my stuff is in center, depends on the built environment, may be you dont need it
	x=x-3
	#anaother fine shift for the boxes
	j=-0.5
	i=-0.5
	
	#my environment can have turned walls so you can rotate each wall here
	wall_deg=[0,0,0,0]
	#position of the walls
	wall_pos=[(-5+i,-5+j),(5+i,-5+j),(-5-i,5+j),(-5-i,-5+j)]
	#all the length of the walls
	l=[11,11,1,1]
	#all the broadnesses of the walls
	b=[1,1,9,9]
	
	#adds  wall with optional rotation to the drawing!
	for i in range(4):
		r = patches.Rectangle(wall_pos[i], b[i], l[i], color="#505060", alpha=1)
		t= mpl.transforms.Affine2D().rotate_deg(wall_deg[i]) + axs.transData
		r.set_transform(t)
		axs.add_patch(r)

	#here you specify your environment, i use boxes here, they can have an angle, i and j is another shift. could be combined ...
	angle = 1
	j=-0.5
	i=-3.5
	rect_pos=[(0+i,0+j),(1+i,1+j),(2+i,2+j),(3+i,-2+j),(5+i,-1+j),(6+i,-2+j),(4+i,2+j),(5+i,-3+j),(-1+i,-4+j),(3+i,4+j),(7+i,0.5+j),(0+i,3+j),(0+i,-2+j),(-1.5+i,-2+j),(1.5+i,-4+j),(7+i,-4+j),(3.2+i,0+j),(1.5+i,-1+j),(5.2+i,3.8+j),(7+i,4+j),(5.8+i,2+j),(3.5+i,-4+j)]
	rect_deg=[angle*3,angle*2,angle*4,angle*2,angle*1,angle*1,angle*5,angle*3,angle*0,angle*0,angle*0,angle*3,angle*2,angle*4,angle*2,angle*3,angle*2,angle*4,angle*3,angle*2,angle*2,angle*1]

	for i in range(22):
		(a,b)=rect_pos[i]
		#circle = Circle((0, 0), radius=0.75, fc='y')
		r = patches.Rectangle(rect_pos[i], 1, 1, color="#000050", alpha=0.5)
		#r = patches.Circle((rect_pos[i]), radius=0.5, fc='y')
		t=mpl.transforms.Affine2D().rotate_around(a-j,b-j,rect_deg[i]) +  axs.transData 
		r.set_transform(t)
		axs.add_patch(r)


	#draw arrow at the beginning and then in the field setted here but only every 500 steps
	#i=0
	#drw_intervall=500
	#while i<9479:
		#average over two deltas of (x,y) points
		#arrow( x[i], y[i], (x[100+i]-x[i]+x[200+i]-x[i+100])*0.5 ,(y[100+i]-y[i]+y[200+i]-y[i+100])*0.5, fc="k", ec="k",head_width=0.2, head_length=0.3 )
		#i=i+drw_intervall
		

	#set grid and x y ranges and also ticks
	axs.grid(True)
	axs.set_xticks([i/2 for i in range(-8,9)])
	axs.set_yticks([i/2 for i in range(-8,9)])
	axs.set_xlim((-5,5)) 
	axs.set_ylim((-5,5)) 
	axs.plot(x,y,color="#CC5050")
	
	#show()
	savefig("x_y_angle.pdf",dpi=75,pad_inches=0,bbox_inches='tight')
	return 0

if __name__ == '__main__':
	main()
