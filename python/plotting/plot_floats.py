##Generic Line Plotter Tool
import time
import struct
import numpy as np
from matplotlib import animation
from matplotlib import pyplot as plt


lines = []
xbuf = []
ybuf = []
tstart = 0
num_lines = 0
bufwidth = 0

#initialization function. needed for the 'blitting' option,
#which is the lowest latency plotting option
def init(): # required for blitting to give a clean slate.
	global lines
		
	for line in lines:
		line.set_data([],[])
	return lines


def animate(args):

	global ax, lines, xbuf, ybuf, num_lines, bufwidth

	for i in range(0,num_lines):
		del xbuf[i][0]
		del ybuf[i][0]
		xbuf[i].append(args[0])
		
	i = 0
	for arg in args:
		if(i > 0):
			ybuf[i-1].append(arg)
		i = i + 1
	for i, line in enumerate(lines):
		line.set_data(xbuf[i],ybuf[i])
	
	xmin = min(xbuf[0])
	xmax = max(xbuf[0])
	plt.setp(ax,xlim = (xmin,xmax))

	ax.relim()
	ax.autoscale_view(scalex=False, scaley=False)
	return lines

def plot_floats(n, width, data_gen, xtuple, ytuple, title="", xlabel="", ylabel=""):
	
	global fig, ax, lines, xbuf, ybuf, num_lines, bufwidth, tstart

	fig,ax = plt.subplots()
	plt.setp(ax,ylim = xtuple)	#manually set axis y limits
	plt.setp(ax,xlim = ytuple)

	plt.title(title)
	plt.xlabel(xlabel) 
	plt.ylabel(ylabel)
	
	num_lines = n
	bufwidth = width
	
	lines = []
	xbuf = []
	ybuf = []

	#setup xy buffers to plot and axes
	for i in range(num_lines):
		lines.append(ax.plot([],[])[0])
		xbuf.append([])
		ybuf.append([])
	#initalize all xy buffers to 0
	for i in range(0, num_lines):	
		for j in range(0,bufwidth):
			xbuf[i].append(0)
			ybuf[i].append(0)
			
	tstart = time.time()
		

	anim = animation.FuncAnimation(fig, animate, init_func=init, frames=data_gen, interval=0, blit=True,  save_count = 50)
	plt.show()
	
	
	
