#! /usr/bin/python

# Tool for visualizing quaternion as rotated cube
import math
import linecache
import sys
import sys
import math
from ivy.std_api import *
import logging
import getopt

import pygame
import time
import platform
import os

_NAME = 'attitude_viz'

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors as matcol


LAST_DATA=range(10,164)
LAST_LINE=range(10,100)
MATRIX_HEIGHT=9
LINE_RECEIVED=0
LAST_PITCH=0
LAST_ROLL=0
AVERAGE_DATA = False
ONLY_DRAW_LAST=True
def draw_sonar_visualisation(matrix, single_line, last_pitch,last_roll):
    try:
	    plt.ion()
	    r = matrix[1]
	    r = (map(abs, map(int, r)))
	    theta = np.arange(0,2*np.pi,(2*np.pi)/len(r))
	    ax = plt.subplot(111, polar=True)
	    ax.clear()
	    colors=[]
	
	    for element in matcol.cnames:
		#print 'color: ' , element
		colors.append(element)
#	    colors = ['r', 'g', 'b', 'y', 'k','r', 'g', 'b']
	    if ONLY_DRAW_LAST:
		r = single_line
		r = (map(abs, map(int, r)))
		theta = np.arange(0,2*np.pi,(2*np.pi)/len(r))
		r = r[::-1]
		new_r=[]
		rotateTimes=24
		new_r.extend(r[rotateTimes::])
		new_r.extend(r[:rotateTimes])
		r = new_r
		print 'len r: ', len(r) , ' and len theta: ', len(theta)
		print 'r when plotting : ', r , ' and theta: ', theta
		ax.plot(theta, r, color=colors[0],linewidth=5)
	    elif AVERAGE_DATA:
		toPlotSum = np.array(matrix[0])
		for i in range(1, MATRIX_HEIGHT):
			toPlotSum += np.array(matrix[i])
		r = toPlotSum/MATRIX_HEIGHT	
		r = (map(abs, map(int, r)))
		theta = np.append(theta,theta[0])
		r = np.append(r,r[0])
		#print 'len R ', len(r), ' len theta: ', len(theta)
		ax.plot(theta, r, color=colors[0], linewidth=5)
	    else:
		for i in range(0, MATRIX_HEIGHT):
			r = matrix[i]
			r = (map(abs, map(int, r)))
			theta = np.arange(0,2*np.pi,(2*np.pi)/len(r))
			print 'R is now ', r
			print 'R is len ', len(r) , ' theta is len: ', len(theta)
			print 'now at i: ', i, ' colors len: ', len(colors)
			ax.plot(theta, r, color=colors[i%len(colors)], linewidth=3)	
	    if last_pitch!=0 or last_roll !=0:
		    print 'last pitch: ', last_pitch, ' last roll: ', last_roll
		    last_angle=math.atan2(-1*last_pitch,last_roll)
		   ## last_angle+=(2*math.pi)/3 
		    theta=[last_angle,last_angle+0.01]
		    r=[100,0]
		    print 'plotting angle: ' , theta, ' , R: ' , r
		    ax.plot(theta, r, color='k', linewidth=3)	
	    ax.set_rmax(15.0)
	    ax.grid(True)
	    plt.draw()
    except Exception as eee:
        print 'Stopping program! ' , eee
        PrintException()
 





def PrintException():
    exc_type, exc_obj, tb = sys.exc_info()
    f = tb.tb_frame
    lineno = tb.tb_lineno
    filename = f.f_code.co_filename
    linecache.checkcache(filename)
    line = linecache.getline(filename, lineno, f.f_globals)
    print 'EXCEPTION IN ({}, LINE {} "{}"): {}'.format(filename, lineno, line.strip(), exc_obj)


class Visualization:
    def __init__(self, parent):
         print 'Initialisation visualization'
    def onmsgproc(self, agent, *larg):
        global LAST_DATA,LINE_RECEIVED,LAST_PITCH,LAST_ROLL
        data = str(larg[0]).split(' ')
	#print 'data received: ',data
	if data[1]=='R_DOT_AND_SPEED':
		print 'received data: ', data
		LAST_PITCH=float(data[4])
		LAST_ROLL=float(data[5])
	elif data[1]=='DISTANCE_MATRIX':	
		LINE_RECEIVED=int(data[2])
		print 'REICEIVED DISTANCE MATRIX: ', LINE_RECEIVED
		LAST_DATA = data[3::][0].split(',')
	
		for i in range(0,len(LAST_DATA)):
			LAST_DATA[i]=int(LAST_DATA[i])
	
	
     
class Visualizer:
    def __init__(self):
        self.visualization = Visualization(self)
        # listen to Ivy
        logging.getLogger('Ivy').setLevel(logging.WARN)
        IvyInit(_NAME,"",0,lambda x, y: y,lambda x, z: z)

        if os.getenv('IVY_BUS') is not None:
            IvyStart(os.getenv('IVY_BUS'))
        else:
            if platform.system() == 'Darwin':
                IvyStart("224.255.255.255:2010")
            else:
                IvyStart()

        # list of all message names
        messages = []
        messages.append("R_DOT_AND_SPEED")
        messages.append("DISTANCE_MATRIX")

        # bind to set of messages (ie, only bind each message once)
        for message_name in set(messages):
            bind_string = "(^.*" + message_name + ".*$)"
            print 'Binding on string: ', bind_string
            IvyBindMsg(self.visualization.onmsgproc, bind_string)

    def OnClose(self):
        IvyStop()



def run():
    
    global LAST_DATA
    window_title = "Sonar_Viz"

   

    visualizer = Visualizer()
    # INITIALISE EVERYTHING HERE BECAUSE PLOTTING CAN ONLY HAPPEN ON THE MAIN THREAD!
    X = range(10,15)
    Y = range(10,15)
    plt.ion()
    graph = plt.plot(X,Y)[0]
    plt.draw()
    matrix = []
    for h in range(0,MATRIX_HEIGHT):
      matrix.append([0]*36)
    try:
        while True:
            time.sleep(.02)
	    horizontalPixelsAmount=36
	    verticalPixelsAmount=6
            matrix[LINE_RECEIVED]=LAST_DATA
	    
	    draw_sonar_visualisation(matrix,LAST_DATA,LAST_PITCH,LAST_ROLL)

              

    except Exception as eee:
        print 'Stopping program! ' , eee
        PrintException()
        visualizer.OnClose()
        return


if __name__ == "__main__":
    run()

