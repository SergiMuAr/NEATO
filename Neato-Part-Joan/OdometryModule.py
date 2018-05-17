
debug = True

import time
import math
import numpy
from multiprocessing import Queue
from Queue import Empty
import http_viewer

verboseCommands = True;			
	
###################### !!! W A R N I N G !!! ########################
# Each group working in the same robot has to chose a different port.
port_web_server = 11235
#####################################################################	
	
def printVerbose(str,verbose = False):
	if verbose:
		print "Odometry: "+str
		
def odometry(ant,L, R):
	result = []
	
	L_ini = ant[0]
	R_ini = ant[1]
	 
	x = ant[2]
	y = ant[3]
	suma_theta = ant[4]

	new_L, new_R = L - L_ini, R - R_ini

	result.append(L)
	result.append(R)
	

	delta_d = (new_R+new_L)/2
	delta_th = (new_R-new_L)/243.
	suma_theta = (suma_theta+delta_th)
	suma_theta = numpy.mod(suma_theta,2*math.pi)
	dx = delta_d*math.cos(suma_theta)
	dy = delta_d*math.sin(suma_theta)
	x = x + dx
	y = y + dy
	result.append(x)
	result.append(y)
	result.append(suma_theta)

	#print("El que vui printar %f.2" % dada)
	
	return result
	
def func(input,output):
	print 'Odometry process running...'
	
	odometry_queue = Queue()
	laser_queue = Queue()
	viewer = http_viewer.HttpViewer(port_web_server, laser_queue, odometry_queue)
	
	S = float(input.get())
	print 'S=' + str(S)
	
	anterior = [0,0,0]

	output.put('motors')
	msg = input.get().split("\n")
	L_ini = float(msg[4].split(',')[1])
	R_ini = float(msg[8].split(',')[1])
	anterior = [L_ini,R_ini,0,0,0]
	
	quite = False
	
	while not quite:
		
		try:
			msg = input.get_nowait()

			if msg == "q":
				printVerbose("quite",verboseCommands)
				quite = True
				viewer.quit()
		
			elif msg == "o2":
				output.put(str(anterior[2])+","+str(anterior[3])+","+str(anterior[4]))
		
			elif msg[0] == "m": #Obtencion de datos del motor
				printVerbose("motors",verboseCommands)
				msg = msg.split("\n") #dentro de msg se encuentran los valores del motor
				L = float(msg[4].split(',')[1])
				R = float(msg[8].split(',')[1])
				anterior = odometry(anterior,L, R)
				odometry_queue.put([(anterior[2], anterior[3]), (100,100)])
				print "Odometry: x:"+str(anterior[2])+" y:"+str(anterior[3])+" theta:"+str(anterior[4]*180/math.pi)
			elif msg[0] == "l": #Obtencion de datos del laser
				printVerbose("laser",verboseCommands)
				msg = msg[1:] #dentro de msg se encuentran los valores del laser

		except Empty:
			pass
			
		