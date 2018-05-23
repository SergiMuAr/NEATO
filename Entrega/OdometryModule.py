
debug = False

import time
import math
import numpy
from multiprocessing import Queue
from Queue import Empty
import http_viewer
from sets import Set

verboseCommands = True;			
	
###################### !!! W A R N I N G !!! ########################
# Each group working in the same robot has to chose a different port.
port_web_server = 11239
#####################################################################	
	
def printVerbose(str,verbose = False):
	if verbose:
		print "\rOdometry: "+str
		
def get_laser(msg):
	var = []
	for line in msg.split('\r\n')[2:362]:
		s = line.split(',')
		var.append([s[0], s[1], s[2], s[3]])
	return var

def polar_to_cart(laser):
	cart = []
	for i in range (0,359): #Polar 2 Cartesian in Robot Reference Frame
		if int(laser[i][3]) == 0:
			angulo = (0 + i)*numpy.pi/180
			x_r = int(laser[i][1])*numpy.cos(angulo) 
			y_r = int(laser[i][1])*numpy.sin(angulo)
			cart.append([x_r, y_r, 1])

	return cart

def cart_to_world(coord,odo):
	x = odo[2]
	y = odo[3]
	theta = odo[4]
	TAB = [[numpy.cos(theta),-numpy.sin(theta), x], [numpy.sin(theta), numpy.cos(theta), y], [0, 0, 1]]
	world = numpy.dot(TAB,numpy.transpose(coord))
	return numpy.transpose(world)
		
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
	laser_set = Set()
	odometry_queue = Queue()
	laser_queue = Queue()
	viewer = http_viewer.HttpViewer(port_web_server, laser_queue, odometry_queue)
	
	S = float(input.get())
	#print 'S=' + str(S)
	
	anterior = [0,0,0]

	output.put('m')
	msg = input.get().split("\n")
	L_ini = float(msg[4].split(',')[1])
	R_ini = float(msg[8].split(',')[1])
	anterior = [L_ini,R_ini,0,0,0]

	laserMotor = True
	motReq = False
	lasReq = False
	numOdometrys = 0
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
				print "\rOdometry: x:"+str(anterior[2])+" y:"+str(anterior[3])+" theta:"+str(anterior[4]*180/math.pi)
				motReq = False
				if(numOdometrys > 5):
					laserMotor = False
				numOdometrys += 1
				
			elif msg[0] == "l": #Obtencion de datos del laser
				printVerbose("laser",verboseCommands)
				msg = msg[1:] #dentro de msg se encuentran los valores del laser
				
				laser_points = get_laser(msg)
				
				var = polar_to_cart(laser_points)
				
				if len(var) > 0:
					var = cart_to_world(var,anterior)

					for i in var:
						tuple = (int(i[0]), int(i[1]))
						if tuple not in laser_set:
							laser_queue.put([(i[0], i[1]), (100,100)])
							laser_set.add(tuple)
				lasReq = False
				laserMotor = True
				numOdometrys = 0
							
			elif not motReq: 
				#odometry(anterior,L,R)
				output.put("motors")
				motReq = True		
				time.sleep(0.1)

		except Empty:
			if not motReq and not lasReq: 
				if laserMotor:
					#odometry(anterior,L,R)
					print "\nEnviem Calc Odometry"
					output.put("motors")
					motReq = True		
					time.sleep(0.1)
				else: 
					print "\nEnviem Calc Laser"
					output.put("laser")
					lasReq = True		
					time.sleep(0.1)
		

		
			
		