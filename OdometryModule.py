
import time
import math
import numpy
from multiprocessing import Queue
from Queue import Empty

verboseCommands = True;			
	
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
	
	S = float(input.get())
	anterior = [0,0,0]
	
	output.put('motors')
	msg = input.get()
	time.sleep(0.2);
	
	splited = msg.split('\n')
	L_ini = int(splited[4].split(',')[1])
	R_ini = int(splited[8].split(',')[1])
	anterior = [L_ini,R_ini,0,0,0]
	quite = False
	
	while not quite:
		
		try:
			msg = input.get_nowait()

			if msg == "q":
				printVerbose("quite",verboseCommands)
				quite = True
		
			elif msg[0] == "m": #Obtencion de datos del motor
				printVerbose("motors",verboseCommands)
				splited = msg.split('\n')
				
				L = int(splited[4].split(',')[1])
				R = int(splited[8].split(',')[1])
				anterior = odometry(anterior,L, R)
				print (anterior)
			elif msg[0] == "l": #Obtencion de datos del laser
				printVerbose("laser",verboseCommands)
				msg = msg[1:] #dentro de msg se encuentran los valores del laser
		
		except Empty:
			pass
				
		#output.put("motors")
		#time.sleep(0.2)
		#msg = input.get().split("\n")
		#Lf = int(msg[4].split(',')[1])
		#Rf = int(msg[8].split(',')[1])
		#print str(Lf)+","+str(Rf)