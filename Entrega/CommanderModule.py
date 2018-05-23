
debug = False

import time
import math
if not debug:
	import serial
else:
	from dataMotors import getMotors

from multiprocessing import Queue
import numpy

verboseCommands = True;
verboseSending = True;	
verboseDebug = True;	

def printVerbose(str,verbose = False):
	if verbose:
		print "\rCommander - "+str
	
def envia(ser, missatge,temps=0.1, show_time = False):
	printVerbose("Sending: "+missatge,verboseSending)
	if not debug:
		first_time=time.time()
		rbuffer = ''
		resp = ''
		ser.write(missatge + chr(10)) 
		time.sleep(temps) # giving a breath to pi
		
		while ser.inWaiting() > 0:
			resp = ser.readline()
			rbuffer = rbuffer + resp

		if show_time:
			t = time.time() - first_time
			print("Round time: ",t)

		return rbuffer  
	
def get_laser():
	msg = envia(ser, 'GetLDSScan',0.2,False)
	var = []
	for line in msg.split('\r\n')[2:362]:
		s = line.split(',')
		var.append([s[0], s[1], s[2], s[3]])
	return var

def search_max():
	msg = get_laser()
	values = [5000 for num in range(360)]
	values2= []
	for value in msg: #Aqui recorrem els 360 resultats
		if int(value[1]) > 0 and (int(value[0]) > 300 or int(value[0]) < 60):
			values2.append([int(value[0]),int(value[1])])
	print(values2)
	"""
	for value in msg:
			values2[int(value[0])] = int(value[3])
	print(values2)
	"""
	factor_varianza=100
	res=[]
	i=1
	anterior=values2[0][1]
	seguidos=0
	while i<(len(values2)):
		if values2[i]<5000:
			if abs(values2[i][1]-anterior)>=factor_varianza:
				aux = (values2[i][0],values2[i][1])
				res.append(aux)
			anterior=values2[i][1]
		i=i+1

	return res
		

def comprovaLaser():
	envia(ser, 'SetLDSRotation On',0.2,False)
	time.sleep(3)
	print 'INI: SetLaser -> on'
	maxims = search_max()
	flag = False
	if len(maxims)>0: flag = True
	print('----------')
	print(maxims)
	print('----------')

	envia(ser, 'SetLDSRotation Off',0.2,False)
	time.sleep(3)
	print 'FIN: SetLaser -> off'
	return flag

def func(input,output):
	print 'Commander process running...'
	
	global ser
	ser = 0
	#Open the Serial Port.
	if not debug:
		ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)
	
	envia(ser,'TestMode On', 0.2)
	envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)
	envia(ser,'PlaySound 1', 0.3)
	time.sleep(0.2)
	envia(ser,'SetLDSRotation On', 0.2)
	
	#Parametros Robot.
	S = 121.5		# en mm
	distancia_L = 0	# en mm
	distancia_R = 0	# en mm
	speed = 0 		# en mm/s
	tita_dot = 0
	tiempo = 5		
	direccion = 0
	
	output.put(S)
	
	if debug:
		numLec = 0
	
	quite = False
	while not quite:
	
		msg = input.get()
		
		
		if msg == "q": #recibinos orden de salir
			printVerbose("quite",verboseCommands)
			quite = True
		
		elif msg == "forward": #recibimos acelerar hacia delante
			printVerbose(msg,verboseCommands)
			speed = speed+50
			if speed >= 0:
				direccion = 0
			
			if speed == 0:
				comando = 'SetMotor LWheelDisable RWheelDisable'
				envia(ser, comando, 0.2)
				comando = 'SetMotor RWheelEnable LWheelEnable'
				envia(ser, comando, 0.2)
			else:
				distancia_R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * tiempo) * pow(-1, direccion)
				distancia_L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * tiempo) * pow(-1, direccion)
				
				comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
				envia(ser,comando, 0.2)
			
		elif msg == "back": #recibimos acelerar hacia atras
			printVerbose(msg,verboseCommands)
			speed = speed-50
			if speed < 0:
				direccion = 1
				
			if speed == 0:
				comando = 'SetMotor LWheelDisable RWheelDisable'
				envia(ser, comando, 0.2)
				comando = 'SetMotor RWheelEnable LWheelEnable'
				envia(ser, comando, 0.2)
			else:
				distancia_R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * tiempo) * pow(-1, direccion)
				distancia_L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * tiempo) * pow(-1, direccion)
				
				comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
				envia(ser,comando, 0.2)
				
		elif msg == "left": #recibimos girar a la izquierda
			printVerbose(msg,verboseCommands)
			tita_dot = tita_dot + (3.1415/10)
			
			distancia_R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * tiempo) * pow(-1, direccion)
			distancia_L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * tiempo) * pow(-1, direccion)
			
			comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
			envia(ser,comando, 0.2)
			
		elif msg == "right": #recibimos girar a la derecha
			printVerbose(msg,verboseCommands)
			tita_dot = tita_dot - (3.1415/10)
			
			distancia_R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * tiempo) * pow(-1, direccion)
			distancia_L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * tiempo) * pow(-1, direccion)
			
			comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
			envia(ser,comando, 0.2)
			
		elif msg == "stop": #recibimos parar
			printVerbose(msg,verboseCommands)
			
			direccion = 0
			speed = 0
			tita_dot = 0
			distancia_L = 0
			distancia_R = 0
			
			comando = 'SetMotor LWheelDisable RWheelDisable'
			envia(ser, comando, 0.2)
			comando = 'SetMotor RWheelEnable LWheelEnable'
			envia(ser, comando, 0.2)
			
		elif msg == "backbase": #recibimos orden de volver a la base
			printVerbose(msg,verboseCommands)
			
			distance_weels_mid = 121.5
			distance_robot = 318
			speed = 125
	
			#per parar el moviment cap a la direccio, hem de tenir en compte els 2 maxims i que la distancia fins el objectiu sigui menor a 800!!!
			base = [0, 0]
			
			#funcionara mentres no detectem 2 maxims
			output.put("o2")
			time.sleep(0.1)

			msg = input.get().split(",")

			pos = "Odom:"+msg[0]+" "+msg[1]+" "+msg[2]
			printVerbose(pos,verboseDebug)
			
			odom = [float(msg[0]),float(msg[1]),float(msg[2])]
			x_go = base[0] - odom[0]
			y_go = base[1] - odom[1]

			angleRest = numpy.arctan2(y_go,x_go)
			if odom[2] - angleRest > 2*math.pi - (odom[2] - angleRest):
				angle = - (2 * math.pi - (odom[2] - angleRest))
			else: 
				angle = odom[2] - angleRest

			envia(ser, 'SetMotor LWheelDist '+ str(angle*distance_weels_mid) +' RWheelDist ' + str(-angle*distance_weels_mid) + ' Speed '+str(speed))
			time.sleep(abs((angle*distance_weels_mid)/speed))
			
			Dist = numpy.sqrt(x_go*x_go + y_go*y_go)
			divs = 10
			totalTime = Dist/speed
			timeRuning = totalTime/divs
			
			i = 0
			distRest = 0
			flagLaser = False
			while i < divs and not flagLaser: #!!! Falta condicio de detectar maxims i minims si la distancia restant es menor a 1000 !!!
				envia(ser, 'SetMotor LWheelDist '+ str(timeRuning*speed) +' RwheelDist ' + str(timeRuning*speed) + ' Speed '+str(speed))
				time.sleep(timeRuning)
				i += 1
				distRest = Dist - (i*timeRuning*speed)
				if (distRest < 1000):
					flagLaser = comprovaLaser()
			if (flagLaser == True): print "HE DETECTAT MAXIMS"
				
			#envia(ser, 'SetMotor LWheelDist '+ str(-angle*distance_weels_mid) +' RWheelDist ' + str(angle*distance_weels_mid) + ' Speed '+str(speed))
			#time.sleep(abs((angle*distance_weels_mid)/speed))
			 
			 
			#calcularem punt mitg maxims
			
			#ens digirem a aquest punt
			
			
			
		elif msg == "laser": #recibimos leer laser
			printVerbose(msg,verboseCommands)
			comando = 'GetLDSScan'
			ret = envia(ser, comando, 0.1)
			output.put('l' + ret)
			
		elif msg == "motors": #recibimos leer motores
			printVerbose(msg,verboseCommands)
			comando = 'GetMotors LeftWheel RightWheel'
			ret = envia(ser, comando, 0.2)
			if debug:
				(L, R) = getMotors(numLec)
				mult = 1
				L_int = float(L*mult)
				R_int = float(R*mult)

				printVerbose("L:"+str(L_int)+" R:"+str(R_int),verboseDebug)
				numLec = numLec + 1
				ret = "Parameter,Value\nLeftWheel_MaxPWM,65536\nLeftWheel_PWM,-858993460\nLeftWheel_mVolts,1310\nLeftWheel_Encoder,"+str(L_int)+"\nRightWheel_MaxPWM,65536\nRightWheel_PWM,-858993460\nRightWheel_mVolts,1310\nRightWheel_Encoder,"+str(R_int)

			output.put('m' + ret)
		
	envia(ser, 'SetLDSRotation Off', 0.2)
	envia(ser, 'TestMode Off', 0.2)
	
	ser.close()