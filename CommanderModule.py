
import time
import serial
from multiprocessing import Queue

def envia(ser, missatge,temps=0.1, show_time = False):
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
	
	
verboseCommands = False;
verboseSending = False;		
	
def printVerbose(str,verbose = False):
	if verbose:
		print "Commander: "+str
	
def func(input,output):
	print 'Commander process running...'
	
	global ser
	#Open the Serial Port.
	ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)
	
	print 'Test Mode On'
	envia(ser,'TestMode On', 0.2)
	print 'Set Motos Enable'
	envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)
	print 'Play Sound 1'
	envia(ser,'PlaySound 1', 0.3)

	
	#Parametros Robot.
	S = 121.5		# en mm
	distancia_L = 0	# en mm
	distancia_R = 0	# en mm
	speed = 0 		# en mm/s
	tita_dot = 0
	tiempo = 5		
	direccion = 0
	
	output.put(S)
	
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
				printVerbose(comando,verboseSending)
				envia(ser, comando, 0.2)
				comando = 'SetMotor RWheelEnable LWheelEnable'
				printVerbose(comando,verboseSending)
				envia(ser, comando, 0.2)
			else:
				distancia_R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * tiempo) * pow(-1, direccion)
				distancia_L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * tiempo) * pow(-1, direccion)
				
				comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
				printVerbose(comando,verboseSending)
				envia(ser,comando, 0.2)
			
		elif msg == "back": #recibimos acelerar hacia atras
			printVerbose(msg,verboseCommands)
			speed = speed-50
			if speed < 0:
				direccion = 1
				
			if speed == 0:
				comando = 'SetMotor LWheelDisable RWheelDisable'
				printVerbose(comando,verboseSending)
				envia(ser, comando, 0.2)
				comando = 'SetMotor RWheelEnable LWheelEnable'
				printVerbose(comando,verboseSending)
				envia(ser, comando, 0.2)
			else:
				distancia_R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * tiempo) * pow(-1, direccion)
				distancia_L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * tiempo) * pow(-1, direccion)
				
				comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
				printVerbose(comando,verboseSending)
				envia(ser,comando, 0.2)
				
		elif msg == "left": #recibimos girar a la izquierda
			printVerbose(msg,verboseCommands)
			tita_dot = tita_dot + (3.1415/10)
			
			distancia_R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * tiempo) * pow(-1, direccion)
			distancia_L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * tiempo) * pow(-1, direccion)
			
			comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
			printVerbose(comando,verboseSending)
			envia(ser,comando, 0.2)
			
		elif msg == "right": #recibimos girar a la derecha
			printVerbose(msg,verboseCommands)
			tita_dot = tita_dot - (3.1415/10)
			
			distancia_R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * tiempo) * pow(-1, direccion)
			distancia_L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * tiempo) * pow(-1, direccion)
			
			comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
			printVerbose(comando,verboseSending)
			envia(ser,comando, 0.2)
			
		elif msg == "stop": #recibimos parar
			printVerbose(msg,verboseCommands)
			
			direccion = 0
			speed = 0
			tita_dot = 0
			distancia_L = 0
			distancia_R = 0
			
			comando = 'SetMotor LWheelDisable RWheelDisable'
			printVerbose(comando,verboseSending)
			envia(ser, comando, 0.2)
			comando = 'SetMotor RWheelEnable LWheelEnable'
			printVerbose(comando,verboseSending)
			envia(ser, comando, 0.2)
			
		elif msg == "backbase": #recibimos orden de volver a la base
			printVerbose(msg,verboseCommands)
			
		elif msg == "laser": #recibimos leer laser
			printVerbose(msg,verboseCommands)
		
			comando = 'GetLDSScan'
			printVerbose(comando,verboseSending)
			ret = envia(ser, comando, 0.1)
			output.put('l' + ret)
			
		elif msg == "motors": #recibimos leer motores
			printVerbose(msg,verboseCommands)
			
			comando = 'GetMotors LeftWheel RightWheel'
			printVerbose(comando,verboseSending)
			ret = envia(ser, comando, 0.2)
			output.put('m' + ret)
		
	envia(ser, 'SetLDSRotation Off', 0.2)
	envia(ser, 'TestMode Off', 0.2)
	
	ser.close()