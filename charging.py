from test_NeatoCommands import envia

import serial
import time
import numpy as np
import math 

import os, signal
import sys, tty, termios
from select import select


def search_max():
	msg = get_laser()
	values = [5000 for num in range(360)]
	values2= [5000 for num in range(360)]
	for value in msg:
		if int(value[1]) > 0:
			values[int(value[0])] = int(value[1])
	print(values)
	for value in msg:
			values[int(value[0])] = int(value[3])
	print(values)
	factor_varianza=300
	varianza=[]
	i=1
	anterior=values[0]
	seguidos=0
	while i<(len(values)):
		if values[i]<5000:
			if abs(values[i]-anterior)>=factor_varianza:
				aux = (i,values[i])
				varianza.append(aux)
			anterior=values[i]
		i=i+1
	return varianza
	
	
		

def handler(signum, frame):
	print "Signal detected"
	# Finalizar -------------------------------
	print '----------------------------'
	envia(ser, 'PlaySound 2')
	envia(ser, 'TestMode Off',0.2)
	print 'FIN: TestMode -> off'
	envia(ser, 'SetLDSRotation Off',0.2,False)
	time.sleep(3)
	print 'FIN: SetLaser -> off'
	# Close the Serial Port.
	ser.close()  
	print 'FIN: SerialPort -> off'    
	print "FIN: Programa finalizado"
	exit(1)



# -------------------------------------------------------------------------------
# VARIABLES
# -------------------------------------------------------------------------------
#variables Neato
S = 121.5
wheel_rad = 38.5
dist_max = 4000
velocity_max = 300
calibration = 0.5
distDetection = 1000

# -------------------------------------------------------------------------------
# MOTOR FUNC
# -------------------------------------------------------------------------------
def get_motors():
    msg = envia(ser, 'GetMotors LeftWheel RightWheel').split('\n')    
    L = int(msg[4].split(',')[1])
    R = int(msg[8].split(',')[1])
    
    return (L, R)

# -------------------------------------------------------------------------------
# LASER FUNC
# -------------------------------------------------------------------------------
def get_laser():
	msg = envia(ser, 'GetLDSScan',0.2,False)
	var = []
	for line in msg.split('\r\n')[2:362]:
		s = line.split(',')
		var.append([s[0], s[1], s[2], s[3]])
	return var

def get_5sensor (laser):
	sensor = []
	for i in range (0,5):
		sensor.append(dist_max)
		for j in range (-60 + i*24, -60 + 24 + i*24):
			if int(laser[j][3]) == 0:
				#print laser[j][1],' < ',sensor[i]
				if (int(laser[j][1]) < sensor[i]):
					sensor[i] = int(laser[j][1])

	return sensor
	
def getLaserValues():
	global speed
	global difDestino
	res = [2000,2000,2000,2000,2000,2000,2000,2000,2000,2000]
	msg = get_laser()
	values = [2000 for num in range(360)]
	for value in msg:
		if int(value[1]) > 0:
			values[int(value[0])] = int(value[1])
	# Coger 5 posiciones: 0-36, 37-72, 73-108, 109-144, 145-180
	# Init
	res[0] = values[0]
	res[1] = values[36]
	res[2] = values[72]
	res[3] = values[108]
	res[4] = values[144]
	res[5] = values[180]
	res[6] = values[216]
	res[7] = values[252]
	res[8] = values[288]
	res[9] = values[324]

	for i in range(0,359):
		if i >= 0 and i <= 35:
			# Zona izquierda
			if values[i] < res[0] and values[i] != 0:
				res[0] = values[i]
		if i >= 36 and i <= 71:
			# Zona izquierda central
			if values[i] < res[1] and values[i] != 0:
				res[1] = values[i]
		if i >= 72 and i <= 107:
			# Zona central
			if values[i] < res[2] and values[i] != 0:
				res[2] = values[i]
		if i >= 108 and i <= 143:
			# Zona derecha central
			if values[i] < res[3] and values[i] != 0:
				res[3] = values[i]
		if i >= 144 and i <= 179:
			# Zona derecha
			if values[i] < res[4] and values[i] != 0:
				res[4] = values[i]
		if i >= 180 and i <= 215:
			# Zona izquierda
			if values[i] < res[5] and values[i] != 0:
				res[5] = values[i]
		if i >= 216 and i <= 251:
			# Zona izquierda central
			if values[i] < res[6] and values[i] != 0:
				res[6] = values[i]
		if i >= 252 and i <= 287:
			# Zona central
			if values[i] < res[7] and values[i] != 0:
				res[7] = values[i]
		if i >= 288 and i <= 323:
			# Zona derecha central
			if values[i] < res[8] and values[i] != 0:
				res[8] = values[i]
		if i >= 324 and i <= 359:
			# Zona derecha
			if values[i] < res[9] and values[i] != 0:
				res[9] = values[i]

	res[0]=res[0]/10
	res[1]=res[1]/10
	res[2]=res[2]/10
	res[8]=res[8]/10
	res[9]=res[9]/10
	return res

# -------------------------------------------------------------------------------
# MAIN
# -------------------------------------------------------------------------------
if __name__ == "__main__":
	signal.signal(signal.SIGINT, handler)
	# Iniciamos -------------------------------
	global ser
	ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.05)
	print 'INI: SerialPort -> on'
	envia(ser, 'TestMode On',0.2)
	envia(ser, 'PlaySound 1')
	print 'INI: TestMode -> on'
	envia(ser, 'SetMotor RWheelEnable LWheelEnable')
	print 'INI: SetMotor -> on'
	envia(ser, 'SetLDSRotation On',0.2,False)
	time.sleep(3)
	print 'INI: SetLaser -> on'
	print '----------------------------'
	# Config inicial -------------------------------
	global L_ini, R_ini
	L_ini, R_ini = get_motors()
	time.sleep(0.2)

	# Variables - - - - - - - - - - - - - - - - - - - -
	initialVelocity = 0.7*velocity_max

	# Variables - - - - - - - - - - - - - - - - - - - -
	

	
	print(getLaserValues())
	print('----------')
	print(search_max())

	# Finalizar -------------------------------
	print '----------------------------'
	envia(ser, 'PlaySound 2')
	envia(ser, 'TestMode Off',0.2)
	print 'FIN: TestMode -> off'
	envia(ser, 'SetLDSRotation Off',0.2,False)
	time.sleep(3)
	print 'FIN: SetLaser -> off'
	# Close the Serial Port.
	ser.close()  
	print 'FIN: SerialPort -> off'    
	print "FIN: Programa finalizado"