from test_NeatoCommands import envia

import serial
import time
import numpy as np
import math 

import os, signal
import sys, tty, termios
from select import select

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
	while True:
		var = get_5sensor(get_laser())

		centralLeftSensorValue = ((distDetection-var[3])/calibration if var[3] < distDetection else 0)
		outerLeftSensorValue = ((distDetection-var[4])/calibration  if var[4] < distDetection else 0)
		centralRightSensorValue = ((distDetection-var[1])/calibration if var[1] < distDetection else 0)
		outerRightSensorValue = ((distDetection-var[0])/calibration if var[0] < distDetection else 0)
		centralSensorValue = ((distDetection-var[2])/calibration if var[2] < distDetection else 0)

		left = (centralLeftSensorValue + outerLeftSensorValue)/2
		right = (centralRightSensorValue + outerRightSensorValue)/2

		minLeft = min(centralLeftSensorValue,outerLeftSensorValue)
		maxLeft = max(centralLeftSensorValue,outerLeftSensorValue)
		minRight = min(centralRightSensorValue,outerRightSensorValue)
		maxRight = max(centralRightSensorValue,outerRightSensorValue)

		if maxLeft < maxRight or (maxLeft == maxRight and maxLeft > maxRight):
			print 'GIRAR IZQUIERDA -------------------------------------'	
			right = right + centralSensorValue
		else:
			print 'GIRAR DERECHA ---------------------------------------'	
			left = left + centralSensorValue

		if abs(left-right) < 5:
			right = right + centralSensorValue

		if left == 0 and right == 0:
			leftMotor = 150
			rightMotor = 150
			velocity = initialVelocity
		elif (abs(left)+abs(right)) < 50:
			leftMotor = max(1000 - right, 0)
			rightMotor = max(1000 - left, 0)
			velocity = 150
		else:
			leftMotor = max(1000 - right, 0)
			rightMotor = max(1000 - left, 0)
			velocity = 100

		print 'Left/Right: ',left,' / ',right
		print 'LeftMotor/RightMotor: ',leftMotor,' / ',rightMotor

		envia(ser, 'SetMotor LWheelDist '+ str(rightMotor) +' RwheelDist ' + str(leftMotor) + ' Speed ' + str(initialVelocity))

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