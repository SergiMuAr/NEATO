# -------------------------------------------------------------------------------
# Jordi Fructos Hernandez
# -------------------------------------------------------------------------------
from test_NeatoCommands import envia

import serial
import time
import numpy as np
import math
import os, signal

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
	exit(-1)

# -------------------------------------------------------------------------------
# VARIABLES
# -------------------------------------------------------------------------------
# variables Neato
S = 121.5
wheel_rad = 38.5
dist_max = 4000
velocity_max = 300
calibration = 2
distDetection = 600
initialVelocity = 0.6*velocity_max
# x,y,theta
pos_wordl = [[0], [0], [0]]
# x,y,theta estimada
pos_wordl_est = [[0], [0], [0]]
# covariance matrix
pos_pk = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
# error
V = [[0.5, 0], [0, 0.008]]

# -------------------------------------------------------------------------------
# ODOMETRY
# -------------------------------------------------------------------------------
def odometry(L, R):
    # Calculamos la x_world
    global pos_wordl, pos_wordl_est, pos_pk
    global L_ini, R_ini

    new_L = L - L_ini
    new_R = R - R_ini
    L_ini = L
    R_ini = R

    delta_D = (new_R + new_L) / 2.
    delta_O = (new_R - new_L) / 243.

    pos_wordl[0][0] += delta_D * np.cos(pos_wordl[2][0])
    pos_wordl[1][0] += delta_D * np.sin(pos_wordl[2][0])
    pos_wordl[2][0] += delta_O
    pos_wordl[2][0] = np.mod(pos_wordl[2][0], 2 * np.pi)

    # Calculamos la x_world estimada
    cos_theta = np.cos(pos_wordl[2][0])
    sin_theta = np.sin(pos_wordl[2][0])
    delta_D_sin_delta_O = delta_D * sin_theta
    delta_D_cos_delta_O = delta_D * cos_theta

    Fx = [[1, 0, -delta_D_sin_delta_O],
          [0, 1, delta_D_cos_delta_O],
          [0, 0, 1]]

    Fv = [[cos_theta, -delta_D_sin_delta_O],
          [sin_theta, delta_D_cos_delta_O],
          [0, 1]]

    pos_wordl_est = np.add(pos_wordl_est, np.add(np.dot(Fx, np.subtract(pos_wordl, pos_wordl_est)),
                                                 np.dot(Fv, [[V[0][0]], [V[1][1]]])))

    # Calculamos la estimacion
    pos_pk = np.add(np.dot(np.dot(Fx, pos_pk), np.transpose(Fx)), np.dot(np.dot(Fv, V), np.transpose(Fv)))

    # Print
    # print("--------------------------------------------------------")
    # print("x_world: ",pos_wordl[0][0], pos_wordl[1][0], pos_wordl[2][0])
    # print("x_world_est: ",pos_wordl_est[0][0], pos_wordl_est[1][0], pos_wordl_est[2][0])

    return [pos_wordl[0][0], pos_wordl[1][0], pos_wordl[2][0]]

	
def get_laser():
	msg = envia(ser, 'GetLDSScan',0.2,False)
	var = []
	for line in msg.split('\r\n')[2:362]:
		s = line.split(',')
		var.append([s[0], s[1], s[2], s[3]])
	return var

# -------------------------------------------------------------------------------
# MOTOR FUNC
# -------------------------------------------------------------------------------
def get_motors():
    msg = envia(ser, 'GetMotors LeftWheel RightWheel').split('\n')
    L = int(msg[4].split(',')[1])
    R = int(msg[8].split(',')[1])

    return (L, R)

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

def move(X, Y):
    distX = 1000
    distY = 1000
    emergency = 0
    while abs(distX) > 55 or abs(distY) > 55:
        L, R = get_motors()
        Odo = odometry(L, R)
        var = get_5sensor(get_laser())
		
        centralLeftSensorValue = ((distDetection-var[3])/calibration if var[3] < distDetection else 0)
        outerLeftSensorValue = ((distDetection-var[4])/calibration  if var[4] < distDetection else 0)
        centralRightSensorValue = ((distDetection-var[1])/calibration if var[1] < distDetection else 0)
        outerRightSensorValue = ((distDetection-var[0])/calibration if var[0] < distDetection else 0)
        centralSensorValue = ((distDetection-var[2])/calibration if var[2] < distDetection else 0)

        left = (centralLeftSensorValue + outerLeftSensorValue)/2
        right = (centralRightSensorValue + outerRightSensorValue)/2
		
        if left != 0 or right != 0:
            minLeft = min(centralLeftSensorValue,outerLeftSensorValue)
            maxLeft = max(centralLeftSensorValue,outerLeftSensorValue)
            minRight = min(centralRightSensorValue,outerRightSensorValue)
            maxRight = max(centralRightSensorValue,outerRightSensorValue)

            if maxLeft < maxRight or (maxLeft == maxRight and maxLeft > maxRight):
                right = right + centralSensorValue
                if maxRight > distDetection/2:
                    emergency = 'L'
            else:
                left = left + centralSensorValue
                if maxLeft > distDetection/2:
                    emergency = 'R'

            if abs(left-right) < 5:
                right = right + centralSensorValue

            if left == 0 and right == 0:
                leftMotor = 150
                rightMotor = 150
                velocity = initialVelocity
            elif (abs(left)+abs(right)) < 50:
                leftMotor = initialVelocity - right
                rightMotor = initialVelocity - left
                velocity = 150
            else:
                leftMotor = initialVelocity - right
                rightMotor = initialVelocity - left
                velocity = 100

            print 'Left/Right: ',left,' / ',right
            print 'LeftMotor/RightMotor: ',leftMotor,' / ',rightMotor

            if emergency == 'L':
                print 'WAIT ------------------------------------------------------------------------------'
                leftMotor = -initialVelocity
                rightMotor = initialVelocity
            elif emergency == 'R':
                print 'WAIT'
                leftMotor = initialVelocity
                rightMotor = -initialVelocity
            emergency = 'N'

            envia(ser, 'SetMotor LWheelDist '+ str(leftMotor) +' RwheelDist ' + str(rightMotor) + ' Speed ' + str(velocity))
        else:
            #time.sleep(0.1)
            distX = X - Odo[0]
            distY = Y - Odo[1]
            print Odo
            print 'distX ' + str(distX) + ' distY ' + str(distY)
            distToGoal = np.sqrt(distX * distX + distY * distY)
            angleToGoal = np.arctan2(distY, distX) - Odo[2]
            if angleToGoal > 2 * np.pi - angleToGoal:
                angleToGoal = -(2 * np.pi - angleToGoal)
            elif angleToGoal < -np.pi:
                angleToGoal = 2*np.pi + angleToGoal

            print 'angleToGoal ' + str(angleToGoal) + ' distToGoal ' + str(distToGoal)
            # Movimiento
            velGain = distToGoal * 0.5
            angleGain = angleToGoal * 2.5
            print 'velGain ' + str(velGain) + ' angleGain ' + str(angleGain)
            dist_L = 1000
            dist_R = (angleGain * 2 * S) + dist_L
            speed = min(velGain,300)
            envia(ser, 'SetMotor LWheelDist ' + str(dist_L) + ' RwheelDist ' + str(dist_R) + ' Speed ' + str(speed))

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

	move(0, -1000)
	L, R = get_motors()
	Odo = odometry(L, R)
	print 'Ha acabado en ' + str(Odo[0]) + ' ' + str(Odo[1])
	envia(ser, 'PlaySound 2')
	time.sleep(1)
	#move(0,0)
	#L, R = get_motors()
	#Odo = odometry(L, R)
	#print 'Ha acabado en ' + str(Odo[0]) + ' ' + str(Odo[1])

	# Finalizar -------------------------------
	print '----------------------------'
	envia(ser, 'PlaySound 2')
	envia(ser, 'TestMode Off', 0.2)
	print 'FIN: TestMode -> off'
	# Close the Serial Port.
	ser.close()
	print 'FIN: SerialPort -> off'
	print "FIN: Programa finalizado"
