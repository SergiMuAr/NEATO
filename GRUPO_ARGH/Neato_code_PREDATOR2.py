from test_NeatoCommands import envia

import serial
import time
import numpy as np
import math 


import os, signal
import sys, tty, termios
from select import select





# -------------------------------------------------------------------------------
# VARIABLES
# -------------------------------------------------------------------------------
#variables Neato
S = 121.5
wheel_rad = 38.5
dist_max = 4000
velocity_max = 300
calibration = 1
distDetection = 600
# x,y,theta
pos_wordl = [[0],[0],[0]]
# laser distance
laserPos = []
# odometry ini
L_ini = 0
R_ini = 0


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:

        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:

        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
	print ch
    return ch

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

#-------------------------------------------------------------------------------
# ODOMETRY
#-------------------------------------------------------------------------------
def reset_odometry():
	global pos_wordl, L_ini, R_ini
	pos_wordl = [[0],[0],[0]]
	L_ini, R_ini = get_motors()

def odometry(L, R):
	#Calculamos la x_world
	global pos_wordl
	global L_ini, R_ini

	new_L = L - L_ini
	new_R = R - R_ini
	L_ini = L
	R_ini = R

	delta_D = (new_R + new_L)/2.
	delta_O = (new_R - new_L)/243.

	pos_wordl[0][0] += delta_D*np.cos(pos_wordl[2][0])
	pos_wordl[1][0] += delta_D*np.sin(pos_wordl[2][0])
	pos_wordl[2][0] += delta_O 
	pos_wordl[2][0] = np.mod(pos_wordl[2][0],2*np.pi)

	return [pos_wordl[0][0], pos_wordl[1][0], pos_wordl[2][0]]

# -------------------------------------------------------------------------------
# MOTOR FUNC
# -------------------------------------------------------------------------------
def get_motors():
    msg = envia(ser, 'GetMotors LeftWheel RightWheel').split('\n')    
    L = int(msg[4].split(',')[1])
    R = int(msg[8].split(',')[1])
    
    return (L, R)

def wait_motors():
	L, R = get_motors()
	Odo = odometry(L, R)
	PrevOdo = Odo[:]
	PrevOdo[0] = PrevOdo[0] + 1
	while PrevOdo != Odo:
		L, R = get_motors()
		PrevOdo = Odo[:]
		Odo = odometry(L, R)

def move(X, Y):
	#Moves the robot to the point X, Y
        difTheta = np.arctan2(Y,X)
        difThetaInv = difTheta + 2*np.pi
        #if Odo[2] - ElAnguloQueHayEntreLosDos > 2*np.pi - (Odo[2] - ElAnguloQueHayEntreLosDos):
        if - difTheta > difThetaInv:
            NewTheta = - difThetaInv
        else: 
	    	NewTheta = - difTheta
	#Turn
        envia(ser, 'SetMotor LWheelDist '+ str(NewTheta*S) +' RWheelDist ' + str(-NewTheta*S) + ' Speed ' + str(150))
        L, R = get_motors()
        PostOdo = odometry(L, R)
        time.sleep(0.1)
        PrevOdo = PostOdo[:]
        PrevOdo[0] = PrevOdo[0] + 1
        while PrevOdo != PostOdo :
            L, R = get_motors()
            PrevOdo = PostOdo[:]
            PostOdo = odometry(L, R)
            #time.sleep(0.1)

	#Move
	Dist = np.sqrt(X*X + Y*Y)*0.85
	envia(ser, 'SetMotor LWheelDist '+ str(Dist) +' RwheelDist ' + str(Dist) + ' Speed ' + str(300))
        L, R = get_motors()
        PostOdo = odometry(L, R)
        time.sleep(0.1)
        PrevOdo = PostOdo[:]
        PrevOdo[0] = PrevOdo[0] + 1
        while PrevOdo != PostOdo :
            L, R = get_motors()
            PrevOdo = PostOdo[:]
            PostOdo = odometry(L, R)
            #time.sleep(0.1)
        return PostOdo

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

# -------------------------------------------------------------------------------
# PREDATOR FUNC
# -------------------------------------------------------------------------------

def ini_sensor ():
	global laserPos
	for i in range (0,360):
		laserPos.append(dist_max)

def reset_sensor (laser):			
	global laserPos
	for i in range (0,360):
		if int(laser[i][3]) == 0 and int(laser[i][1]) <= dist_max:
			laserPos[i] = int(laser[i][1])
		else:
			laserPos[i] = dist_max

def detect_sensor (laser):
	global laserPos
	aux = [4000,0]
	auxVector = []
	for i in range (0,360):
		auxVector.append([0,0,0])
		distCurrentLaser = int(laser[i][1])
		if int(laser[i][3]) == 0 and distCurrentLaser <= dist_max:
			if laserPos[i] != dist_max:
				difDist = abs(distCurrentLaser - laserPos[i])
				if 50 < difDist: #Detectado movimiento
					auxVector[i][0] = 1
					auxVector[i][1] = distCurrentLaser
					auxVector[i][2] = i
			else:
				laserPos[i] = distCurrentLaser

	for i in range (0,360):
		if auxVector[i-1][0] and auxVector[i][0] and auxVector[i+1][0]:
			print i
			if auxVector[i][1] < aux[0]: #Buscando movimiento mas cercano
				aux = [auxVector[i][1],auxVector[i][2]]	
				print i,' >>> '	

	print 'MOVE >>>>>>>>>>>>>>>>>> ', aux	

	return aux			

def move_predator (dist,theta):
	reset_odometry()
	x = dist*np.cos(np.pi*theta/180)
	y = dist*np.sin(np.pi*theta/180)
	move(x,y)
	reset_sensor(get_laser())



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
	# START -------------------------------

	ini_sensor()
	reset_sensor(get_laser())
	while True:
		[dist,theta] = detect_sensor(get_laser())
		if (dist != 4000):
			move_predator(dist,theta)

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