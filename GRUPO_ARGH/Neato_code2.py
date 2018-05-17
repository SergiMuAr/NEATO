# -------------------------------------------------------------------------------
# Jordi Fructos Hernandez
# -------------------------------------------------------------------------------
from test_NeatoCommands import envia

import serial
import time
import numpy as np
import math 




# -------------------------------------------------------------------------------
# VARIABLES
# -------------------------------------------------------------------------------
#variables Neato
S = 121.5
wheel_rad = 38.5
dist_max = 4000
velocity_max = 300
distanceCalibration = 5
# x,y,theta
pos_wordl = [[0],[0],[0]]
# x,y,theta estimada
pos_wordl_est = [[0],[0],[0]]
# covariance matrix
pos_pk = [[0,0,0],[0,0,0],[0,0,0]]
# error
V = [[0.5, 0],[0, 0.008]]




#-------------------------------------------------------------------------------
# ODOMETRY
#-------------------------------------------------------------------------------
def odometry(L, R):
	#Calculamos la x_world
	global pos_wordl, pos_wordl_est, pos_pk
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


	#Calculamos la x_world estimada
	cos_theta = np.cos(pos_wordl[2][0])
	sin_theta = np.sin(pos_wordl[2][0])
	delta_D_sin_delta_O = delta_D*sin_theta
	delta_D_cos_delta_O = delta_D*cos_theta

	Fx = [	[1, 0, 	-delta_D_sin_delta_O],
			[0,	1,	delta_D_cos_delta_O],
			[0, 0, 	1] ]

	Fv = [	[cos_theta, -delta_D_sin_delta_O],
			[sin_theta,	delta_D_cos_delta_O],
			[0, 1] ]

	pos_wordl_est = np.add(pos_wordl_est,np.add(np.dot(Fx,np.subtract(pos_wordl,pos_wordl_est)),np.dot(Fv,[[V[0][0]],[V[1][1]]])))

	#Calculamos la estimacion
	pos_pk = np.add(  np.dot(np.dot(Fx,pos_pk),np.transpose(Fx)),  np.dot(np.dot(Fv,V),np.transpose(Fv))  )

	#Print
	print("--------------------------------------------------------")
	print("x_world: ",pos_wordl[0][0], pos_wordl[1][0], pos_wordl[2][0])
	print("x_world_est: ",pos_wordl_est[0][0], pos_wordl_est[1][0], pos_wordl_est[2][0])	

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
	time.sleep(0.2)
	PrevOdo = Odo[:]
	PrevOdo[0] = PrevOdo[0] + 1
	while PrevOdo != Odo:
		L, R = get_motors()
		PrevOdo = Odo[:]
		Odo = odometry(L, R)
		time.sleep(0.2)
		envia(ser, 'PlaySound 2')


def move(X, Y):
	distX = 1
	distY = 1
	while distX != 0 and distY != 0:
		L, R = get_motors()
		Odo = odometry(L,R)
		time.sleep(0.2)
		distX = X - Odo[0]
		distY = Y - Odo[1]
		distToGoal = np.sqrt(distX*distX + distY*distY)
		angleToGoal = Odo[2] - np.arctan2(distY,distX)
		# Movimiento
		velGain = distToGoal*0.5
		angleGain = angleToGoal*0.2


# -------------------------------------------------------------------------------
# LASER FUNC
# -------------------------------------------------------------------------------
def get_laser():
	msg = envia(ser, 'GetLDSScan',0.2,True)
	var = []
	for line in msg.split('\r\n')[2:362]:
		s = line.split(',')
		var.append([s[0], s[1], s[2], s[3]])
	return var

def polar_to_cart(laser):
	cart = []
	for i in range (0,len(laser)):
		if int(laser[i][3]) == 0:
			angulo = i*np.pi/180 
			x_r = int(laser[i][1])*np.cos(angulo)/1000
			y_r = int(laser[i][1])*np.sin(angulo)/1000
			cart.append([x_r, y_r, 1])

	return cart

def cart_to_world(coord,odo):
	x = odo[0]
	y = odo[1]
	theta = odo[2]
	TAB = [	[np.cos(theta),-np.sin(theta), x],
			[np.sin(theta), np.cos(theta), y],
			[0, 0, 1]	]
	world = np.dot(TAB,np.transpose(coord))

	return world

def get_5sensor (laser):
	sensor = []
	for i in range (0,5):
		sensor.append(dist_max)
		for j in range (-90 + 30+i*24, -90 + 54+i*24):
			if int(laser[j][3]) == 0:
				#print laser[j][1],' < ',sensor[i]
				if (int(laser[j][1]) < sensor[i]):
					sensor[i] = int(laser[j][1])

	return sensor



# -------------------------------------------------------------------------------
# MAIN
# -------------------------------------------------------------------------------
if __name__ == "__main__":
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
	Odo = odometry(L_ini, R_ini)
	time.sleep(0.2)


	# Movimientos -------------------------------
	#move(2000, 0)
	#move(3700, -2500)
	#move(3000, -1000)
	#move(3000, 1000)
	#move(3700, 2500)
	#move(2000, 0)
	#move(0, 0)

	# Laser -------------------------------
	print("Obtenemos los 5 sensores ----------------------------")
	initialVelocity = 0.7*velocity_max
	# Variables ----
	leftRotated = 0
	rightRotated = 0
	timeStepCount = 0
	timesRotated = 0

	restTime = 20
	restCount = 0

	IDLE = 0
	ROTATING = 1
	WAITING = 2

	state = IDLE

	threshold = 0
	# Variables ----

	#while True:
	for i in range (0,100):
		var = get_5sensor(get_laser())
		print var
		centralLeftSensorValue = (var[3]/distanceCalibration if var[3] < 1000 else 0)
		outerLeftSensorValue = (var[4]/distanceCalibration if var[4] < 1000 else 0)
		centralRightSensorValue = (var[1]/distanceCalibration if var[1] < 1000 else 0)
		outerRightSensorValue = (var[0]/distanceCalibration if var[0] < 1000 else 0)
		centralSensorValue = (var[2]/distanceCalibration if var[2] < 1000 else 0)

		left = (centralLeftSensorValue + outerLeftSensorValue)/2 - centralSensorValue
		right = (centralRightSensorValue + outerRightSensorValue)/2

		leftMotor = initialVelocity - right
		rightMotor = initialVelocity - left
		
		envia(ser, 'SetMotor LWheelDist '+ str(leftMotor) +' RwheelDist ' + str(rightMotor) + ' Speed ' + str(200))

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