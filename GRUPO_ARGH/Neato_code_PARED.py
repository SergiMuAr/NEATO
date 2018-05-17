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
calibration = 2
distDetection = 750
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
	#print("--------------------------------------------------------")
	#print("x_world: ",pos_wordl[0][0], pos_wordl[1][0], pos_wordl[2][0])
	#print("x_world_est: ",pos_wordl_est[0][0], pos_wordl_est[1][0], pos_wordl_est[2][0])	

	return [pos_wordl[0][0], pos_wordl[1][0], pos_wordl[2][0]]




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
	msg = envia(ser, 'GetLDSScan',0.2,True)
	#time.sleep(0.1)
	var = []
	for line in msg.split('\r\n')[2:362]:
		s = line.split(',')
		var.append([s[0], s[1], s[2], s[3]])
	return var

def get_sensors (laser):
	sensor = []
	for i in range (0,12):
		sensor.append(dist_max)
		for j in range (-15+i*30, -15 + 30+i*30):
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
	#global L_ini, R_ini
	#L_ini, R_ini = get_motors()
	#Odo = odometry(L_ini, R_ini)
	#time.sleep(0.2)


	# Movimientos -------------------------------
	#envia(ser, 'SetMotor LWheelDist '+ str(2000) +' RwheelDist ' + str(2000) + ' Speed ' + str(300))
	#time.sleep(1.0)
	#envia(ser, 'SetMotor LWheelDist '+ str(-1000) +' RwheelDist ' + str(1000) + ' Speed ' + str(100))

	# Laser -------------------------------
	initialVelocity = 0.7*velocity_max
	# Variables ----
	leftRotated = 0
	rightRotated = 0
	timeStepCount = 0
	timesRotated = 0

	restTime = 1
	restCount = 0

	IDLE = 0
	ROTATING = 1
	WAITING = 2

	state = IDLE
	# Variables ----

	MAX_SPEED = 150

	for i in range (0,100):
		var = get_sensors(get_laser())
		
		frontSensor = var[0]
		leftSensor = min(var[2],var[3],var[4])

		nearDeath = False
		if frontSensor < 600:
			leftMotor = MAX_SPEED/6
			rightMotor = -MAX_SPEED/6
			nearDeath = True

		if nearDeath:
			pass
		# Too close to the wall, we need to turn right.
		elif leftSensor < 250:
			leftMotor = MAX_SPEED
			rightMotor = MAX_SPEED * 0.9

		# Too far from the wall, we need to turn left.
		elif leftSensor > 300:
			leftMotor = MAX_SPEED*0.9
			rightMotor = MAX_SPEED

		# We are in the right direction.
		else:
			leftMotor = MAX_SPEED
			rightMotor = MAX_SPEED

		print 'v---v---v ',' v---v---v'
		print 'Front/Left: ',frontSensor,' / ',leftSensor
		print 'LeftMotor/RightMotor: ',leftMotor,' / ',rightMotor
		envia(ser, 'SetMotor LWheelDist '+ str(leftMotor) +' RwheelDist ' + str(rightMotor) + ' Speed ' + str(150))

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