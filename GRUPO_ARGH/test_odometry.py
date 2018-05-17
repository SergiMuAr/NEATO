# To import the function "envia" from the file "test_NeatoCommands.py"
from test_NeatoCommands import envia

import serial
import time
import numpy as np
import math 


#variables Neato
S = 121.5
wheel_rad = 38.5

# x,y,theta
pos_wordl = [[0],[0],[0]]

# x,y,theta estimada
pos_wordl_est = [[0],[0],[0]]

# covariance matrix
pos_pk = [[0,0,0],[0,0,0],[0,0,0]]

V = [[0.5, 0],[0, 0.008]]

def get_motors():
    """ Ask to the robot for the current state of the motors. """
    msg = envia(ser, 'GetMotors LeftWheel RightWheel').split('\n')
    
    # For better understanding see the neato commands PDF.
    
    L = int(msg[4].split(',')[1])
    R = int(msg[8].split(',')[1])
    
    return (L, R)

def get_laser():
	msg = envia(ser, 'GetLDSScan', 0.2, True)

	print("GetLDSScan ----------------------------")
	print msg

	var = []
	for line in msg.split('\r\n')[2:362]:
		s = line.split(',')
		var.append([s[0], s[1], s[2], s[3]])
	return var

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


	pos_pk = np.add(  np.dot(np.dot(Fx,pos_pk),np.transpose(Fx)),  np.dot(np.dot(Fv,V),np.transpose(Fv))  )
	#Calculamos la estimacion



	#Print & End
	print("--------------------------------------------------------")
	print("x_world: ",pos_wordl[0][0], pos_wordl[1][0], pos_wordl[2][0])
	print("x_world_est: ",pos_wordl_est[0][0], pos_wordl_est[1][0], pos_wordl_est[2][0])	
	return [pos_wordl[0][0], pos_wordl[1][0], pos_wordl[2][0]]





#-------------------------------------------------------------------------------
# LASER
#-------------------------------------------------------------------------------
def polar_to_cart(laser):
	cart = []
	print("Polar_to_cart ----------------------------")
	for i in range (0,len(laser)): #Polar 2 Cartesian in Robot Reference Frame
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






#-------------------------------------------------------------------------------
# MOVE
#-------------------------------------------------------------------------------
def move(X, Y):
	#Moves the robot to the point X, Y
	L, R = get_motors()
	Odo = odometry(L,R)
	time.sleep(0.1)
	DistX = X - Odo[0]
        DistY = Y - Odo[1]
        ElAnguloQueHayEntreLosDos = np.arctan2(DistY,DistX)
        if Odo[2] - ElAnguloQueHayEntreLosDos > 2*np.pi - (Odo[2] - ElAnguloQueHayEntreLosDos):
            NewTheta = -(2*np.pi - (Odo[2] - ElAnguloQueHayEntreLosDos))
        else: 
	    NewTheta = Odo[2] - ElAnguloQueHayEntreLosDos

        print(ElAnguloQueHayEntreLosDos)
        print(DistX, DistY)
	#Turn
        envia(ser, 'SetMotor LWheelDist '+ str(NewTheta*S) +' RWheelDist ' + str(-NewTheta*S) + ' Speed ' + str(100))
        L, R = get_motors()
        PostOdo = odometry(L, R)
        time.sleep(0.1)
        PrevOdo = PostOdo[:]
        PrevOdo[0] = PrevOdo[0] + 1
        while PrevOdo != PostOdo :
            L, R = get_motors()
            PrevOdo = PostOdo[:]
            PostOdo = odometry(L, R)
            time.sleep(0.1)
        envia(ser, 'PlaySound 2')

	#Move
	Dist = np.sqrt(DistX*DistX + DistY*DistY)
	envia(ser, 'SetMotor LWheelDist '+ str(Dist) +' RwheelDist ' + str(Dist) + ' Speed ' + str(200))
        L, R = get_motors()
        PostOdo = odometry(L, R)
        time.sleep(0.1)
        PrevOdo = PostOdo[:]
        PrevOdo[0] = PrevOdo[0] + 1
        while PrevOdo != PostOdo :
            L, R = get_motors()
            PrevOdo = PostOdo[:]
            PostOdo = odometry(L, R)
            time.sleep(0.1)
        envia(ser, 'PlaySound 2')




#-------------------------------------------------------------------------------
# MAIN
#-------------------------------------------------------------------------------
if __name__ == "__main__":
	# Iniciamos -------------------------------
	global ser
	ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.05)
	envia(ser, 'TestMode On')

	envia(ser, 'PlaySound 1')

	envia(ser ,'SetMotor RWheelEnable LWheelEnable')

	# Configuracion inicial -------------------
	global L_ini, R_ini
	L_ini, R_ini = get_motors()
	
	speed = 100	# en mm/s

	L, R = get_motors()
	Odo = odometry(L, R)
	time.sleep(0.1)
	
	# Movimientos -----------------------------
	#move(2000, 0)
	#move(3700, -2500)
	#move(3000, -1000)
	#move(3000, 1000)
	#move(3700, 2500)
	#move(2000, 0)
	#move(0, 0)

	envia(ser, 'SetLDSRotation On',0.2,False)
	time.sleep(3)
	print("Info ya en cart ----------------------------")
	var = polar_to_cart(get_laser())
	print("Num ok:", len(var))
	print("Info ya en world ----------------------------")
	var = cart_to_world(var,Odo)
	print(var)

	envia(ser, 'PlaySound 2')
	envia(ser, 'TestMode Off', 0.2)

	# Close the Serial Port.
	ser.close()      

	print "Final"