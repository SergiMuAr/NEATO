# To import the function "envia" from the file "test_NeatoCommands.py"
from test_NeatoCommands import envia

import serial
import time
import numpy as np

S = 121.5
wheel_rad = 38.5
x_world = 0
y_world = 0
theta_world = 0


def get_motors():
    """ Ask to the robot for the current state of the motors. """
    msg = envia(ser, 'GetMotors LeftWheel RightWheel').split('\n')
    
    # For better understanding see the neato commands PDF.
    
    L = int(msg[4].split(',')[1])
    R = int(msg[8].split(',')[1])
    
    return (L, R)



def odometry(L, R):
	#Implement the pos integration. Assume initial conditions [0,0,0].

	#Use global variables, discoment this line
	#global VARIABLES
    global x_world, y_world, theta_world
    global L_ini, R_ini
    new_L, new_R = L - L_ini, R - R_ini
    L_ini = L
    R_ini = R
    envia(ser, 'PlaySound 19')
    delta_d = (new_L+new_R)/2.
    delta_theta = (new_R-new_L)/243.
    x_world += delta_d*np.cos(theta_world)
    y_world += delta_d*np.sin(theta_world)
    theta_world += delta_theta
    theta_world = np.mod(theta_world, 2*np.pi)
    #print(new_L, new_R)
    #print(x_world, y_world, theta_world)
    #print(L, R)
    return [x_world, y_world, theta_world]



def move(X, Y, Odo):
	#Moves the robot to the point X, Y
	

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
        return PostOdo

if __name__ == "__main__":
	# Open the Serial Port.
	global ser
	ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.05)
	envia(ser, 'TestMode On')

	envia(ser, 'PlaySound 1')

	envia(ser ,'SetMotor RWheelEnable LWheelEnable')

	global L_ini, R_ini
	L_ini, R_ini = get_motors()
	
	speed = 250	# en mm/s
#        envia(ser, 'SetMotor LWheelDist '+ str(2*np.pi*S) +' RWheelDist ' + str(-2*np.pi*S) + ' Speed ' + str(speed))
#	envia(ser, 'SetMotor LWheelDist '+ str(519) +' RWheelDist ' + str(2046) + ' Speed ' + str(speed))


#	while True:
        L, R = get_motors()
	Odo = odometry(L, R)
	time.sleep(0.1)
	
        Odo = move(3000, 0, Odo)
        Odo = move(4000, 7000, Odo)
	Odo = move(3000, 0, Odo)
	Odo = move(0, 0, Odo)

        envia(ser, 'SetMotor LWheelDist '+ str(-np.pi*S) +' RWheelDist ' + str(np.pi*S) + ' Speed ' + str(speed))
        time.sleep(2)

        envia(ser, 'PlaySound 7')
	envia(ser, 'TestMode Off', 0.2)

	# Close the Serial Port.
	ser.close()      

	print "Final"
