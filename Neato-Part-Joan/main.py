
debug = True

#Hacemos import de los modulos necesarios
import os,sys
if not debug:
	import sys, tty, termios
from select import select

import time
import math

#Si teneis dudas sobre como funciona multiprocessing
#https://docs.python.org/3/library/multiprocessing.html
from multiprocessing import Process, Queue

import CommanderModule as CommanderModule
import OdometryModule as OdometryModule


#Esta funcion coje solo una tecla sin tener que pulsar enter
def getch():
	if not debug:
		fd = sys.stdin.fileno()
		old_settings = termios.tcgetattr(fd)

		try:
			tty.setraw(sys.stdin.fileno())
			ch = sys.stdin.read(1)
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
		
		print ch
		
	else:
		ch = raw_input("")
    
	return ch


verboseGeneral = True;

def printVerbose(str,verbose = False):
	if verbose:
		print str

if __name__ == '__main__':

	input_command = Queue()
	input_odometry = Queue()

	#Creamos los procesos
	printVerbose('Creating commander process...',verboseGeneral)
	commander = Process(target=CommanderModule.func, args=(input_command,input_odometry,))
	printVerbose('Creating odometry process...',verboseGeneral)
	odometry = Process(target=OdometryModule.func, args=(input_odometry,input_command,))

	#Arrancamos los procesos
	printVerbose('Starting commander process...',verboseGeneral)
	commander.start()
	time.sleep(0.1);
	printVerbose('Starting odometry process...',verboseGeneral)
	odometry.start()
	
	#Esperamos a que los procesos esten activos.
	time.sleep(0.1);
	
	key = ""
	quite = False
	while not quite:
		print 'Write a command:'
		key = getch()
		#key = raw_input("")
		
		if key == "q":
			quite = True
			input_command.put("q")
			input_odometry.put("q")
		
		elif key == "8":
			input_command.put("forward")
		
		elif key == "2":
			input_command.put("back")
			
		elif key == "4":
			input_command.put("left")
		
		elif key == "6":
			input_command.put("right")
			
		elif key == "5":
			input_command.put("stop")
			
		elif key == "b":
			input_command.put("backbase")
		elif key == "o":
			input_command.put("motors")
		#tiempo de espera para que el mensaje llege bien
		time.sleep(0.01);
			


	printVerbose('Waiting commander process...',verboseGeneral)
	commander.join()
	printVerbose('Waiting odometry process...',verboseGeneral)
	odometry.join()
	
	print 'Finish!'