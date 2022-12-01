import sys
import traceback
import time
import os
import math
from zmqRemoteApi import RemoteAPIClient
import zmq

def control_logic(sim):

	i = 0
	pi = 3.1416
	finish = 0					#will raise to 1 if the robot is at the final position in the simulation.
	turn = 0					#flag to determine the rotation mode is ON or OFF.
	normal_speed = 8			#the maximum speed that the vehicle can accelrate upto.
	offset = 0.300				#front sensor distance value after which the robot has to stop to rotate of detect whether it has reached the final position.
	m = 2						#sensor_variable, to determine which sensor is to be the robot in center of the lane and detect whether roatated 90 degree or not.
	low_band_r = 0.168			
	high_band_r = 0.182

	low_band_l = 0.195
	high_band_l = 0.205

	rot_speed = 0.2				#rotaional speed of vehicle, perfectly optimized to get sensor values with maximum time difference that will not deflect the anle more than 3 degrees.
	min_dist = 0.00005			# is not being used currently in the code.
	slow_speed = 3				#is not being used currently in the code.
	diff = 0.2					#differential speed that is applied on the wheel
	sp = 1						#the actual speed that is being applied to the vehicle.
	differential = 0			#flag to understand whether the differential speed is applied or not.
	distance = 0				#flag to determine whether the front sensor is detecting any object or not.

	left_joint = sim.getObjectHandle('./left_joint')
	right_joint = sim.getObjectHandle('./right_joint')
	sim.setJointTargetVelocity(left_joint, 0)
	sim.setJointTargetVelocity(right_joint, 0)
	
	while(finish == 0):
		while(turn == 0):
			if(sp < normal_speed and distance != 1): 
				if(differential == 2 or differential == 4):
					ldiff = 0
					rdiff = diff
				elif(differential == 1 or differential == 3):
					ldiff = diff
					rdiff = 0
				else:
					ldiff = 0
					rdiff = 0
				
				
				if(sp%2 == 0):
					sim.setJointTargetVelocity(left_joint,sp + ldiff)
					sim.setJointTargetVelocity(right_joint,sp + rdiff)
					#print(f'speed {sp} + {sp + ldiff}')
				else:
					sim.setJointTargetVelocity(right_joint,sp+ rdiff)
					sim.setJointTargetVelocity(left_joint,sp + ldiff)
					#print(f'speed {sp} + {sp + ldiff}')
				sp = sp + 1.75

			sensor_1 = detect_distance_sensor_1(sim)
			sensor_2 = detect_distance_sensor_2(sim)
			sensor_3 = detect_distance_sensor_3(sim)
			
			if(m == 2):
				if(differential == 0 ):
					if(sensor_2[1]<low_band_r or sensor_2[1]> high_band_r):
						if(sensor_2[1]> high_band_r and differential != 1):
							sim.setJointTargetVelocity(left_joint, sp + diff)
							sim.setJointTargetVelocity(right_joint, sp)
							differential = 1
					
						elif(differential != 2):
							sim.setJointTargetVelocity(right_joint, sp + diff)
							sim.setJointTargetVelocity(left_joint, sp)
							differential = 2
				else:
					if(sensor_2[1]>low_band_r and sensor_2[1]< high_band_r):
						sim.setJointTargetVelocity(left_joint, sp)
						sim.setJointTargetVelocity(right_joint, sp)
						differential = 0
			else:
				if(differential == 0 ):
					if(sensor_3[1]< low_band_l or sensor_3[1]> high_band_l ):
						if(sensor_3[1]> high_band_l and differential != 4):
							sim.setJointTargetVelocity(right_joint,sp + diff)
							sim.setJointTargetVelocity(left_joint, sp )
							differential = 4
					
						elif(differential != 3):
							sim.setJointTargetVelocity(left_joint, sp + diff)
							sim.setJointTargetVelocity(right_joint,sp)
							differential = 3
				else:
					if(sensor_3[1]>low_band_l and sensor_3[1]<high_band_l):
						sim.setJointTargetVelocity(left_joint, sp)
						sim.setJointTargetVelocity(right_joint, sp)
						differential = 0

			sensor_1 = detect_distance_sensor_1(sim)
			distance = sensor_1[0]
			if(distance == 1 ):
				if(sp >= 4):
					sp = sp/4
				sim.setJointTargetVelocity(left_joint, sp)
				sim.setJointTargetVelocity(right_joint, sp)
				if(sensor_1[1]<=offset):
					turn = 1
					sp = 1
		differential = 0
		
		
		sim.setJointTargetVelocity(left_joint, 0)
		sim.setJointTargetVelocity(right_joint,0)
		
		#yaha pe sensor_1 jis object pe se distance batayega ussi object pe se side sensor kar 
		#same distance aaya rotate karne ke baad matlab robot 90 degree ghuma hai bara bar se...
		sensor_1 = detect_distance_sensor_1(sim)
		sensor_2 = detect_distance_sensor_2(sim)
		sensor_3 = detect_distance_sensor_3(sim)
		#print(f'sensor_1 se value mila {sensor_1}')
		#ye upar wale function ke bohot saare output hai check karlo acche se..
		
		#print(f'Suface Vector {sensor_1[4]}')
		
		if(detect_distance_sensor_2(sim)[0] == 0):
			sim.setJointTargetVelocity(left_joint, rot_speed)
			sim.setJointTargetVelocity(right_joint,-rot_speed)
			#print('clock_wise_ghumaya')
			m = 3
		elif(detect_distance_sensor_3(sim)[0] == 0):
			sim.setJointTargetVelocity(left_joint, -rot_speed)
			sim.setJointTargetVelocity(right_joint,rot_speed)
			#print('anti_clock_wise_ghumaya')
			m = 2
		else:
			#agar dono sensor wall detect karte hai jab front wala wall detect karta hai iska matlab humlog 
			#stop position pe hai, to kaam khatam
			finish = 1

		min = 0
		ninty_degree = 0
		if(finish == 0):
		#second while loop me jabtak side sensor ki value sensor_1 ki value ke barabar 
		#nahi hoti for the same object as it was before the robot started rotating
		#tab tak robot ko ghumate rehna hai(yane while loop ke bahar nahi aayega, abhi tumlog ko 
		# secodn while loop ka logic build karna hai upar ke hisab se.)
	
			if(m == 3):
			
				while(ninty_degree == 0):
					r_sensor = detect_distance_sensor_3(sim)
					if(r_sensor[3] == sensor_1[3]):
						if(angle(r_sensor) == 1):
							ninty_degree = 1
							break
							
			
			else:
	
				while(ninty_degree == 0):
					r_sensor = detect_distance_sensor_2(sim)
					if(r_sensor[3] == sensor_1[3]):
						if(angle(r_sensor) == 1):
							#print(f'second Surface vector{r_sensor[4]}')
							ninty_degree = 1
							break
								
		
			ninty_degree = 0
			
			turn = 0

def angle(r_sensor):
	pi = 3.1416
	dot = r_sensor[2][0]*r_sensor[4][0] + r_sensor[2][1]*r_sensor[4][1] +r_sensor[2][2]*r_sensor[4][2]
	mag = ((r_sensor[2][0])**2 + (r_sensor[2][1])**2 + (r_sensor[2][2])**2)**(1/2) * 1   #((r_sensor[4][0])**2 + (r_sensor[4][1])**2 + (r_sensor[4][2])**2)**(1/2)
	ang = (math.acos(dot/mag))*(180/pi)
	if(ang >= 177):
		c = 1
	else: c = 0
	return c


def detect_distance_sensor_1(sim):

	distance = None
	distance = 0
	sensor_1 = sim.getObjectHandle('./distance_sensor_1')
	distance_sensor_a = sim.readProximitySensor(sensor_1)
	if distance_sensor_a == 0:
		distance = -1
	else:
		distance = distance_sensor_a

	return distance

def detect_distance_sensor_2(sim):

	distance = 0
	sensor_2 = sim.getObjectHandle('./distance_sensor_2')
	distance_sensor_b = sim.readProximitySensor(sensor_2)
	if distance_sensor_b == 0:
		distance = -1
	else:
		distance = distance_sensor_b

	return distance

def detect_distance_sensor_3(sim):

	distance = 0
	sensor_3 = sim.getObjectHandle('./distance_sensor_3')
	distance_sensor_c = sim.readProximitySensor(sensor_3)
	if distance_sensor_c == 0:
		distance = -1
	else:
		distance = distance_sensor_c
	
	return distance

if __name__ == "__main__":
	client = RemoteAPIClient()
	sim = client.getObject('sim')

	try:

		## Start the simulation using ZeroMQ RemoteAPI
		try:
			return_code = sim.startSimulation()
			if sim.getSimulationState() != sim.simulation_stopped:
				print('\nSimulation started correctly in CoppeliaSim.')
			else:
				print('\nSimulation could not be started correctly in CoppeliaSim.')
				sys.exit()

		except Exception:
			print('\n[ERROR] Simulation could not be started !!')
			traceback.print_exc(file=sys.stdout)
			sys.exit()

		## Runs the robot navigation logic written by participants
		try:
			control_logic(sim)
			time.sleep(3)

		except Exception:
			print('\n[ERROR] Your control_logic function throwed an Exception, kindly debug your code!')
			print('Stop the CoppeliaSim simulation manually if required.\n')
			traceback.print_exc(file=sys.stdout)
			print()
			sys.exit()

		
		## Stop the simulation using ZeroMQ RemoteAPI
		try:
			return_code = sim.stopSimulation()
			time.sleep(0.5)
			if sim.getSimulationState() == sim.simulation_stopped:
				print('\nSimulation stopped correctly in CoppeliaSim.')
			else:
				print('\nSimulation could not be stopped correctly in CoppeliaSim.')
				sys.exit()

		except Exception:
			print('\n[ERROR] Simulation could not be stopped !!')
			traceback.print_exc(file=sys.stdout)
			sys.exit()

	except KeyboardInterrupt:
		## Stop the simulation using ZeroMQ RemoteAPI
		return_code = sim.stopSimulation()
		time.sleep(0.5)
		if sim.getSimulationState() == sim.simulation_stopped:
			print('\nSimulation interrupted by user in CoppeliaSim.')
		else:
			print('\nSimulation could not be interrupted. Stop the simulation manually .')
			sys.exit()