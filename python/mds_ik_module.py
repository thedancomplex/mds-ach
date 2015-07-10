import numpy as np
from struct import pack
from struct import unpack
import mds_ach as mds
import pylab
import ach
import time
import signal
from mds_ik import * 
import sys
import os
import math
from ctypes import *
import copy
from numpy.linalg import inv
import mds_ik_include as ike
import mds_ach as ha
from mds_ach import *


#define global variables
threshold = .0025
l1 = 0.2455100   # center of body to shoulder
l2 = 0.2825750   # shoulder to eblow
l3 = 0.3127375   # elbow to wrist roll
alpha = .5
dT1 = .001		
dT2 = .001
dT3 = .001
#open ach channel for coodinates being sent from file
coordinates = ike.CONTROLLER_REF()
coordinates.x = -0.25
coordinates.y = -0.20
coordinates.z =  0.20
# feed-forward will now be refered to as "state"
state = ha.MDS_REF()
#initial feed-back
initial_ref = ha.MDS_REF()
# feed-back will now be refered to as "ref"
ref = ha.MDS_REF()
# Command line for console
command = ha.MDS_CON2IK()
#bend elbow to start to force jacobian to solve in righ direction
#avoid singularity
ref.joint[ha.REB].ref = -math.pi/6	
ref.joint[ha.LEB].ref = -math.pi/6
# rotation matrix definition (abotu x, y, or z)
def_x = 1
def_y = 2
def_z = 3
#define arm constant
arm = 'left'

#Needed for name resolution
state = mds.MDS_STATE()


#Current Status: 
#3 DOF Capabilities

#git checkout -b feature /smoothing
#git push origin branch name


def mainLoop():
	eff_end	= np.array([0, 0, 0])
	eff_joint_space_L = [0.0, 0.0, 0.0, -0.5, 0.0, 0.0]
	eff_joint_space_R = [0.0, 0.0, 0.0, -0.5, 0.0, 0.0]
	m = ach.Channel(MDS_CHAN_REF_NAME)
	c = ach.Channel(MDS_CHAN_CON2IK_NAME)
	r = ach.Channel(MDS_CHAN_REF_FILTER_NAME)
	state = mds.MDS_STATE()
	#Get initial references	
	m.get(initial_ref)
	#get initial joint space for left arm
	eff_joint_space_L[0] = initial_ref.joint[mds.getAddress('LSP',state)].ref
	eff_joint_space_L[1] = initial_ref.joint[mds.getAddress('LSR',state)].ref
	eff_joint_space_L[2] = initial_ref.joint[mds.getAddress('LSY',state)].ref
	eff_joint_space_L[3] = initial_ref.joint[mds.getAddress('LEB',state)].ref
	eff_joint_space_L[4] = initial_ref.joint[mds.getAddress(LWY,state)].ref
	eff_joint_space_L[5] = initial_ref.joint[mds.getAddress(LWR,state)].ref
	#get initial joint space for right arm
	eff_joint_space_R[0] = initial_ref.joint[mds.getAddress(RSP,state)].ref
	eff_joint_space_R[1] = initial_ref.joint[mds.getAddress(RSR,state)].ref
	eff_joint_space_R[2] = initial_ref.joint[mds.getAddress(RSY,state)].ref
	eff_joint_space_R[3] = initial_ref.joint[mds.getAddress(REB,state)].ref
	eff_joint_space_R[4] = initial_ref.joint[mds.getAddress(RWY,state)].ref
	eff_joint_space_R[5] = initial_ref.joint[mds.getAddress(RWR,state)].ref
	print(mds.getAddress('LSY',state));
	#take user input
	while(1):
		# Define IK 3 coord xyz
		#arm = 'left'
		#arm_type = 0
		#USE THE ONE BELOW
		# Get IK 3DOF
		c.get(command)
		if(command.arm == 0):
			arm = 'left'
			arm_type = 0
			eff_joint_space_current=eff_joint_space_L
			print arm
		elif(command.arm == 1):
			arm = 'right'
			arm_type = 1
			eff_joint_space_current=eff_joint_space_R
			print arm
		else:
			arm = 'left'
			arm_type = 0
			eff_joint_space_current=eff_joint_space_L
			print arm
		#print "HERE: " + str(command.x) + str(command.y) + str(command.z)
		eff_end = np.array([command.x, command.y, command.z])
		eff_joint_space_current = getIK3dof(eff_joint_space_current, eff_end, arm)
		# Print result for reference
		A = getFkArm(eff_joint_space_current,arm)
		eff_end_ret = np.array([ A[0,3], A[1,3], A[2,3]])
		eff_end_dif = eff_end - eff_end_ret
		#send the solved ik to simulation
		if(arm_type == 0):
			ref.joint[mds.getAddress(LSP,state)].ref = eff_joint_space_current[0]
			ref.joint[mds.getAddress(LSR,state)].ref = eff_joint_space_current[1]
			ref.joint[mds.getAddress(LSY,state)].ref = eff_joint_space_current[2]
			ref.joint[mds.getAddress(LEB,state)].ref = eff_joint_space_current[3]
			ref.joint[mds.getAddress(LWY,state)].ref = eff_joint_space_current[4]
			ref.joint[mds.getAddress(LWR,state)].ref = eff_joint_space_current[5]
		else:
			ref.joint[mds.getAddress(RSP,state)].ref = eff_joint_space_current[0]
			ref.joint[mds.getAddress(RSR,state)].ref = eff_joint_space_current[1]
			ref.joint[mds.getAddress(RSY,state)].ref = eff_joint_space_current[2]
			ref.joint[mds.getAddress(REB,state)].ref = eff_joint_space_current[3]
			ref.joint[mds.getAddress(RWY,state)].ref = eff_joint_space_current[4]
			ref.joint[mds.getAddress(RWR,state)].ref = eff_joint_space_current[5]	
		#print("\n"+"JOINT ANGLES:   ")
		#print(eff_joint_space_current) #+ " " +str(eff_joint_space_current[1])+ " " +str(eff_joint_space_current[2])+ " " +str(eff_joint_space_current[3])+ " " +str(eff_joint_space_current[4])+ " " +str(eff_joint_space_current[5]))
		#print 'des:\tx = ', round(eff_end[0],5)     , '\ty = ' , round(eff_end[1],5)     , '\tz = ', round(eff_end[2],5)
		#print 'ret:\tx = ', round(eff_end_ret[0],5) , '\ty = ' , round(eff_end_ret[1],5) , '\tz = ', round(eff_end_ret[2],5)
		#print 'dif:\tx = ', round(eff_end_dif[0],5) , '\ty = ' , round(eff_end_dif[1],5) , '\tz = ', round(eff_end_dif[2],5)
		r.put(ref)
def main():
	mainLoop()
main()
