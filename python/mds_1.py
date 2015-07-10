#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2015, Daniel M. Lofaro
# Copyright (c) 2013, Daniel M. Lofaro
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */



import mds_ach as ha
import ach
import sys
import time
import math
import numpy as np
from numpy.linalg import inv
from ctypes import *
import mds_ik_include as ike

#threshold = .025
threshold = .0025
## l1 = .2145   # center of body to shoulder
## l2 = .17914  # shoulder to eblow
## l3 = .18159  # elbow to wrist roll
l1 = 0.2455100   # center of body to shoulder
l2 = 0.2825750   # shoulder to eblow
l3 = 0.3127375   # elbow to wrist roll
alpha = .5
dT1 = .001		
dT2 = .001
dT3 = .001

#open ach channel for coodinates being sent from file
### k = ach.Channel(ike.CONTROLLER_REF_NAME)
coordinates = ike.CONTROLLER_REF()
coordinates.x = -0.25
coordinates.y = -0.20
coordinates.z =  0.20

# feed-forward will now be refered to as "state"
state = ha.MDS_REF()

# feed-back will now be refered to as "ref"
ref = ha.MDS_REF()

#bend elbow to start to force jacobian to solve in righ direction
#avoid singularity
ref.joint[ha.REB].ref = -math.pi/6	
ref.joint[ha.LEB].ref = -math.pi/6
	
### r.put(ref)
### time.sleep(3)

def FKE_arm(t1, t2, t3):
	x = l3*(math.sin(t3)*math.sin(t1) - math.cos(t1)*math.cos(t2)*math.cos(t3)) - l2*math.cos(t1)*math.cos(t2)
	y = -l3*(math.sin(t1)*math.cos(t2)*math.cos(t3) + math.cos(t1)*math.sin(t3)) - l2*math.sin(t1)*math.cos(t2)
	z = l3*math.sin(t2)*math.cos(t3) + l2*math.sin(t2)
	return x, y, z

def error(t1, t2, t3, pos):
	x,y,z = FKE_arm(t1+dT1,t2+dT2,t3+dT3)
	eX = x - pos[0,0]
	eY = y - pos[1,0]
	eZ = z - pos[2,0]

	dE = np.array([[eX],[eY],[eZ]])	

	return dE


def Jacobian(t1, t2, t3, x, y, z):
	dxt1 = (l3*(math.sin(t3)*math.sin(t1+dT1) - math.cos(t1+dT1)*math.cos(t2)*math.cos(t3)) - l2*math.cos(t1+dT1)*math.cos(t2)) - x
	dyt1 = (-l3*(math.sin(t1+dT1)*math.cos(t2)*math.cos(t3) + math.cos(t1+dT1)*math.sin(t3)) - l2*math.sin(t1+dT1)*math.cos(t2)) - y
	dzt1 = (l3*math.sin(t2)*math.cos(t3) + l2*math.sin(t2)) - z
			
	dxt2 = (l3*(math.sin(t3)*math.sin(t1) - math.cos(t1)*math.cos(t2+dT2)*math.cos(t3)) - l2*math.cos(t1)*math.cos(t2+dT2)) - x
	dyt2 = (-l3*(math.sin(t1)*math.cos(t2+dT2)*math.cos(t3) + math.cos(t1)*math.sin(t3)) - l2*math.sin(t1)*math.cos(t2+dT2)) - y
	dzt2 = (l3*math.sin(t2+dT2)*math.cos(t3) + l2*math.sin(t2+dT2)) - z

	dxt3 = (l3*(math.sin(t3+dT3)*math.sin(t1) - math.cos(t1)*math.cos(t2)*math.cos(t3+dT3)) - l2*math.cos(t1)*math.cos(t2)) - x
	dyt3 = (-l3*(math.sin(t1)*math.cos(t2)*math.cos(t3+dT3) + math.cos(t1)*math.sin(t3+dT3)) - l2*math.sin(t1)*math.cos(t2)) - y
	dzt3 = (l3*math.sin(t2)*math.cos(t3+dT3) + l2*math.sin(t2)) - z

        J = np.array([[dxt1/dT1, dxt2/dT2, dxt3/dT3],[dyt1/dT1, dyt2/dT2, dyt3/dT3],[dzt1/dT1, dzt2/dT2, dzt3/dT3]])

	return J


def getIK(x,y,z):
    	coordinates.x = z
	coordinates.y = y
	coordinates.z = x
	e_R = 1	
	e_L = 1
	while (e_R > threshold) or (e_L > threshold):
		#rotate/translate position vector to match IKE equations reference
		T_R = np.array([[0,1,0,0],[0,0,1,0],[1,0,0,l1],[0,0,0,1]])
		T_L = np.array([[0,1,0,0],[0,0,1,0],[1,0,0,-l1],[0,0,0,1]])
###		T_R = np.array([[0,1,0,0],[0,0,1,0],[1,0,0,.2145],[0,0,0,1]])
###		T_L = np.array([[0,1,0,0],[0,0,1,0],[1,0,0,-.2145],[0,0,0,1]])

		X_R = np.array([[coordinates.x], [coordinates.y], [coordinates.z], [1]])
		X_L = np.array([[-1*coordinates.x], [coordinates.y], [coordinates.z], [1]])

		posR = T_R.dot(X_R)
		posL = T_L.dot(X_L)

###		[statuss, framesizes] = s.get(state, wait=False, last =True)
		#Get current joint angles
###		RSP = state.joint[ha.RSP].pos	#t1
###		RSR = state.joint[ha.RSR].pos	#t2
###		REB = state.joint[ha.REB].pos 	#t3
###		LSP = state.joint[ha.LSP].pos
###		LSR = state.joint[ha.LSR].pos
###		LEB = state.joint[ha.LEB].pos
                state = ref
		RSP = state.joint[ha.RSP].ref	#t1
		RSR = state.joint[ha.RSR].ref	#t2
		REB = state.joint[ha.REB].ref 	#t3
		LSP = state.joint[ha.LSP].ref
		LSR = state.joint[ha.LSR].ref
		LEB = state.joint[ha.LEB].ref

		#remove when rest of code updated to new refs
		t1 = RSP
		t2 = RSR
		t3 = REB
	
		#compute current end effector positions
		x_R,y_R,z_R = FKE_arm(RSP,RSR,REB)
		x_L,y_L,z_L = FKE_arm(LSP,LSR,LEB)

		#compute and update error from current position
		e_R = math.sqrt((posR[0,0] - x_R)**2 + (posR[1,0] - y_R)**2 + (posR[2,0] - z_R)**2)
		e_L = math.sqrt((posL[0,0] - x_L)**2 + (posL[1,0] - y_L)**2 + (posL[2,0] - z_L)**2)
   		print 'e_R = ', e_R, '  e_L = ', e_L	
		#check if one end effector has arrived while the other hasnt, stop that end effector while other continues to move
		if (e_R <= threshold):
			stopR = 0
		else:
			stopR = 1

		if (e_L <= threshold):
			stopL = 0
		else:
			stopL = 1

		#compute dE values from dTheta and desired position
		dE_R = error(RSP, RSR, REB, posR)
		dE_L = error(LSP, LSR, LEB, posL)

		#compute jacobian
		J_R = Jacobian(RSP, RSR, REB, x_R, y_R, z_R)
		J_L = Jacobian(LSP, LSR, LEB, x_L, y_L, z_L)

		#computing pseudo inverse
		JplusR = (inv((J_R.T).dot(J_R))).dot(J_R.T)
		JplusL = (inv((J_L.T).dot(J_L))).dot(J_L.T)

		#computing dTheta
		dThetaR = JplusR.dot(dE_R)
		dThetaL = JplusL.dot(dE_L)
		
		#send to robot
###		ref.ref[ha.RSP] = state.joint[ha.RSP].pos - dThetaR[0,0]*alpha*stopR
###		ref.ref[ha.RSR] = state.joint[ha.RSR].pos - dThetaR[1,0]*alpha*stopR
###		ref.ref[ha.REB] = state.joint[ha.REB].pos - dThetaR[2,0]*alpha*stopR
###		ref.ref[ha.LSP] = state.joint[ha.LSP].pos - dThetaL[0,0]*alpha*stopL
###		ref.ref[ha.LSR] = state.joint[ha.LSR].pos - dThetaL[1,0]*alpha*stopL
###		ref.ref[ha.LEB] = state.joint[ha.LEB].pos - dThetaL[2,0]*alpha*stopL
		ref.joint[ha.RSP].ref = state.joint[ha.RSP].ref - dThetaR[0,0]*alpha*stopR
		ref.joint[ha.RSR].ref = state.joint[ha.RSR].ref - dThetaR[1,0]*alpha*stopR
		ref.joint[ha.REB].ref = state.joint[ha.REB].ref - dThetaR[2,0]*alpha*stopR
		ref.joint[ha.LSP].ref = state.joint[ha.LSP].ref - dThetaL[0,0]*alpha*stopL
		ref.joint[ha.LSR].ref = state.joint[ha.LSR].ref - dThetaL[1,0]*alpha*stopL
		ref.joint[ha.LEB].ref = state.joint[ha.LEB].ref - dThetaL[2,0]*alpha*stopL
###		r.put(ref)
		time.sleep(.01)

	#wait 3 seconds once arrived at position
	print 'Arrived at target position'
###	[statuss, framesizes] = s.get(state, wait=False, last =True)
        return [LSP, LSR, LEB]
	x_R,y_R,z_R = FKE_arm(RSP,RSR,REB)
	x_L,y_L,z_L = FKE_arm(LSP,LSR,LEB)
	print 'Right arm current position (local frame)' , x_R , y_R , z_R
	print 'Left arm current position (local frame)' , x_L , y_L , z_L
	time.sleep(3)

while True:
	#get next set of coordinates. Readcoords.py will fill the ach channel with 3 iterations worth of targets and end. 3 second wait added at and of while loop
###	[statuss, framesizes] = k.get(coordinates, wait=False, last=False)


    jnt = getIK( 0.20, -0.10, -l2*0.9)
    print jnt




