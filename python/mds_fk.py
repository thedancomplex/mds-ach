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
import copy


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


# rotation matrix definition (abotu x, y, or z)
def_x = 1
def_y = 2
def_z = 3


def getRotMatrix(t, d):
    if d == def_x:
        return np.matrix([[1.0, 0.0, 0.0], [ 0.0, np.cos(t), -np.sin(t)], [ 0.0, np.sin(t), np.cos(t)]])
    elif d == def_y:
        return np.matrix([[np.cos(t), 0.0, np.sin(t)], [ 0.0, 1.0, 0.0], [ -np.sin(t), 0.0, np.cos(t)]])
    elif d == def_z:
        return np.matrix([[np.cos(t), -np.sin(t), 0.0], [ np.sin(t), np.cos(t), 0.0], [ 0.0, 0.0, 1.0]])
    else:
        return np.matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

def getTransMatrix(x,y,z):
    return np.matrix([[x], [y], [z]])
     
def getTfMatrix(R,T):    
    Ap = np.hstack([R,T])
    return np.vstack([Ap,[0.0, 0.0, 0.0, 1.0]])

def getA(a):    
#    a = [theta  , Tx  , Ty        , Tz  , def_xyz]
    R = getRotMatrix(a[0], a[4])
    T = getTransMatrix(a[1],a[2],a[3])
    A  = getTfMatrix(R,T)
    return A

def getFkArm(a, arm):
  m = 1.0 
  if arm == 'right':
    m = -1.0
  elif arm == 'left':
    m = 1.0
  else:
    return -1

  #       theta        , Tx       , Ty          , Tz         , def_xyz
  LSP = [ a[0]         , 0.0      , m*0.2455100 , 0.0        , def_y ] 
  LSR = [ a[1]         , 0.0      , 0.0         , 0.0        , def_x ]
  LSY = [ a[2]         , 0.0      , 0.0         , -0.2825750 , def_z ] 
  LEB = [ a[3]         , 0.0      , 0.0         , 0.0        , def_y ]
  LWY = [ a[4]         , 0.0      , 0.0         , -0.3127375 , def_z ] 
  LWR = [ a[5]         , 0.0      , 0.0         , 0.0        , def_x ]
  LEN = [ 0.0          , 0.0      , 0.0         , -0.0635    , def_x ] 
  # note rolls seem to be oppisit (-)

  A1 = getA(LSP)
  A2 = getA(LSR)
  A3 = getA(LSY)
  A4 = getA(LEB)
  A5 = getA(LWY)
  A6 = getA(LWR)
  A7 = getA(LEN)

  A = np.dot(A1,A2)
  A = np.dot(A, A2)
  A = np.dot(A, A3)
  A = np.dot(A, A4)
  A = np.dot(A, A5)
  A = np.dot(A, A6)
  A = np.dot(A, A7)

  return A


def getJacobian(d0,arm):
  # gets numerical jacobian of 'arm'
  # d0 is the current position of the arm

  d = 0.001

  # Jacobian for xyz and 6 dof
  xyz = 3
  dof = 6
  J = np.zeros((xyz,dof))
  # J[ row, col ] 
  d0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  for i in range(xyz):
     for j in range(dof):
        d0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        d1 = copy.deepcopy(d0)
        d1[j] = d0[j] + d
        A0 = getFkArm(d0,arm)
        A1 = getFkArm(d1,arm)
        J[i,j] = (A1[i,3] - A0[i,3])/d
  return J   



#print A
def doSolve():
 a = [0.0, 0.0, 0.0, -0.5, 0.0, 0.0]
 A = getFkArm(a,'left')
 de0 = np.array([ A[0,3], A[1,3], A[2,3]])
# ef  = np.array([0.2, 0.2, -0.1])
 ef  = np.array([-0.14583 , 0.74283, 0.13834])

 de = 0.0000001 # change in goal in meters
 err = 0.01
 dd = 1.0
 v = ef - de0 # vector to end point
 dd = np.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
 while (dd > err):
  # linear interpolation to find next point
  de1 = de0 + v/dd * de

  J = getJacobian(de1,'left')
  Ji = np.linalg.pinv(J)   # psudo inverse
  
  dt = np.dot(Ji,de1)

  a = a + dt
  A = getFkArm(a,'left')
  
  de0 = np.array([ A[0,3], A[1,3], A[2,3]])

  v = ef - de0 # vector to end point
  dd = np.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
  print 'dd = ', dd
  print 'e0 = ', de0
  print 'ef = ', ef
  if dd < err:
     print 'dd = ', dd
     print 'pos = ', de1
     print 'ang = ', a


#J = getJacobian('left')
#print J
a = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
print 'ang = ', a
A = getFkArm(a,'left')
print ' x = ', round(A[0,3],5) , '  y = ' , round(A[1,3],5) , '  z = ', round(A[2,3],5)
A = getFkArm(a,'right')
print ' x = ', round(A[0,3],5) , '  y = ' , round(A[1,3],5) , '  z = ', round(A[2,3],5)
###jnt = getIK( 0.20, -0.10, -l2*0.9)
###print jnt

doSolve()


