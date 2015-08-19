import numpy as np
from struct import pack
from struct import unpack
import mds_ach as mds
import pylab
import ach
import time
import signal
import mds_ik as ik
import sys
import os
import math
from ctypes import *
import copy
from numpy.linalg import inv
import mds_ik_include as ike
from mds_ach import *

#define global variables
# feed-forward will now be refered to as "state"
state = mds.MDS_REF()
#initial feed-back
initial_ref = mds.MDS_REF()
initial_state = mds.MDS_STATE()
# feed-back will now be refered to as "state"
ref = mds.MDS_REF()
state = mds.MDS_STATE()
# Command line for console
ikc = mds.MDS_IK()
#define arm constant

def mainLoop():
  # Open Ach Channels
  k = ach.Channel(mds.MDS_CHAN_IK_NAME)
  r = ach.Channel(mds.MDS_CHAN_REF_FILTER_NAME)
  s = ach.Channel(mds.MDS_CHAN_STATE_NAME)
  k.flush()

  # Make Structs
  state = mds.MDS_STATE()
  ref = mds.MDS_REF()
  ikc = mds.MDS_IK()

  # Get the latest on the channels
  [status, framesize] = s.get(state, wait=False, last=True)
  [status, framesize] = r.get(ref, wait=False, last=True)

  # Desired position 
  eff_end = np.array([0.3, 0.2, 0.0, 0.0, 0.0, 0.0])

  # Define Arm
  arm = 'left'

  # Get current joint space pose of arm
  jntn = mds.getAddress('LSP',state)
  j0 = state.joint[jntn].ref
  jntn = mds.getAddress('LSR',state)
  j1 = state.joint[jntn].ref
  jntn = mds.getAddress('LSY',state)
  j2 = state.joint[jntn].ref
  jntn = mds.getAddress('LEP',state)
  j3 = state.joint[jntn].ref
  jntn = mds.getAddress('LWY',state)
  j4 = state.joint[jntn].ref
  jntn = mds.getAddress('LWR',state)
  j5 = state.joint[jntn].ref
  eff_joint_space_current = [j0, j1, j2, j3, j4, j5]

  # set the dof and the order (dof = 3)
  dof = 3
  order = ['p_x','p_y','p_z']

  # set the allowable error
  err = np.array([0.01,0.01, 0.01])

  # Set solving step number max
  stepNum = 1000

  # Deisred position for only the dof we want
  eff_end = eff_end[:dof]

  # Solve IK
  jnt_return = ik.getIK(eff_joint_space_current, eff_end, order, arm, err, stepNum)

  # exits if no solution within setNum itirations
  if (jnt_return[1] == -1):
    print 'no IK found within ', stepNum, ' steps'
    quit()
  # else set the new joint space
  else:
    eff_joint_space_current = jnt_return[0]

  # get FK of arm
  A = ik.getFkArm(eff_joint_space_current,arm)
  eff_end_ret = ik.getPosCurrentFromOrder(A,order)

  # find different in desired vs actuial pos
  eff_end_dif = eff_end - eff_end_ret

  # Print
  print 'N-3 DOF IK Solution - for ', arm, ' arm'
  print 'des:\tx = ', round(eff_end[0],5)     , '\ty = ' , round(eff_end[1],5)     , '\tz = ', round(eff_end[2],5)
  print 'ret:\tx = ', round(eff_end_ret[0],5) , '\ty = ' , round(eff_end_ret[1],5) , '\tz = ', round(eff_end_ret[2],5)
  print 'dif:\tx = ', round(eff_end_dif[0],5) , '\ty = ' , round(eff_end_dif[1],5) , '\tz = ', round(eff_end_dif[2],5)


if __name__ == '__main__':
   mainLoop()
                
