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
	k = ach.Channel(mds.MDS_CHAN_IK_NAME)
	r = ach.Channel(mds.MDS_CHAN_REF_FILTER_NAME)
        s = ach.Channel(mds.MDS_CHAN_STATE_NAME)
        k.flush()

	state = mds.MDS_STATE()
        ref = mds.MDS_REF()
        ikc = mds.MDS_IK()
	#take user input
	while(1):
          [status, framesize] = k.get(ikc, wait=True, last=True)
          [status, framesize] = s.get(state, wait=False, last=True)
          [status, framesize] = r.get(ref, wait=False, last=True)
          at_x = 0.0
          at_y = 0.0
          at_z = 0.0
          ar_x = 0.0
          ar_y = 0.0
          ar_z = 0.0
          arm = 'none'
          dof = 0
          jointSet = 'no'
          eff_end = np.array([0.0, 0.0, 0.0,    0.0, 0.0, 0.0])
          runIK = False
          armi = -1
          if ikc.move == mds.LEFT:
             runIK = True
             arm = 'left'
          if ikc.move == mds.RIGHT:
             runIK = True
             arm = 'right'

          if runIK:
            armi = ikc.move
            dof = ikc.arm[armi].ik_method
            at_x = ikc.arm[armi].t_x
            at_y = ikc.arm[armi].t_y
            at_z = ikc.arm[armi].t_z
            ar_x = ikc.arm[armi].r_x
            ar_y = ikc.arm[armi].r_y
            ar_z = ikc.arm[armi].r_z
            eff_end[0] = at_x
            eff_end[1] = at_y
            eff_end[2] = at_z
            eff_end[3] = ar_x
            eff_end[4] = ar_y
            eff_end[5] = ar_z
            print ikc.arm[armi].t_x
            print eff_end
            jointSet = 'set'
            ref = doIK(state, ref, eff_end, dof, jointSet, arm)
            r.put(ref)



def doIK(state, ref, eff_end,  dof, jointSet, arm):
       try:
           # Define IK
           eff_joint_space_current = [0.0, 0.0, 0.0, -0.5, 0.0, 0.0]
           if arm == 'left':
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
           elif arm == 'right':
              jntn = mds.getAddress('RSP',state)
              j0 = state.joint[jntn].ref
              jntn = mds.getAddress('RSR',state)
              j1 = state.joint[jntn].ref
              jntn = mds.getAddress('RSY',state)
              j2 = state.joint[jntn].ref
              jntn = mds.getAddress('REP',state)
              j3 = state.joint[jntn].ref
              jntn = mds.getAddress('RWY',state)
              j4 = state.joint[jntn].ref
              jntn = mds.getAddress('RWR',state)
              j5 = state.joint[jntn].ref
              eff_joint_space_current = [j0, j1, j2, j3, j4, j5]
           order = []
           err = np.array([0.01,0.01, 0.01])
           if dof == 3:
             order = ['p_x','p_y','p_z']
           elif dof == 6:
             order = ['p_x','p_y','p_z','t_x','t_y','t_z']
             err = err*10
           elif dof == 5:
             order = ['p_x','p_y','p_z','t_x','t_y']
           # Get IK
           eff_end = eff_end[:dof]
           eff_joint_space_current = ik.getIK(eff_joint_space_current, eff_end, order, arm, err)
           #eff_joint_space_current = ik.getIK3dof(eff_joint_space_current, eff_end, arm)

           # Print result
           A = ik.getFkArm(eff_joint_space_current,arm)
           eff_end_ret = ik.getPosCurrentFromOrder(A,order)
           eff_end_dif = eff_end - eff_end_ret
           print 'N-3 DOF IK Solution - for ', arm, ' arm'
           print 'des:\tx = ', round(eff_end[0],5)     , '\ty = ' , round(eff_end[1],5)     , '\tz = ', round(eff_end[2],5)
           print 'ret:\tx = ', round(eff_end_ret[0],5) , '\ty = ' , round(eff_end_ret[1],5) , '\tz = ', round(eff_end_ret[2],5)
           print 'dif:\tx = ', round(eff_end_dif[0],5) , '\ty = ' , round(eff_end_dif[1],5) , '\tz = ', round(eff_end_dif[2],5)


           if jointSet == 'set':
              if arm == 'left':
                jntn = mds.getAddress('LSP',state)
                ref.joint[jntn].ref = eff_joint_space_current[0]
                jntn = mds.getAddress('LSR',state)
                ref.joint[jntn].ref = eff_joint_space_current[1]
                jntn = mds.getAddress('LSY',state)
                ref.joint[jntn].ref = eff_joint_space_current[2]
                jntn = mds.getAddress('LEP',state)
                ref.joint[jntn].ref = eff_joint_space_current[3]
                jntn = mds.getAddress('LWY',state)
                ref.joint[jntn].ref = eff_joint_space_current[4]
                jntn = mds.getAddress('LWR',state)
                ref.joint[jntn].ref = eff_joint_space_current[5]
              elif arm == 'right':
                jntn = mds.getAddress('RSP',state)
                ref.joint[jntn].ref = eff_joint_space_current[0]
                jntn = mds.getAddress('RSR',state)
                ref.joint[jntn].ref = eff_joint_space_current[1]
                jntn = mds.getAddress('RSY',state)
                ref.joint[jntn].ref = eff_joint_space_current[2]
                jntn = mds.getAddress('REP',state)
                ref.joint[jntn].ref = eff_joint_space_current[3]
                jntn = mds.getAddress('RWY',state)
                ref.joint[jntn].ref = eff_joint_space_current[4]
                jntn = mds.getAddress('RWR',state)
                ref.joint[jntn].ref = eff_joint_space_current[5]
           return ref
       except:
         print "  Invalid input... "












def main():
	mainLoop()
main()
