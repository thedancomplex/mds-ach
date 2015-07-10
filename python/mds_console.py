#!/usr/bin/env python
import mds_ach as mds
import ach
#import curses
import time
import signal
import sys
import os
from optparse import OptionParser
import math
import readline
import numpy as np
import mds_ik as ik
#c.flush()


mdsIntro = ">> mds-ach-console$ "

radT = 0.1
T = 0.1

def mainLoop():
  # Start curses
 s = ach.Channel(mds.MDS_CHAN_STATE_NAME)
 rf = ach.Channel(mds.MDS_CHAN_REF_FILTER_NAME)
 r = ach.Channel(mds.MDS_CHAN_REF_NAME)
 con = ach.Channel(mds.MDS_CHAN_CON2IK_NAME)
 state = mds.MDS_STATE()
 ref = mds.MDS_REF()
 ref_filt = mds.MDS_REF()
 command = mds.MDS_CON2IK()
 doloop = True
 while(doloop):
    # Block until input is received
  
    var = raw_input(mdsIntro)
    c = var.split(" ")
    # Get latest states
    [status, framesize] = s.get(state, wait=False, last=True)
    [status, framesize] = r.get(ref, wait=False, last=True)
    [status, framesize] = rf.get(ref_filt, wait=False, last=True)

    # get command
    cmd = c[0]
    if cmd == 'get':
       try:
         if c[1] == 'fk':
           arm = c[2]
           eff_joint_space_current = [0.0, 0.0, 0.0, -0.5, 0.0, 0.0]
           if arm == 'left':
              jntn = getAddress('LSP',state)
              j0 = state.joint[jntn].ref
              jntn = getAddress('LSR',state)
              j1 = state.joint[jntn].ref
              jntn = getAddress('LSY',state)
              j2 = state.joint[jntn].ref
              jntn = getAddress('LEP',state)
              j3 = state.joint[jntn].ref
              jntn = getAddress('LWY',state)
              j4 = state.joint[jntn].ref
              jntn = getAddress('LWR',state)
              j5 = state.joint[jntn].ref
              eff_joint_space_current = [j0, j1, j2, j3, j4, j5]
           elif arm == 'right':
              jntn = getAddress('RSP',state)
              j0 = state.joint[jntn].ref
              jntn = getAddress('RSR',state)
              j1 = state.joint[jntn].ref
              jntn = getAddress('RSY',state)
              j2 = state.joint[jntn].ref
              jntn = getAddress('REP',state)
              j3 = state.joint[jntn].ref
              jntn = getAddress('RWY',state)
              j4 = state.joint[jntn].ref
              jntn = getAddress('RWR',state)
              j5 = state.joint[jntn].ref
              eff_joint_space_current = [j0, j1, j2, j3, j4, j5]
           A = ik.getFkArm(eff_joint_space_current,arm)
           order = ['p_x','p_y','p_z','t_x','t_y','t_z']
           eff_end_ret = ik.getPosCurrentFromOrder(A,order)
           print '6DOF FK - for ', arm, ' arm'
           print 'pos:\tx = ', round(eff_end_ret[0],5) , '\ty = ' , round(eff_end_ret[1],5) , '\tz = ', round(eff_end_ret[2],5)
           print 'rot:\tx = ', round(eff_end_ret[3],5) , '\ty = ' , round(eff_end_ret[4],5) , '\tz = ', round(eff_end_ret[5],5)
         
         else:
           jnt = c[1]
           jntn = getAddress(jnt,state)
           pos = state.joint[jntn].pos
           pos_r = state.joint[jntn].ref
           jntName = (mds.ubytes2str(state.joint[jntn].name)).replace('\0', '')
           print jntName, ' State = ', str(format(pos,'.5f')), ' rad'
           print jntName, ' Ref   = ', str(format(pos_r,'.5f')), ' rad'
       except:
         print "  Invalid input... "
    elif cmd == 'exit':
         s.close()
         r.close()
         quit()
         doloop = False
    elif cmd == 'goto':
       try:
        if c[1] == 'filter':
         jnt = c[2]
         jntn = getAddress(jnt,state)
         pos = float(c[3])
         ref_filt.joint[jntn].ref = pos
         rf.put(ref_filt)
        else:
         jnt = c[1]
         jntn = getAddress(jnt,state)
         pos = float(c[2])
         pos0 = state.joint[jntn].ref
         doLoop = True
         while (doLoop):
           pd = pos - pos0
           psign = np.sign(pd)
           if np.abs(pd) > radT:
              pos0 = pos0 + psign*radT
           else:
              pos0 = pos
              doLoop = False
         
           ref.joint[jntn].ref = pos0
           r.put(ref)
           time.sleep(T)
           sys.stdout.write(".")
           sys.stdout.flush()
         print "done"
       except:
         print "  Invalid input... "
    elif cmd == 'gotopos':
      try:
	#command.ik_method = int(c[1])
	#command.num_dof = int(c[2])
	#0 is left arm 1 is right arm
	command.arm = int(c[1])
	command.x = float(c[2])
	command.y = float(c[3])
	command.z = float(c[4])
	con.put(command)
      except:
	print "  Invalid input... "
    elif cmd == 'zeroall':      
       for i in range(mds.MDS_JOINT_COUNT):
           ref.joint[i].ref = 0.0
       r.put(ref)  

    elif cmd == 'ik':
      try:
        arm = c[1]
        dof = 3
        jointSet = c[len(c)-1]
        eff_end     = np.array([float(c[2]) , float(c[3]), float(c[4])])
        ref = doIK(state, ref, eff_end, dof, jointSet, arm)
        if jointSet == 'set':
          r.put(ref)
      except:
         print "  Invalid input... "
    elif cmd == 'ik5':
      try:
        arm = c[1]
        dof = 5
        jointSet = c[len(c)-1]
        eff_end     = np.array([float(c[2]) , float(c[3]), float(c[4]), float(c[5]), float(c[5])])
        ref = doIK(state, ref, eff_end, dof, jointSet, arm)
        if jointSet == 'set':
          r.put(ref)
      except:
         print "  Invalid input... "
    
 #s.close()


def doIK(state, ref, eff_end,  dof, jointSet, arm):
       try:
           # Define IK
           eff_joint_space_current = [0.0, 0.0, 0.0, -0.5, 0.0, 0.0]
           if arm == 'left':
              jntn = getAddress('LSP',state)
              j0 = state.joint[jntn].ref
              jntn = getAddress('LSR',state)
              j1 = state.joint[jntn].ref
              jntn = getAddress('LSY',state)
              j2 = state.joint[jntn].ref
              jntn = getAddress('LEP',state)
              j3 = state.joint[jntn].ref
              jntn = getAddress('LWY',state)
              j4 = state.joint[jntn].ref
              jntn = getAddress('LWR',state)
              j5 = state.joint[jntn].ref
              eff_joint_space_current = [j0, j1, j2, j3, j4, j5]
           elif arm == 'right':
              jntn = getAddress('RSP',state)
              j0 = state.joint[jntn].ref
              jntn = getAddress('RSR',state)
              j1 = state.joint[jntn].ref
              jntn = getAddress('RSY',state)
              j2 = state.joint[jntn].ref
              jntn = getAddress('REP',state)
              j3 = state.joint[jntn].ref
              jntn = getAddress('RWY',state)
              j4 = state.joint[jntn].ref
              jntn = getAddress('RWR',state)
              j5 = state.joint[jntn].ref
              eff_joint_space_current = [j0, j1, j2, j3, j4, j5]
           order = []
           if dof == 3:
             order = ['p_x','p_y','p_z']
           elif dof == 6:
             order = ['p_x','p_y','p_z','t_x','t_y','t_z']
           elif dof == 5:
             order = ['p_x','p_y','p_z','t_x','t_y']
           err = np.array([0.01,0.01, 0.01])
           # Get IK
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
                jntn = getAddress('LSP',state)
                ref.joint[jntn].ref = eff_joint_space_current[0]
                jntn = getAddress('LSR',state)
                ref.joint[jntn].ref = eff_joint_space_current[1]
                jntn = getAddress('LSY',state)
                ref.joint[jntn].ref = eff_joint_space_current[2]
                jntn = getAddress('LEP',state)
                ref.joint[jntn].ref = eff_joint_space_current[3]
                jntn = getAddress('LWY',state)
                ref.joint[jntn].ref = eff_joint_space_current[4]
                jntn = getAddress('LWR',state)
                ref.joint[jntn].ref = eff_joint_space_current[5]
              elif arm == 'right':
                jntn = getAddress('RSP',state)
                ref.joint[jntn].ref = eff_joint_space_current[0]
                jntn = getAddress('RSR',state)
                ref.joint[jntn].ref = eff_joint_space_current[1]
                jntn = getAddress('RSY',state)
                ref.joint[jntn].ref = eff_joint_space_current[2]
                jntn = getAddress('REP',state)
                ref.joint[jntn].ref = eff_joint_space_current[3]
                jntn = getAddress('RWY',state)
                ref.joint[jntn].ref = eff_joint_space_current[4]
                jntn = getAddress('RWR',state)
                ref.joint[jntn].ref = eff_joint_space_current[5]
           return ref   
       except:
         print "  Invalid input... "




def getAddress(name, state):
    a = getNameAndAddress(state)
    a1 = getNameShort(a,state)
    return getAddressOnly(name,a1)

def getAddressOnly(name,a):
  for j in range(0,mds.MDS_JOINT_COUNT):
     jnt = a[j]
     if ((jnt[0] == name) | (jnt[2] == name)):
        return jnt[1]
  return -1


def getNameAndAddress(state):
#  a = np.chararray((mds.MDS_JOINT_COUNT,3),itemsize=mds.MDS_CHAR_PARAM_BUFFER_SIZE)
  a = []
  for j in range(0,mds.MDS_JOINT_COUNT):
     jnt = state.joint[j]
     jntName = (mds.ubytes2str(jnt.name)).replace('\0', '')
     a.append([jntName, jnt.address])
  return a

def getNameShort(a,state):
#  b = np.chararray((mds.MDS_JOINT_COUNT,2),itemsize=mds.MDS_CHAR_PARAM_BUFFER_SIZE)
  b = [['HeadRoll'             , 'NKR'  ],
       ['LeftEyePan'           , 'LYY'  ],
       ['EyePitch'             , 'EYP'  ],
       ['JawPitch'             , 'JAP'  ],
       ['LeftUpperEyelidPitch' , 'LUYP' ],
       ['LeftEyebrowPitch'     , 'LYBP' ],
       ['RightEyelidRoll'      , 'RYLR' ],
       ['RightEyebrowPitch'    , 'RYBP' ],
       ['HeadPitch'            , 'NKP1' ],
       ['NeckPitch'            , 'NKP2' ],
       ['HeadPan'              , 'NKY'  ],
       ['JawRoll'              , 'JAR'  ],
       ['RightUpperEyelidPitch', 'RUYLP'],
       ['RightLowerEyelidPitch', 'RLYLP'],
       ['RightEyebrowRoll'     , 'RYBR' ],
       ['RightEyePan'          , 'RYY'  ],
       ['LeftEyebrowRoll'      , 'LYBR' ],
       ['LeftEyelidRoll'       , 'LYLR' ],
       ['LeftLowerEyelidPitch' , 'LLYLP'],
       ['JawJut'               , 'JAJ'  ],
       ['RightElbowFlex'       , 'REP'  ],
       ['RightUpperArmRoll'    , 'RSY'  ],
       ['RightMiddleFlex'      , 'RF3'  ],
       ['RightShoulderAbd'     , 'RSR'  ],
       ['LeftElbowFlex'        , 'LEP'  ],
       ['LeftUpperArmRoll'     , 'LSY'  ],
       ['LeftMiddleFlex'       , 'LF3'  ],
       ['LeftShoulderAbd'      , 'LSR'  ],
       ['RightWristFlex'       , 'RWR'  ],
       ['RightIndexFlex'       , 'RF2'  ],
       ['RightWristRoll'       , 'RWY'  ],
       ['RightThumbRoll'       , 'RF0'  ],
       ['LeftWristFlex'        , 'LWR'  ],
       ['LeftIndexFlex'        , 'LF2'  ], 
       ['LeftWristRoll'        , 'LWY'  ],
       ['LeftThumbRoll'        , 'LF0'  ],
       ['RightThumbFlex'       , 'RF1'  ],
       ['RightPinkieFlex'      , 'RF4'  ],
       ['RightShoulderExt'     , 'RSP'  ],
       ['TorsoPan'             , 'WST'  ],
       ['LeftThumbFlex'        , 'LF1'  ],
       ['LeftPinkieFlex'       , 'LF4'  ],
       ['LeftShoulderExt'      , 'LSP'  ]]
 
  a_out = []
  for j in range(0,mds.MDS_JOINT_COUNT):
    aa = a[j]
    a0 = aa[0]
    a1 = aa[1]
    a2 = '-------' 
    for i in range(0,len(b)):
     jnt = state.joint[j]
     jntName = (mds.ubytes2str(jnt.name)).replace('\0', '')
     c = b[i]
     if jntName == c[0]:
         a2 = c[1]
    a_out.append([a0, a1, a2])

  return a_out


def exit_gracefully(signum, frame):
    # restore the original signal handler as otherwise evil things will happen
    # in raw_input when CTRL+C is pressed, and our signal handler is not re-entrant
    signal.signal(signal.SIGINT, original_sigint)

    try:
        print "Exiting ... "
        os.system('reset')
        sys.exit(1)

    except KeyboardInterrupt:
        print("Ok ok, quitting")
        sys.exit(1)

    # restore the exit gracefully handler here    
    signal.signal(signal.SIGINT, exit_gracefully)
def ubytes2str(ubarray):
     return (''.join(map(chr,ubarray))).rstrip()

def printParams(jnt):
    p = ach.Channel(mds.MDS_CHAN_PARAM_NAME)
    param = mds.MDS_JOINT_PARAM()
    [status, framesize] = p.get(param, wait=False, last=True)
    j = param.joint[jnt]
    print 'Name           \t:\t', ubytes2str(j.name)
    print 'Address        \t:\t', j.address
    print 'Rom            \t:\t', j.rom
    print 'Enc Rez        \t:\t', j.encoderresolution
    print 'Gear Ratio     \t:\t', j.gearratio
    print 'Home Acc       \t:\t', j.HomeAcceleration
    print 'Home Velos     \t:\t', j.HomeVelocity
    print 'Home Buff      \t:\t', j.HomeBuffer
    print 'home Timeout   \t:\t', j.HomeTimeout
    print 'MBC KP         \t:\t', j.MCB_KP
    print 'MBC KI         \t:\t', j.MCB_KI
    print 'MBC KD         \t:\t', j.MCB_KD
    print 'MBC KM         \t:\t', j.MCB_KM
    print 'Safty Margin   \t:\t', j.safetymargin
    print 'Enc Tic/Cnt    \t:\t', j.encoderconfig_tickspercount
    print 'Rom Margin     \t:\t', j.rom_margin
    print 'Coord Sys      \t:\t', ubytes2str(j.coordsys)
    print 'Home Type      \t:\t', ubytes2str(j.HomeType)
    print 'Home Target    \t:\t', ubytes2str(j.HomeTarget)
    
def printHelp():
         print '\n----------------------------------------------'
         print '--------- MDS-ACH Read -----------------------'
         print '----- Contact: Daniel M. Lofaro --------------'
         print '--------- dan@danlofaro.com ------------------'
         print '----------------------------------------------'
         print '-- python mds_ach.py read'
         print '--       Return: Prints auto updated states for all joints'
         print '-- python mds_ach.py param #'
         print '      Return: Prints paramaters for joint #'
         print '-- python mds_ach.py help'
         print '-- python mds_ach.py -h'
         print '-- python mds_ach.py h'
         print '-- python mds_ach.py'
         print '--       Return: Prints this help menue'
         print '----------------------------------------------'
         print '----------------------------------------------'
         print '----------------------------------------------\n\r'

if __name__ == '__main__':
    jnt = -1

    # Exit gracefully
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, exit_gracefully)

    mainLoop()
