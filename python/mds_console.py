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
#c.flush()


mdsIntro = ">> mds-ach-console$ "

radT = 0.1
T = 0.1

def mainLoop():
  # Start curses
  s = ach.Channel(mds.MDS_CHAN_STATE_NAME)
  r = ach.Channel(mds.MDS_CHAN_REF_NAME)
  state = mds.MDS_STATE()
  ref = mds.MDS_REF()
  while(1):
    # Block until input is received
    var = raw_input(mdsIntro)
    c = var.split(" ")
    # Get latest states
    [status, framesize] = s.get(state, wait=False, last=True)
    [status, framesize] = r.get(ref, wait=False, last=True)

    # get command
    cmd = c[0]
    

    if cmd == 'get':
       try:
         jnt = c[1]
         jntn = getAddress(jnt,state)
         pos = state.joint[jntn].pos
         jntName = (mds.ubytes2str(state.joint[jntn].name)).replace('\0', '')
         print jntName, ' = ', str(format(pos,'.5f')), ' rad'
       except:
         print "  Invalid input... "
    elif cmd == 'goto':
       try:
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
            




#    print (" you entered " + str(len(cmds)))


  




  s .close()


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
       ['HeadPan'              , 'NPY'  ],
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
       ['RightUpperArmRoll'    , 'RSR'  ],
       ['RightMiddleFlex'      , 'RF3'  ],
       ['RightShoulderAbd'     , 'RSY'  ],
       ['LeftElbowFlex'        , 'LEP'  ],
       ['LeftUpperArmRoll'     , 'LSR'  ],
       ['LeftMiddleFlex'       , 'LF3'  ],
       ['LeftShoulderAbd'      , 'LSY'  ],
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
