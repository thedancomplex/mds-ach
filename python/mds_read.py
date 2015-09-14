#!/usr/bin/env python
import mds_ach as mds
import ach
import curses
import time
import signal
import sys
import re
import os
from optparse import OptionParser
import math

#c.flush()

rMin = 0x0040
rMax = 0x004f

timey = 1
timex = 2
heady = 2
posy = heady 

x  = 2
xd = 8
shortnamex = x
x = x + xd
namex = x
xd = 22
x = x + xd

enabledx = x
xd = 9
x = x + xd
addressx = x
xd = 11
x = x + xd
posrefx = x
xd = 16
x = x + xd
posstatex = x
xd = 8
x = x + xd

def mainLoop(enabled):
  # Start curses
  stdscr = curses.initscr()
  s = ach.Channel(mds.MDS_CHAN_STATE_NAME)
  state = mds.MDS_STATE()
  curses.resizeterm(101, 100)
  while(1):
    [status, framesize] = s.get(state, wait=False, last=True)

#    print "joint = ", state.joint[0x004c].pos, "  time = ",state.time
    stdscr.addstr(heady,namex, "Joint", curses.A_BOLD)
    stdscr.addstr(heady,addressx, "address", curses.A_BOLD)
    stdscr.addstr(heady,enabledx, "Enabled", curses.A_BOLD)
    stdscr.addstr(heady,posrefx, "Commanded Pos", curses.A_BOLD)
    stdscr.addstr(heady,posstatex, "Actual Pos", curses.A_BOLD)
    i = 0
    y = 0
    stdscr.addstr(timey,timex, str(state.time))
#    for j in range(rMin,rMax):
    for j in range(39,mds.MDS_JOINT_COUNT):
     jnt = state.joint[j]
     if ((jnt.enabled == 1) | (enabled == 0) ):
       jntName = (mds.ubytes2str(jnt.name)).replace('\0', '')
       jntNameShort = (mds.ubytes2str(jnt.nameshort)).replace('\0', '')
  #     if (False == math.isnan(jnt.pos)):
       if ('' != jntName.strip()):
        y = i+heady+1
        # Name
  #      print jnt.name
        stdscr.addstr(y,shortnamex, jntNameShort)
        stdscr.addstr(y,namex, jntName)
 
        # Enabled
        stdscr.addstr(y,enabledx,str(jnt.enabled))

        # Addresses
        outAdd =  ''.join(format(jnt.address,'04x'))
        outAdd = '0x'+outAdd
        stdscr.addstr(y,addressx, outAdd)

        # cmd pos
        stdscr.addstr(y,posrefx, str(format(jnt.ref,'.5f')))
        #stdscr.addstr(y,posrefx, str(jnt.ref))

        # actuial pos
#        stdscr.addstr(y,posstatex, str(format(jnt.pos,'.5f')))
        stdscr.addstr(y,posstatex, str(format(jnt.ref_c_send,'.5f')))
        #stdscr.addstr(y,posstatex, str(jnt.pos))

        i += 1

    y = i+heady+2
    stdscr.addstr(y, shortnamex, "-----------------[ Collisions ]-----------------------")
    y = y+1
    colx = 9
    stdscr.addstr(y, namex+colx*0, "Hand")
    stdscr.addstr(y, namex+colx*1, "Wrist")
    stdscr.addstr(y, namex+colx*2, "Forearm")
    stdscr.addstr(y, namex+colx*3, "Elbow")
    stdscr.addstr(y, namex+colx*4, "Shoulder")

    y = y+1
    colx2 = 2
    stdscr.addstr(y, shortnamex, "Right:")
    stdscr.addstr(y, namex+colx2+colx*0, str(state.collide.joint[mds.COLLISION_R_HAND].isCollide))
    stdscr.addstr(y, namex+colx2+colx*1, str(state.collide.joint[mds.COLLISION_R_WRIST].isCollide))
    stdscr.addstr(y, namex+colx2+colx*2, str(state.collide.joint[mds.COLLISION_R_FOREARM].isCollide))
    stdscr.addstr(y, namex+colx2+colx*3, str(state.collide.joint[mds.COLLISION_R_ELBOW].isCollide))
    stdscr.addstr(y, namex+colx2+colx*4, str(state.collide.joint[mds.COLLISION_R_SHOULDER].isCollide))

    y = y+1
    stdscr.addstr(y, shortnamex, "Left:")
    stdscr.addstr(y, namex+colx2+colx*0, str(state.collide.joint[mds.COLLISION_L_HAND].isCollide))
    stdscr.addstr(y, namex+colx2+colx*1, str(state.collide.joint[mds.COLLISION_L_WRIST].isCollide))
    stdscr.addstr(y, namex+colx2+colx*2, str(state.collide.joint[mds.COLLISION_L_FOREARM].isCollide))
    stdscr.addstr(y, namex+colx2+colx*3, str(state.collide.joint[mds.COLLISION_L_ELBOW].isCollide))
    stdscr.addstr(y, namex+colx2+colx*4, str(state.collide.joint[mds.COLLISION_L_SHOULDER].isCollide))

    y = y+1
    stdscr.addstr(y, shortnamex, ("Collide: %r  " % bool(state.collide.isCollide)))

    stdscr.refresh()
    time.sleep(0.05)

#    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME:
#        print b.ref[0]
#    else:
#        raise ach.AchException( c.result_string(status) )
  s .close()



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
         print '--       Return: Prints auto updated states for enabled joints'
         print '-- python mds_ach.py all'
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
    enabled = 0


    parser = OptionParser()
    (options, args) = parser.parse_args()
    for arg in args:
      if arg == 'param':
         jnt = -2
      elif jnt == -2:
         jnt = printParams(int(arg))
         exit()      
      elif arg == 'read':
         original_sigint = signal.getsignal(signal.SIGINT)
         signal.signal(signal.SIGINT, exit_gracefully)
         enabled = 1
         mainLoop(enabled)
      elif arg == 'all':
         original_sigint = signal.getsignal(signal.SIGINT)
         signal.signal(signal.SIGINT, exit_gracefully)
         mainLoop(enabled)
      elif ((arg == 'help') | (arg == '-h') | (arg == 'h')):
         printHelp()

    if (len(args) ==0):
      printHelp()
