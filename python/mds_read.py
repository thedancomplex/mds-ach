#!/usr/bin/env python
import mds_ach as mds
import ach
import curses
import time
import signal
import sys
import os
from optparse import OptionParser

#c.flush()

rMin = 0x0040
rMax = 0x004f

timey = 1
timex = 2
heady = 2
addressx = 2
posy = heady
posrefx = 10
posstatex = 30

def mainLoop():
  # Start curses
  stdscr = curses.initscr()
  s = ach.Channel(mds.MDS_CHAN_STATE_NAME)
  state = mds.MDS_STATE()
  while(1):
    [status, framesize] = s.get(state, wait=False, last=True)

#    print "joint = ", state.joint[0x004c].pos, "  time = ",state.time
    stdscr.addstr(heady,addressx, "Joint", curses.A_BOLD)
    stdscr.addstr(heady,posrefx, "Commanded Pos", curses.A_BOLD)
    stdscr.addstr(heady,posstatex, "Actual Pos", curses.A_BOLD)
    i = 0
    stdscr.addstr(timey,timex, str(state.time))
    for j in range(rMin,rMax):
      y = i+heady+1
      # Addresses
      stdscr.addstr(y,addressx, str(j))

      # cmd pos
      stdscr.addstr(y,posrefx, str(state.joint[j].ref))

      # actuial pos
      stdscr.addstr(y,posstatex, str(state.joint[j].pos))

      i += 1
    stdscr.refresh()
    time.sleep(0.1)

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
         mainLoop()
      elif ((arg == 'help') | (arg == '-h') | (arg == 'h')):
         printHelp()

    if (len(args) ==0):
      printHelp()
