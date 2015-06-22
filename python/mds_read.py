#!/usr/bin/env python
import mds_ach
import ach
import curses
import time
import signal
import sys
import os

# Start curses
stdscr = curses.initscr()


mds = mds_ach



s = ach.Channel(mds.MDS_CHAN_STATE_NAME)
state = mds.MDS_STATE()
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

if __name__ == '__main__':
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, exit_gracefully)
    mainLoop()




