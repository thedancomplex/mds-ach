#!/usr/bin/env python
import mds_ach as mds
import ach
import time
import signal
import sys
import re
import os
from optparse import OptionParser
import math


def mainLoop():
  k = ach.Channel(mds.MDS_CHAN_IK_NAME)
  ikc = mds.MDS_IK()
  ii = 0
  while(1):
    c = [0.3, 0.2, 0.0]
    if ii == 0:
      c = [0.3, 0.2, 0.0]
      ii = 1
    elif ii == 1:
      c = [0.3, 0.2, 0.3]
      ii = ii+1
    elif ii == 2:
      c = [0.4, 0.0, 0.3]
      ii = ii+1
    elif ii == 3:
      c = [0.4, 0.0, -0.3]
      ii = ii+1
    elif ii == 4:
      c = [0.4, 0.2, -0.3]
      ii = 1
    if True:
        arm = 'left'
        armi = -1
        dof = 3
        eff = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for i in range(0,dof):
          eff[i] = c[i]

        if arm == 'left':
             armi = mds.LEFT
        if arm == 'right':
             armi = mds.RIGHT

        if armi >= 0:
           ikc.move = armi
           ikc.arm[armi].ik_method = dof
           ikc.arm[armi].t_x = eff[0]
           ikc.arm[armi].t_y = eff[1]
           ikc.arm[armi].t_z = eff[2]
           ikc.arm[armi].r_x = eff[3]
           ikc.arm[armi].r_y = eff[4]
           ikc.arm[armi].r_z = eff[5]
           print c 
           k.put(ikc)



    time.sleep(5.0)

#    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME:
#        print b.ref[0]
#    else:
#        raise ach.AchException( c.result_string(status) )



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
   mainLoop()
