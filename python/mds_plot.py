#-------------------------------------------------------------------------------
#  Get a screen catpure from DPO4000 series scope and save it to a file
# python        2.7         (http://www.python.org/)
# pyvisa        1.4         (http://pyvisa.sourceforge.net/)
# numpy         1.6.2       (http://numpy.scipy.org/)
# MatPlotLib    1.0.1       (http://matplotlib.sourceforge.net/)
#-------------------------------------------------------------------------------
import numpy as np
from struct import unpack
import pylab
import mds_ach as mds
import ach
import time
import signal
import sys
import os
import math
import matplotlib.pyplot as plt
plt.axis([0, 1000, 0, 1])
plt.ion()
plt.show()
plt.xlabel("time (sec)")
plt.ylabel("angle (rad)")
plt.title("Joint Angle")
plt.grid(b=True, which='major', color='b', linestyle='-')
plt.grid(b=True, which='minor', color='r', linestyle='--')

ymult = float(1.0)
yzero = float(0.0)
yoff = float(0.0)
xincr = float(1)

##ADC_wave = np.array([1, 2, 3, 4, 5, 6, 7, 6, 5, 4, 3, 2, 1])
#ADC_wave = np.array(unpack('%sB' % len(ADC_wave),ADC_wave))

##Volts = (ADC_wave - yoff) * ymult  + yzero
##Time = np.arange(0, xincr * len(Volts), xincr)

##pylab.plot(Time, Volts)

##pylab.show()

# buffer size
bs = 100
y = []
t = []
for i in range(bs):
   y.append(i) 
   t.append(i)

jnt = 0x004c


def mainLoop():
  s = ach.Channel(mds.MDS_CHAN_STATE_NAME)
  state = mds.MDS_STATE()
  j = 0
  [status, framesize] = s.get(state, wait=False, last=True)
  for i in range(bs):
    y[i] = 0.0
    t[i] = state.time
  while(1):
    [status, framesize] = s.get(state, wait=False, last=True)
    y[j] = state.joint[jnt].pos
    t[j] = state.time
    plt.axis([np.amin(t), np.amax(t), -3, 3])
#    plt.scatter(t, y)
#    plt.plot(t, y,'.r-')
    plt.plot(t, y,'.g-')
    plt.draw()

#    pylab.plot(t,y)
    #pylab.show()



    j = j+1
    if j >= bs:
      j = 0
    

    time.sleep(0.1)

mainLoop()
