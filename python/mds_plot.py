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
from matplotlib.ticker import MultipleLocator, FormatStrFormatter


Fs = 0.05


plt.axis([0, 1000, 0, 1])
plt.ion()
plt.show()
plt.xlabel("time (sec)")
plt.ylabel("angle (rad)")
plt.title("Joint Angle")

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
bs = 154
st = []
st_r = []
st_c = []
t  = []
re = []
for i in range(bs):
   st.append(i) 
   st_r.append(i) 
   st_c.append(i) 
   t.append(i)
   re.append(i)

jnt = 0x004c


def mainLoop():
  s = ach.Channel(mds.MDS_CHAN_STATE_NAME)
  r = ach.Channel(mds.MDS_CHAN_REF_NAME)
  state = mds.MDS_STATE()
  ref = mds.MDS_REF()
  j = 0
  [status, framesize] = s.get(state, wait=False, last=True)
  for i in range(bs):
    st[i] = 0.0
    t[i] = state.time
    re[i] = 0.0
    
  t[bs-1] = t[0] + Fs
  while(1):
    [status, framesize] = s.get(state, wait=False, last=True)
    [status, framesize] = r.get(ref, wait=False, last=True)
    st[j] = state.joint[jnt].pos
    st_r[j] = state.joint[jnt].ref_r
    st_c[j] = state.joint[jnt].ref_c
    re[j] = ref.joint[jnt].ref
    t[j] = state.time
    plt.clf()
    plt.axis([np.amin(t), np.amax(t), -3, 3])
#    plt.scatter(t, y)
#    plt.plot(t, y,'.r-')
#    plt.plot(t, re,'.b-')
#    plt.plot(t, st,'.g-')
    pa = plt.plot(t, re   ,'.b' , label='reference')
    pb = plt.plot(t, st_r ,'.k' , label='cammanded via CAN')
    pc = plt.plot(t, st_c ,'.r' , label='commanded by MCB')
    pd = plt.plot(t, st   ,'.g' , label='actual pos')
    plt.legend(['reference','cammanded via CAN','commanded by MCB','actual pos'])
    plt.grid(b=True, which='major', color='b', linestyle='-')
#    plt.grid(b=True, which='minor', color='r', linestyle='--')
    plt.draw()

#    pylab.plot(t,y)
    #pylab.show()



    j = j+1
    if j >= bs:
      j = 0
    

    time.sleep(Fs)

mainLoop()
