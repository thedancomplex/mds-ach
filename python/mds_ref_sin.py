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

#jnt = 0x0048
jnt = 0x004c
Fs = 0.1
f = 0.2
A = np.pi/6.0
#A = 0.0 #np.pi/6
tt0 = 5.0 # time to zero
def mainLoop():
  s = ach.Channel(mds.MDS_CHAN_STATE_NAME)
  r = ach.Channel(mds.MDS_CHAN_REF_NAME)
  state = mds.MDS_STATE()
  ref = mds.MDS_REF()
  [status, framesize] = r.get(ref, wait=False, last=True)
  ri = ref.joint[jnt].ref

  # bring joint to zero slowly
  

  imax = int(np.ceil(tt0/Fs))

  print "Sign Wave for the following joint:" 
  outAdd =  ''.join(format(jnt,'04x'))
  outAdd = '0x'+outAdd
  print "  Address: \t", outAdd
  print "  Freq (hz)\t", f
  print "  Amp (rad)\t", A
  for i in range(imax):
    ref.joint[jnt].ref = ri - (ri * float(i)/float(imax))
    r.put(ref)
    time.sleep(Fs)



  tm = 0.0;
  tmFlag = True
  while(1):
    [status, framesize] = s.get(state, wait=False, last=True)
    t = state.time
    if tmFlag:
      tm = float(t)
      tmFlag = False
    ref.joint[jnt].ref = A * np.sin(2.0*np.pi*f*(t-tm))
#    ref.joint[jnt].ref = A * np.sin(2*np.pi*f*t) - A * np.sin(2*np.pi*f*tm)
    r.put(ref)
    time.sleep(Fs)

mainLoop()
