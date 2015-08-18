#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2015, Daniel M. Lofaro
# Copyright (c) 2013, Daniel M. Lofaro
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */
import mds_ach as mds
import ach
import time
import sys
import os
import math
import numpy as np
import mds_ach as mds

def mainLoop():
  # Start curses
 s = ach.Channel(mds.MDS_CHAN_STATE_NAME)
 r = ach.Channel(mds.MDS_CHAN_REF_FILTER_NAME)
 state = mds.MDS_STATE()
 ref = mds.MDS_REF()
 doloop = True
 ii = 0
 while(doloop):
    # Get latest states
    [status, framesize] = s.get(state, wait=False, last=True)
    [status, framesize] = r.get(ref, wait=False, last=True)
    
    # Get address of LSP
    jntn = mds.getAddress('LSP',state)
    print 'Address = ', jntn

    # Set LSP reference to -0.123 using the filter controller (smooth)
    ref.joint[jntn].ref = -0.123

    # Command motors to desired references (post to ach channel)
    r.put(ref)
    
    if ii == 1:
      doloop = False
    ii = 1

    # Wait 0.5 sec
    time.sleep(0.5)


 # Close channels
 s.close()
 r.close()

if __name__ == '__main__':
    mainLoop()
