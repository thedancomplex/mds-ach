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
      c = [0.3, 0.45, 0.25]
      ii = ii+1
    elif ii == 2:
      c = [0.4, -0.05, 0.25]
      ii = ii+1
    elif ii == 3:
      c = [0.4, -0.05, -0.25]
      ii = ii+1
    elif ii == 4:
      c = [0.4, 0.45, -0.25]
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

if __name__ == '__main__':
   mainLoop()
