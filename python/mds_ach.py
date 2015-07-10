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

# from ctypes import *
from ctypes import Structure,c_uint16,c_double,c_ubyte,c_uint32,c_int16
# import ach
# import sys



MDS_JOINT_COUNT                  = 100
MDS_CHAN_REF_NAME                = 'mds-ref'
MDS_CHAN_REF_FILTER_NAME	 = 'mds-ref-filter'
MDS_CHAN_CON2IK_NAME	 	 = 'mds-con2ik'
MDS_CHAN_BOARD_CMD_NAME          = 'mds-board-cmd'
MDS_CHAN_PARAM_NAME              = "mds-param"
MDS_CHAN_STATE_NAME              = 'mds-state'
MDS_LOOP_PERIOD                  = 0.005
MDS_CHAR_PARAM_BUFFER_SIZE       = 30
MAX_EFF				 = 6

H = 0 # Neck roll
NYH = 1 # neck yaw
NPL = 2 # neck pitch lower
NPH = 3 # neck pitch high
LSP = 4 #      Left Shoulder Pitch
LSR = 5 #      Left Shoulder Yaw
LSY = 6 #      Left Shoulder Roll
LEB = 7 #      Left Elbow Pitch
LWY = 8 # left wrist yaw
LWR = 9 # left wrist roll
LWP = 10 # left wrist pitch
RSP = 11 #      Right Shoulder Pitch
RSR = 12 #      Right Shoulder Roll
RSY = 13 #      Right Shoulder Yaw
REB = 14 #      Right Elbow Pitch
RWY = 15 # right wrist yaw
RWR = 16 # right wrist roll
RWP = 17 # right wrist Pitch
WST = 18 #      Trunk Yaw
RF1 = 19 #      Right Finger
RF2 = 20 #      Right Finger
RF3 = 21 #      Right Finger
RF4 = 22 #      Right Finger
RFR = 23 #      Right Finger 1 roll (thumb)
LF1 = 24 #      Left Finger
LF2 = 25 #      Left Finger
LF3 = 26 #      Left Finger
LF4 = 27 #      Left Finger
LFR = 28 #      Left Finger 1 roll (thumb)

# Modes
MDS_REF_MODE_REF_FILTER    = 0 # Reference to reference filter
MDS_REF_MODE_REF           = 1 # Direct reference control
MDS_REF_MODE_COMPLIANT     = 2 # Compliant mode, sets ref to current encoder position.
MDS_REF_MODE_ENC_FILTER    = 3 # Reference filter

RIGTH = 0
LEFT = 1
TRUE = 1
FALSE = 0

def getAddress(name, state):
    a = getNameAndAddress(state)
    a1 = getNameShort(a,state)
    return getAddressOnly(name,a1)

def getAddressOnly(name,a):
  for j in range(0,MDS_JOINT_COUNT):
     jnt = a[j]
     if ((jnt[0] == name) | (jnt[2] == name)):
        return jnt[1]
  return -1

def getNameAndAddress(state):
#  a = np.chararray((mds.MDS_JOINT_COUNT,3),itemsize=mds.MDS_CHAR_PARAM_BUFFER_SIZE)
  a = []
  for j in range(0,MDS_JOINT_COUNT):
     jnt = state.joint[j]
     jntName = (ubytes2str(jnt.name)).replace('\0', '')
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
       ['HeadPan'              , 'NKY'  ],
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
       ['RightUpperArmRoll'    , 'RSY'  ],
       ['RightMiddleFlex'      , 'RF3'  ],
       ['RightShoulderAbd'     , 'RSR'  ],
       ['LeftElbowFlex'        , 'LEP'  ],
       ['LeftUpperArmRoll'     , 'LSY'  ],
       ['LeftMiddleFlex'       , 'LF3'  ],
       ['LeftShoulderAbd'      , 'LSR'  ],
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
  for j in range(0,MDS_JOINT_COUNT):
    aa = a[j]
    a0 = aa[0]
    a1 = aa[1]
    a2 = '-------' 
    for i in range(0,len(b)):
     jnt = state.joint[j]
     jntName = (ubytes2str(jnt.name)).replace('\0', '')
     c = b[i]
     if jntName == c[0]:
         a2 = c[1]
    a_out.append([a0, a1, a2])

  return a_out



class MDS_JNT_PARAM(Structure):
    _pack_ = 1
    _fields_ = [("rom"                          , c_double),
                ("encoderresolution"            , c_double),
                ("gearratio"                    , c_double),
                ("HomeAcceleration"             , c_double),
                ("HomeVelocity"                 , c_double),
                ("HomeError"                    , c_double),
                ("HomeBuffer"                   , c_double),
                ("HomeTimeout"                  , c_double),
                ("MCB_KP"                       , c_double),
                ("MCB_KI"                       , c_double),
                ("MCB_KD"                       , c_double),
                ("MCB_KM"                       , c_double),
                ("safetymargin"                 , c_double),
                ("encoderconfig_tickspercount"  , c_double),
                ("rom_margin"                   , c_double),
                ("offset"                       , c_double),
                ("direction"                    , c_double),
                ("enabled"                      , c_int16),
                ("address"                      , c_int16),
                ("coordsys"                     , c_ubyte*MDS_CHAR_PARAM_BUFFER_SIZE),
                ("HomeType"                     , c_ubyte*MDS_CHAR_PARAM_BUFFER_SIZE),
                ("HomeTarget"                   , c_ubyte*MDS_CHAR_PARAM_BUFFER_SIZE),
                ("name"                         , c_ubyte*MDS_CHAR_PARAM_BUFFER_SIZE),
                ("nameshort"                    , c_ubyte*MDS_CHAR_PARAM_BUFFER_SIZE)]

class MDS_JOINT_PARAM(Structure):
    _pack_ = 1
    _fields_ = [("joint",    MDS_JNT_PARAM*MDS_JOINT_COUNT)]

class MDS_JOINT_STATE(Structure):
    _pack_ = 1
    _fields_ = [("ref_r"     , c_double),
                ("ref"       , c_double),
                ("ref_c"     , c_double),
                ("pos"       , c_double),
                ("cur"       , c_double),
                ("vel"       , c_double),
                ("offset"    , c_double),
                ("direction" , c_double),
                ("enabled"   , c_int16),
                ("address"   , c_int16),
                ("active"    , c_ubyte),
                ("homed"     , c_ubyte),
                ("name"      , c_ubyte*MDS_CHAR_PARAM_BUFFER_SIZE),
                ("nameshort" , c_ubyte*MDS_CHAR_PARAM_BUFFER_SIZE)]

class MDS_POWER(Structure):
	_pack_ = 1
	_fields_ = [("voltage", c_double),
                    ("current", c_double),
                    ("power", c_double)]

class MDS_CON2IK(Structure):
    _pack_ = 1
    _fields_ = [
                ("ik_method"  , c_int16),
		("num_dof"  , c_int16),
		("arm"  , c_int16),
                ("x"   , c_double),
                ("y"   , c_double),
                ("z"   , c_double)]

class MDS_STATE(Structure):
    _pack_ = 1
    _fields_ = [
                ("joint"  , MDS_JOINT_STATE*MDS_JOINT_COUNT),
		("power"  , MDS_POWER),
                ("time"   , c_double),
                ("refWait", c_int16)]

class MDS_JOINT_REF(Structure):
    _pack_ = 1
    _fields_ = [("ref", c_double),
                ("mode", c_int16)]


class MDS_REF(Structure):
    _pack_ = 1
    _fields_ = [("joint",    MDS_JOINT_REF*MDS_JOINT_COUNT)]


def p2f(x):
    return float(x.strip('%'))/100

def str2ubytes(str_bytes_ini):
     str_bytes = str_bytes_ini +' '*(mds.MDS_CHAR_PARAM_BUFFER_SIZE - len(str_bytes_ini))
     return (ctypes.c_ubyte * mds.MDS_CHAR_PARAM_BUFFER_SIZE)(*bytearray(str_bytes))

def ubytes2str(ubarray):
     return (''.join(map(chr,ubarray))).rstrip()

