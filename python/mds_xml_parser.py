#Copyright (c) 2015, Daniel M. Lofaro
#All rights reserved.

#Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the author nor the names of its contributors may
#      be used to endorse or promote products derived from this software
#      without specific prior written permission.

#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
#INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
#PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
#LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
#OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#*/

import xml.etree.ElementTree as et
import mds_ach as mds
import ach
import ctypes

def p2f(x):
    return float(x.strip('%'))/100

def str2ubytes(str_bytes_ini):
     str_bytes = str_bytes_ini +' '*(mds.MDS_CHAR_PARAM_BUFFER_SIZE - len(str_bytes_ini))
     return (ctypes.c_ubyte * mds.MDS_CHAR_PARAM_BUFFER_SIZE)(*bytearray(str_bytes))

def ubytes2str(ubarray):
     return (''.join(map(chr,ubarray))).rstrip()

def doParseXmlMds(doc,p):
  tree = et.parse(doc)
  root = tree.getroot()
  for joint in root.findall('muscle'):
     str_bytes_ini = joint.attrib['name']
     name0i = str2ubytes(str_bytes_ini)
     name0 = (mds.ubytes2str(name0i)).replace('\0', '')
     for i in range(mds.MDS_JOINT_COUNT):
      j = p.joint[i]
      name = (mds.ubytes2str(j.name)).replace('\0', '')

      if (name == name0):
        for param in joint.findall('parameter'):
          s = param.attrib['name']
          v = param.attrib['value']
          if( v != ""):
            if( s == 'offset'):
              j.offset = float(v)
            if( s == 'direction'):
              j.direction = float(v)
            if( s == 'enabled'):
              j.enabled = int(v)
            if( s == 'mdsachname'):
              j.nameshort = str2ubytes(v)
        #put it back on the struct
        p.joint[i] = j  
  return p


def doParseXML(doc,p):
  tree = et.parse(doc)
  root = tree.getroot()
  for joint in root.findall('muscle'):
     address = joint.attrib['address']
     jnt = int(address,0)
     j = p.joint[jnt]
     str_bytes_ini = joint.attrib['name']
  #   print 'name1 = ', str_bytes_ini, '\t\t len = ', len(str_bytes_ini)
     j.name = str2ubytes(str_bytes_ini)
     j.address = jnt
     for param in joint.findall('parameter'):
      s = param.attrib['name']
      v = param.attrib['value']
      if( v != ""):
       if( s == 'rom'):
         j.rom = float(v)
       if( s == 'encoderresolution'):
         j.encoderresolution = float(v)
       if( s == 'gearratio'):
         j.gearratio = float(v)
       if( s == 'HomeAcceleration'):
         j.HomeAcceleration = float(v)
       if( s == 'HomeVelocity'):
         j.HomeVelocity = float(v)
       if( s == 'HomeError'):
         j.HomeError = float(v)
       if( s == 'HomeBuffer'):
         j.HomeBuffer = float(v)
       if( s == 'HomeTimeout'):
         j.HomeTimeout = float(v)
       if( s == 'MCB.P'):
         j.MCB_KP = float(v)
       if( s == 'MCB.I'):
         j.MCB_KI = float(v)
       if( s == 'MCB.D'):
         j.MCB_KD = float(v)
       if( s == 'MCB.M'):
         j.MCB_KM = float(v)
       if( s == 'safetymargin'):
         j.safetymargin = p2f(v)
       if( s == 'encoderconfig.tickspercount'):
         j.encoderconfig_tickspercount = float(v)
       if( s == 'rom.margin'):
         j.rom_margin = p2f(v)
       if( s == 'coordsys'):
         j.coordsys = str2ubytes(v)
       if( s == 'HomeType'):
         j.HomeType = str2ubytes(v)
       if( s == 'HomeTarget'):
         j.HomeTarget = str2ubytes(v)
       
     #put it back on the struct
     p.joint[jnt] = j  
  return p


if __name__ == '__main__':
    param = mds.MDS_JOINT_PARAM()
    param = doParseXML('/etc/mds-ach/configs/anatomy.xml',param)
    param = doParseXmlMds('/etc/mds-ach/configs/mdsach.xml',param)
    p = ach.Channel(mds.MDS_CHAN_PARAM_NAME)
    p.put(param)





