#!/usr/bin/env python
import mds_ach
import ach

mds = mds_ach



s = ach.Channel(mds.MDS_CHAN_STATE_NAME)
state = mds.MDS_STATE()
#c.flush()
while(1):
    [status, framesize] = s.get(state, wait=True, last=True)

    print "joint = ", state.joint[0x004c].pos, "  time = ",state.time


#    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME:
#        print b.ref[0]
#    else:
#        raise ach.AchException( c.result_string(status) )
c.close()

