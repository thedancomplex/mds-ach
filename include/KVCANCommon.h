/*! Common CAN Functions
* Basic functions used for accessing and initializing the CAN bus
* \brief Common can fuction declaration
* \file CANCommon.h
* \ingroup CommonHelperFunctions
*/

#ifndef _KVCANCOMMON_H_
#define _KVCANCOMMON_H_

#include <canlib.h>
#include "CANCommon.h"

//! KVaser CANlib CANCommon implementation
/* Implementation of the CCANCommon object for Kvaser Canlib hardware
* \class CCANCommon
*/
class CKVCANCommon: public CCANCommon {
    public:

	CKVCANCommon();
	~CKVCANCommon();

        int read(long int *target, unsigned char *d, 
			unsigned long int *timestamp);
        int write(long int target, unsigned char *d);
        int Initialize(int id);
	int CloseBus(void);
	int goodmsg(void);
    private:
	void Check(const char* id, canStatus stat);
    
	//! handle to open can channel
	int iCANHandle;
    
	//! flags used by various canlib functions
	unsigned int uiFlags;

};

#endif
