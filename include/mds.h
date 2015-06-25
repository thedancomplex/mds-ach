/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
/*
Copyright (c) 2015, Daniel M. Lofaro <dan@danlofaro.com>
Copyright (c) 2012, Daniel M. Lofaro <dan@danlofaro.com>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the author nor the names of its contributors may
      be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef MDS_PRIMARY_H
#define MDS_PRIMARY_H


#ifdef __cplusplus
extern "C" {
#endif


//888888888888888888888888888888888888888888
//---------[Prerequisites for ACH]----------
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <ach.h>
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
//#include "ach.h"

//888888888888888888888888888888888888888888






//888888888888888888888888888888888888888888
//-----[Static Definitions and Offsets]-----
//888888888888888888888888888888888888888888

/* Joint Numbers/Index values */
#define         NRH             0               // Neck roll
#define		NYH		1		// neck yaw
#define		NPL		2		// neck pitch lower
#define		NPH		3		// neck pitch high
#define		LSP		4		//	Left Shoulder Pitch
#define		LSR		5		//	Left Shoulder Yaw
#define		LSY		6		//	Left Shoulder Roll
#define		LEB		7		//	Left Elbow Pitch
#define		LWY		8		// left wrist yaw
#define		LWR		9		// left wrist roll
#define		LWP		10		// left wrist pitch
#define		RSP		11		//	Right Shoulder Pitch
#define		RSR		12		//	Right Shoulder Roll
#define		RSY		13		//	Right Shoulder Yaw
#define		REB		14		//	Right Elbow Pitch
#define		RWY		15		// right wrist yaw
#define		RWR		16		// right wrist roll
#define		RWP		17		// right wrist Pitch
#define		WST		18		//	Trunk Yaw
#define		RF1		19		//	Right Finger
#define		RF2		20		//	Right Finger
#define		RF3		21		//	Right Finger
#define		RF4		22		//	Right Finger
#define		RFR		23		//	Right Finger 1 roll (thumb)
#define		LF1		24		//	Left Finger
#define		LF2		25		//	Left Finger
#define		LF3		26		//	Left Finger
#define		LF4		27		//	Left Finger
#define		LFR		28		//	Left Finger 1 roll (thumb)

#define 	MDS_CAN_CHAN_NUM	4	///> Number of CAN channels avaliable
#define         MDS_JOINT_COUNT        100      ///> The max number of joints

#define		MDS_CHAN_REF_NAME         "mds-ref"                    ///> mds ach channel
#define		MDS_CHAN_BOARD_CMD_NAME   "mds-cmd"                    ///> mds console channel for ach
#define         MDS_CHAN_PARAM_NAME       "mds-param"                  ///> mds param channel
#define		MDS_CHAN_CAN_DAEMON_NAME  "mds-can-daemon"             ///> daemon for reading CAN
#define		MDS_CHAN_STATE_NAME       "mds-state"                  ///> mds state ach channel
#define 	MDS_CHAN_REF_FILTER_NAME  "mds-ref-filter"             ///> mds reference with filter ach channel
#define 	MDS_CHAN_VIRTUAL_TO_SIM_NAME "mds-virtual-to-sim"      ///> virtual channel trigger to simulator
#define 	MDS_CHAN_VIRTUAL_FROM_SIM_NAME "mds-virtual-from-sim"  ///> virtual channel trigger from simulator
//#define		MDS_CAN_TIMEOUT_DEFAULT 0.0005		///> Default time for CAN to time out
//#define		MDS_CAN_TIMEOUT_DEFAULT 0.0002		///> Default time for CAN to time out
#define		MDS_CAN_TIMEOUT_DEFAULT 0.00018		///> Default time for CAN to time out
#define         MDS_REF_FILTER_LENGTH   40
#define         MDS_CAN_BUFFER_CLEAR_I 100
//#define         MDS_LOOP_PERIOD         0.005  ///> period for main loopin sec (0.005 = 200hz)
//#define         MDS_LOOP_PERIOD         0.010  ///> period for main loopin sec (0.010 = 100hz)
#define         MDS_LOOP_PERIOD         0.050  ///> period for main loopin sec (0.050 = 20hz)
//#define         MDS_LOOP_PERIOD         0.1  ///> period for main loopin sec (0.100 = 10hz)
//#define         MDS_LOOP_PERIOD         0.5  ///> period for main loopin sec (0.500 = 2hz)
#define         MDS_STARTUP_SEND_REF_DELAY 0.8   ///> setup delay in secons
#define         MDS_CHAR_PARAM_BUFFER_SIZE 30  // size of the buffer for the char part of the params (such as names)


#define MAX_SAFE_STACK (1024*1024) /* The maximum stack size which is
				   guaranteed safe to access without
				   faulting */

#define FALSE 0 // off static
#define TRUE  1 // on static

// array of joint name strings (total of 42)
static const char *jointNames[MDS_JOINT_COUNT] =
	{"WST", "NKY", "NK1", "NK2", // 0 1 2 3
	 "LSP", "LSR", "LSY", "LEB", "LWY", "LWR", "LWP", // 4 5 6 7 8 9 10
	 "RSP", "RSR", "RSY", "REB", "RWY", "RWR", "RWP", "N/A", // 11 12 13 14 15 16 17 18
	 "LHY", "LHR", "LHP", "LKN", "LAP", "LAR", "N/A", // 19 20 21 22 23 24 25
	 "RHY", "RHR", "RHP", "RKN", "RAP", "RAR", // 26 27 28 29 30 31
	 "RF1", "RF2", "RF3", "RF4", "RF5", // 32 33 34 35 36
	 "LF1", "LF2", "LF3", "LF4", "LF5"}; // 37 38 39 40 41

enum {
	MDS_REF_MODE_REF_FILTER    = 0, ///< Reference to reference filter
	MDS_REF_MODE_REF           = 1, ///< Direct reference control
	MDS_REF_MODE_COMPLIANT     = 2, ///< Compliant mode, sets ref to current encoder position. 
	MDS_REF_MODE_ENC_FILTER    = 3//< Reference filter
};

#define RIGHT 0
#define LEFT 1



typedef struct mds_jnt_param {
    double rom;
    double encoderresolution;
    double gearratio;
    double HomeAcceleration;
    double HomeVelocity;
    double HomeError;
    double HomeBuffer;
    double HomeTimeout;
    double MCB_KP;
    double MCB_KI;
    double MCB_KD;
    double MCB_KM;
    double safetymargin;
    double encoderconfig_tickspercount;
    double rom_margin;
    uint16_t address;
    char coordsys[MDS_CHAR_PARAM_BUFFER_SIZE];
    char HomeType[MDS_CHAR_PARAM_BUFFER_SIZE];
    char HomeTarget[MDS_CHAR_PARAM_BUFFER_SIZE];
    char name[MDS_CHAR_PARAM_BUFFER_SIZE];
}__attribute__((packed)) mds_jnt_param_t;


typedef struct mds_joint_param {
    struct mds_jnt_param joint[MDS_JOINT_COUNT];
}__attribute__((packed)) mds_joint_param_t;

typedef struct mds_joint_state {
        double ref_r;           ///< Last reference value received
        double ref;             ///< Last reference value sent
        double ref_c;           ///< Last reference valuse commanded on motorcontroller
	double pos;     	///< actual position (rad)
	double cur;     	///< actual current (amps)
	double vel;     	///< actual velocity (rad/sec)
        int16_t address;        ///< address of joint (should be same as index)
	uint8_t active; 	///< checks if the joint is active or not
	uint8_t homed;		///< checks to see if the motor is zeroed
        char name[MDS_CHAR_PARAM_BUFFER_SIZE];
}__attribute__((packed)) mds_joint_state_t;

typedef struct mds_power {
	double voltage;
	double current;
	double power;
}__attribute__((packed)) mds_power_t;

typedef struct mds_state {
	struct mds_joint_state joint[MDS_JOINT_COUNT]; ///> Joint pos, velos, and current
	mds_power_t power; // back power board
        double time;
        uint8_t refWait;  // is the robot in a waiting pattern 
}__attribute__((packed)) mds_state_t;

typedef struct mds_joint_ref {
	double ref;	///< joint reference
	int16_t mode; 	///< mode 0 = filter mode, 1 = direct reference mode
}__attribute__((packed)) mds_joint_ref_t;

typedef struct mds_ref {
	struct mds_joint_ref joint[MDS_JOINT_COUNT]; ///> Joint reference
}__attribute__((packed)) mds_ref_t;

extern int mds_debug;


#ifdef __cplusplus
}
#endif



#endif //MDS_PRIMARY_H

