/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
/*
Copyright (c) 2015, Daniel M. Lofaro
Copyright (c) 2012, Daniel M. Lofaro
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


#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

// for timer
#include <time.h>
#include <sched.h>
#include <sys/io.h>
#include <unistd.h>

// for RT
#include <stdlib.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/mman.h>

// for mds
#include "mds.h"
//#include "XitomeMCBCommandParser.c"
//#include "mds-canID.h"
//#include "mds-daemonID.h"
//#include "mds-daemon.h"
//#include "mds-jointparams.h"


// for ach
/*
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include "ach.h"
*/

/* At time of writing, these constants are not defined in the headers */
#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

/* ... */

/* Somewhere in your app */



//static inline void tsnorm(struct timespec *ts);
void mainLoop();

// Timing info
#define NSEC_PER_SEC   1000000000
#define MDS_LOOP_PERIOD_FILTER 0.025
#define TIME 1*(1/MDS_LOOP_PERIOD_FILTER)
//Mode of operation (0-time based movement 1-velocity based movement)
#define OPERATION 1

//Desired Time of Arrival
#define DTOA	1
#define DESIRED_SPEED 6

/* Flags */
int verbose;
int debug;

int can_skt = 0;

// ach message type
//typedef struct hubo h[1];

// ach channels
ach_channel_t chan_ref;       // mds-ref
ach_channel_t chan_filter_ref;       // mds-ref
ach_channel_t chan_param;     // mds-param
ach_channel_t chan_board_cmd; // mds-ach-console
ach_channel_t chan_state;     // mds-ach-state
ach_channel_t chan_to_sim;    // mds-ach-to-sim
ach_channel_t chan_from_sim;  // mds-ach-from-sim


static inline void tsnorm(struct timespec *ts){
//void tsnorm(struct timespec *ts){
//	clock_nanosleep( NSEC_PER_SEC, TIMER_ABSTIME, ts, NULL);
	// calculates the next shot
	while (ts->tv_nsec >= NSEC_PER_SEC) {
		//usleep(100);	// sleep for 100us (1us = 1/1,000,000 sec)
		ts->tv_nsec -= NSEC_PER_SEC;
		ts->tv_sec++;
	}
}



void mainLoop() {
    double T = (double)MDS_LOOP_PERIOD_FILTER;
    int interval = (int)((double)NSEC_PER_SEC*T);
    printf("T = %1.3f sec\n",T);
    
    //velocity control information
    double position_difference=0;
    double position_increment[MDS_JOINT_COUNT];
    double lastref[MDS_JOINT_COUNT];

    // time info
    struct timespec t, time;
    double tsec;
   

    // Initialize Hubo Structs
    mds_ref_t H_ref;
    mds_ref_t H_ref_filter;
    mds_state_t H_state;
    mds_joint_param_t H_param;
    memset( &H_ref,   0, sizeof(H_ref));
    memset( &H_state, 0, sizeof(H_state));
    memset( &H_ref_filter,   0, sizeof(H_ref_filter));
    memset( &H_param,   0, sizeof(H_param));

    // get current time
    //clock_gettime( CLOCK_MONOTONIC,&t);
    clock_gettime( 0,&t);

    printf("Start MDS Loop\n");
    while(1) {
	//NOTE: DTOA MUST BE GREATER THAN UPDATE RATE or an unstable system will 		  develop
	size_t fs = 0;
	//Get the difference in distances if joint command changes 
        /* Get latest ACH message */
	int r = ach_get( &chan_filter_ref, &H_ref_filter, sizeof(H_ref_filter), &fs, NULL, ACH_O_LAST);
        if(ACH_OK != r) {
            if(debug) {
                    fprintf(stderr, "State r = %s\n",ach_result_to_string(r));}
            }
           else
	    {
		assert( sizeof(H_ref) == fs);
	    }
	/*r = ach_get( &chan_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST);
        if(ACH_OK != r) {
            if(debug) {
                    fprintf(stderr, "State r = %s\n",ach_result_to_string(r));}
            }
           else
	    {
		assert( sizeof(H_state) == fs);
	    }
	*/ 
	for(int i=0;i<MDS_JOINT_COUNT;i++)
    	{
		
		//TIME BASED FILTER
		/*if(OPERATION == 0 && TIME<=1)
		{
			if(H_ref_filter.joint[i].ref != lastref[i])
			{
			position_difference = (H_ref_filter.joint[i].ref) - (H_ref.joint[i].ref);
			//position_difference = (H_ref_filter.joint[i].ref) - (H_state.joint[i].pos);
			position_increment[i] = (position_difference/(TIME));
			}
		}
		*/
		//VELOCITY BASED FILTER
		 if(DESIRED_SPEED<=(1/MDS_LOOP_PERIOD_FILTER))
		{
			if(H_ref_filter.joint[i].ref != lastref[i])
			{
			
				position_difference = (H_ref_filter.joint[i].ref) - (H_ref.joint[i].ref);
				double increment_value = (DESIRED_SPEED*(MDS_LOOP_PERIOD_FILTER));
				if(position_difference>=0)
					position_increment[i] = increment_value;
				else
					position_increment[i] = -1*increment_value;

			}
		}
		else 
		{
			printf("\nConfiguration Error: Operation code are not correct or desired time/operational speed is not possible with current update rate!!!");
		}
		
    	}	        
        /* put data back in ACH channel */
 	for(int i=0;i<MDS_JOINT_COUNT;i++)
    	{
		if(H_ref.joint[i].ref<=H_ref_filter.joint[i].ref-DESIRED_SPEED*MDS_LOOP_PERIOD_FILTER || H_ref.joint[i].ref>= H_ref_filter.joint[i].ref+DESIRED_SPEED*MDS_LOOP_PERIOD_FILTER)
		{
			H_ref.joint[i].ref = (H_ref.joint[i].ref) + position_increment[i];
			lastref[i] = H_ref_filter.joint[i].ref;
		}
		else
		{
			H_ref.joint[i].ref = (H_ref_filter.joint[i].ref);
		}
		//printf("\n %f",H_ref.joint[i].ref);
	}
	printf("\n %f",H_ref.joint[77].ref);//H_ref.joint[5].ref,H_ref.joint[6].ref,H_ref.joint[7].ref,H_ref.joint[8].ref,H_ref.joint[9].ref);
        ach_put( &chan_ref, &H_ref, sizeof(H_ref));
        t.tv_nsec+=interval;
        tsnorm(&t);
        fflush(stdout);
        fflush(stderr);
	// wait until next shot
        clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);
    }

}

int main(int argc, char **argv) {

    // Parse user input
    debug = 0;
    int i = 1;
    while(argc > i)
    {
        if(strcmp(argv[i], "-d") == 0) {
            debug = 1;
        }
        if(strcmp(argv[i], "-v") == 0){
           /* put args here */
        }
        i++;
    }
    // Daemonize
//    hubo_daemonize();
    

    // Initialize Hubo Structs
    mds_ref_t H_ref;
    mds_ref_t H_ref_filter;
    mds_state_t H_state;
    memset( &H_ref,   0, sizeof(H_ref));
    memset( &H_state, 0, sizeof(H_state));

    // open reference
    int r = ach_open(&chan_ref, MDS_CHAN_REF_NAME, NULL);
    printf("ACH RESULT: %d",r);
    assert( ACH_OK == r);

    // open state
    /*r = ach_open(&chan_state, MDS_CHAN_STATE_NAME, NULL);
    assert( ACH_OK == r);
    */
    // open param channel
    //r = ach_open(&chan_param, MDS_CHAN_PARAM_NAME, NULL);
    //assert( ACH_OK == r);

    //open smoothed reference
    r = ach_open(&chan_filter_ref,MDS_CHAN_REF_FILTER_NAME, NULL);
    printf("ACH RESULT: %d",r);
    assert( ACH_OK == r);
    



    // initilize control channel
    //r = ach_open(&chan_board_cmd, MDS_CHAN_BOARD_CMD_NAME, NULL);
    //    assert( ACH_OK == r);


    // run main loop
    mainLoop();

//    hubo_daemon_close();
    
    return 0;
}

