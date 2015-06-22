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
#include "XitomeMCBCommandParser.c"
//#include "mds-canID.h"
//#include "mds-daemonID.h"
//#include "mds-daemon.h"
//#include "mds-jointparams.h"

// Check out which CAN API to use
//#ifdef HUBO_CONFIG_ESD
//#include "hubo/hubo-esdcan.h"
//#else
#include "mds-socketcan.h"
#include "mds-socketcan.c"
//#endif

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
void refFilterMode(mds_ref_t *r, mds_state_t *s, mds_ref_t *f, int L);
void setRefAll(mds_ref_t *r, mds_state_t *s, struct can_frame *f);
void getEncAllSlow(mds_state_t *s, struct can_frame *f);
void getCurrentAllSlow(mds_state_t *s, struct can_frame *f);
void getCurrentAllSlowMod(mds_state_t *s, struct can_frame *f, int mod);
void clearCanBuff(mds_state_t *s, struct can_frame *f);





// Timing info
#define NSEC_PER_SEC    1000000000

/* Flags */
int verbose;
int debug;

int can_skt = 0;

// ach message type
//typedef struct hubo h[1];

// ach channels
ach_channel_t chan_ref;       // mds-ref
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
    double T = (double)MDS_LOOP_PERIOD;
    int interval = (int)((double)NSEC_PER_SEC*T);
    printf("T = %1.3f sec\n",T);


    // time info
    struct timespec t, time;
    double tsec;


    // Initialize Hubo Structs
    mds_ref_t H_ref;
    mds_ref_t H_ref_filter;
    mds_state_t H_state;
    memset( &H_ref,   0, sizeof(H_ref));
    memset( &H_state, 0, sizeof(H_state));
    memset( &H_ref_filter,   0, sizeof(H_ref_filter));


    // get current time
    //clock_gettime( CLOCK_MONOTONIC,&t);
    clock_gettime( 0,&t);



/* Create CAN Frame */
    struct can_frame frame;
//   Is this really how the frame should be initialized?
    memset( &frame.data,0, sizeof(frame.data));
    //sprintf( frame.data, "1234578" );
    frame.can_dlc = strlen( frame.data );


    int startFlag = 1;

    printf("Start MDS Loop\n");
    while(1) {

        // wait until next shot
// dan        clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);

        size_t fs = 0;

        /* Get latest ACH message */
        int r = ach_get( &chan_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
        if(ACH_OK != r) {
            if(debug) {
                    fprintf(stderr, "Ref r = %s\n",ach_result_to_string(r));}
            }
        else{    assert( sizeof(H_ref) == fs); 
        }
        
        /* Set all Ref */
//        refFilterMode(&H_ref, &H_state, &H_ref_filter, MDS_REF_FILTER_LENGTH);

        /* This sets ref on the CAN bus */
//        setRefAll(&H_ref, &H_state, &frame);
//        H_state.refWait = FALSE;

        /* Get all Encoder data */
//        getEncAllSlow(&H_state, &frame); 

        /* Get all motor Current data */
//        getCurrentAllSlowMod(&H_state, &frame, 20);

        /* Get motor status/errors  (one each loop) */
//        getStatusIterate( &H_state, H_param, &frame);

        /* Get Power data */
//        getPower(&H_state, H_param, &frame);

        /* Read Console messags */
//        huboMessage(&H_ref, &H_ref_filter, H_param, &H_state, &H_cmd, &frame);

        /* Clear CAN buffer - Read any aditional data left on the buffer */
//        clearCanBuff(&H_state, &frame);
        
        char buff[80];
        memset(&buff, 0, sizeof(buff));


       // readCan(0, &frame, -1);

       /* Request all angles */
       /* Create CAN Frame */
       struct can_frame txframe;
       //   Is this really how the frame should be initialized?
       memset( &txframe.data,0, sizeof(txframe.data));
       //sprintf( frame.data, "1234578" );
       txframe.can_dlc = strlen( txframe.data );
       

       txframe.data[7] = 0x17;
       txframe.data[6] = 0x04;
       txframe.can_id = 0x11;
       txframe.can_dlc = 8;
       sendCan(can_skt,&txframe);



        for(int i = 0; i < MDS_CAN_BUFFER_CLEAR_I; i++) {
          int bytes_read = readCan( can_skt, &frame, 0);
          if (frame.data[5] == 0x00 & frame.data[4] == 0x4c){
            float ftmp;
            memcpy(&ftmp, frame.data, 4);
            printf("%f\n",ftmp);
          }
        }
/*
        frame.data[7] = RESPONSE_STATE;
        frame.data[6] =  ARGUMENT_STATE_MAX_ENCODER_FREQUENCY;
        frame.data[6] =  ARGUMENT_STATE_TRAJECTORY_PERIOD;
*/
        char* data = frame.data;
//        frame.can_id = 0x12;
        ParseResponse(buff,data,(unsigned long)floor(tsec*1000.0));
        sprintf(buff,"%s\t%x",buff,frame.can_id);
// this prints        printf("%s",buff);



        // Get current timestamp to send out with the state struct
        clock_gettime( CLOCK_MONOTONIC, &time );
        tsec = (double)time.tv_sec;
        tsec += (double)(time.tv_nsec)/1.0e9;
        H_state.time = tsec; // add time based on system time

        /* put data back in ACH channel */
        ach_put( &chan_state, &H_state, sizeof(H_state));

        t.tv_nsec+=interval;
        tsnorm(&t);
        fflush(stdout);
        fflush(stderr);
    }

}

void clearCanBuff(mds_state_t *s, struct can_frame *f) {
/*    int i = 0;
    for(i = 0; i < MDS_CAN_BUFFER_CLEAR_I; i++) {
        readCan(0, f, 0);
        decodeFrame(s, f);
        readCan(1, f, 0);
        decodeFrame(s, f);
    }
*/
}



void refFilterMode(mds_ref_t *r, mds_state_t *s, mds_ref_t *f, int L) {
    int i = 0;
    double e = 0.0;
    for(i = 0; i < MDS_JOINT_COUNT; i++) {
        int c = (int)r->joint[i].mode;
      switch (c) {
        case MDS_REF_MODE_REF: // sets reference directly
          f->joint[i].ref = r->joint[i].ref;
          break;
        case MDS_REF_MODE_COMPLIANT:  // complient mode
          f->joint[i].ref = s->joint[i].pos; 
          break;
        case MDS_REF_MODE_REF_FILTER: // slow ref to ref no encoder
          f->joint[i].ref = (f->joint[i].ref * ((double)L-1.0) + r->joint[i].ref) / ((double)L);
          break;
        case MDS_REF_MODE_ENC_FILTER: // sets filter reference encoder feedback
          //f->ref[i] = (f->ref[i] * ((double)L-1.0) + r->ref[i]) / ((double)L);
          e = f->joint[i].ref - s->joint[i].pos;
          f->joint[i].ref = (s->joint[i].pos * ((double)L-1.0) + r->joint[i].ref) / ((double)L);
          
          break;

            default:
                fprintf(stderr, "Unsupported filter mode for joint %s\n", jointNames[i]);
                break;
        }


        s->joint[i].ref = f->joint[i].ref; 
    } 
 
} 




void setRefAll(mds_ref_t *r, mds_state_t *s, struct can_frame *f) {
}

void getEncAllSlow(mds_state_t *s, struct can_frame *f){
}


void getPower(mds_state_t *s, struct can_frame *f){
}






void getBoardStatusAllSlow(mds_state_t *s, struct can_frame *f){
}

void getCurrentAllSlow(mds_state_t *s, struct can_frame *f) {
}

void getCurrentAllSlowMod(mds_state_t *s, struct can_frame *f, int mod) {
}

void mdsMessage(mds_ref_t *r, mds_ref_t *r_filt, mds_state_t *s, struct can_frame *f)
{

    size_t fs;
    int status = 0;
    int i = 0;
    while ( status == 0 | status == ACH_OK | status == ACH_MISSED_FRAME ) {
    /* 
        status = ach_get( &chan_hubo_board_cmd, c, sizeof(*c), &fs, NULL, 0 );
        if( status == ACH_STALE_FRAMES) {
            break; }
        else {
    */
        /*
            switch (c->type)
            {
                case D_GET_STATUS:
                     //hGetBoardStatus(c->joint, s, f);
                     break;
                case D_CTRL_ON:
                    //hFeedbackControllerOnOff( c->joint, r, s, f, D_ENABLE);
                    break;
                case D_CTRL_OFF:
                    //hFeedbackControllerOnOff( c->joint, r, s, f, D_DISABLE);
                    break;
                default:
                    //fprintf(stderr,"Unrecognized command type: %d\n\t",c->type);
                    break;
            }
        */
        }
}





int decodeFrame(mds_state_t *s, struct can_frame *f) {
    int fs = (int)f->can_id;
    int cmd = (int)f->data[0];
    printf("can\r\n");
    return 0; 



/*
    if (fs == H_ENC_BASE_RXDF + 0xE)
    {
        // Voltage, Current Return Message
        double voltage = doubleFromBytePair(f->data[1],f->data[0])/100.0;
        double current = doubleFromBytePair(f->data[3],f->data[2])/100.0;
        double power   = doubleFromBytePair(f->data[5],f->data[4])/10.0;
        s->power.voltage = voltage;
        s->power.current = current;
        s->power.power = power;	
    }
*/
    /* Force-Torque Readings */
/*
    else if( (fs >= H_SENSOR_FT_BASE_RXDF) && (fs <= H_SENSOR_FT_MAX_RXDF) )
    {
    }
    return 0;
*/
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
    mds_state_t H_state;
    memset( &H_ref,   0, sizeof(H_ref));
    memset( &H_state, 0, sizeof(H_state));

    // open reference
    int r = ach_open(&chan_ref, MDS_CHAN_REF_NAME, NULL);
    assert( ACH_OK == r);

    // open state
    r = ach_open(&chan_state, MDS_CHAN_STATE_NAME, NULL);
    assert( ACH_OK == r);

    // initilize control channel
//    r = ach_open(&chan_board_cmd, MDS_CHAN_BOARD_CMD_NAME, NULL);
//    assert( ACH_OK == r);

    /* Open all CAN */
//    openAllCAN(0);
    can_skt = openCAN("can0");

    /* Put on ACH Channels */
    ach_put(&chan_ref, &H_ref, sizeof(H_ref));
//    ach_put(&chan_board_cmd, &H_cmd, sizeof(H_cmd));
//    ach_put(&chan_hubo_state, &H_state, sizeof(H_state));

    // run main loop
    mainLoop();

//    hubo_daemon_close();
    
    return 0;
}

