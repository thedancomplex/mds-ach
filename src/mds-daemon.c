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
void mainLoop(int vcan);
void refFilterMode(mds_ref_t *r, mds_state_t *s, mds_ref_t *f, int L);
void getEncAllSlow(mds_state_t *s, struct can_frame *f);
void getCurrentAllSlow(mds_state_t *s, struct can_frame *f);
void getCurrentAllSlowMod(mds_state_t *s, struct can_frame *f, int mod);
void clearCanBuff(int skt, mds_state_t *s, mds_joint_param_t *p, char *buff, struct can_frame *f);
int decodeFrame(mds_state_t *s, mds_joint_param_t *p, char *buff, struct can_frame *f);
int getPos(mds_state_t *s, mds_joint_param_t *p, char *buff, char *delm);
int getCmdPos(mds_state_t *s, mds_joint_param_t *p, char *buff, char *delm);
void requestState(int skt);
float deg2enc(double deg, int address, mds_joint_param_t *p);
float rad2enc(double radss, int address, mds_joint_param_t *p);
double rad2deg(double rad);
void f_setAcc(double radss, int address, mds_joint_param_t *p, int skt, struct can_frame *f);
void f_setRef(double ref, int address, mds_joint_param_t *p, int skt, struct can_frame *f);
void f_setRefTrajectoryModeDefault(int address, mds_joint_param_t *p, int skt, struct can_frame *f);
void f_setRefTrajectoryMode(int mode, int address, mds_joint_param_t *p, int skt, struct can_frame *f);
void f_setRefTrajectoryPeriodDefault(int address, mds_joint_param_t *p, int skt, struct can_frame *f);
void f_setRefTrajectoryPeriod(int period, int address, mds_joint_param_t *p, int skt, struct can_frame *f);
void setRef(double rad, double radss, int address, mds_joint_param_t *p, int skt, struct can_frame *f);
void setRefAll(mds_ref_t *r, mds_state_t *s, mds_ref_t *fi, mds_joint_param_t *p, int L, int skt, struct can_frame *f);
void setMdsParamState(mds_state_t *s, mds_joint_param_t *p);


// Timing info
#define NSEC_PER_SEC    1000000000

/* Flags */
int verbose;
int debug;

int can_skt = 0;

/* histor of encoder sending for CAN jitter fix */
double deg_hist[MDS_JOINT_COUNT];


// ach message type
//typedef struct hubo h[1];

// ach channels
ach_channel_t chan_ref;       // mds-ref
ach_channel_t chan_param;     // mds-param
ach_channel_t chan_board_cmd; // mds-ach-console
ach_channel_t chan_state;     // mds-ach-state
ach_channel_t chan_to_sim;    // mds-ach-to-sim
ach_channel_t chan_from_sim;  // mds-ach-from-sim
ach_channel_t chan_collide;  // collide channel


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


void setNameToState(mds_state_t *s, mds_joint_param_t *p){
    for( int i = 0; i < MDS_JOINT_COUNT; i++) {
//        memset(&s->joint[i].name, p->joint[i].name, sizeof(s->joint[i].name));
        memcpy(s->joint[i].name,      p->joint[i].name,      strlen(p->joint[i].name));        
        memcpy(s->joint[i].nameshort, p->joint[i].nameshort, strlen(p->joint[i].nameshort));        
//s->joint[i].name = p->joint[i].name;
    }
}

void setAddressToState(mds_state_t *s, mds_joint_param_t *p){
    for( int i = 0; i < MDS_JOINT_COUNT; i++) {
        s->joint[i].address = p->joint[i].address;
    }
}

void setMdsParamState(mds_state_t *s, mds_joint_param_t *p){
    for( int i = 0; i < MDS_JOINT_COUNT; i++) {
        s->joint[i].offset    = p->joint[i].offset;
        s->joint[i].direction = p->joint[i].direction;
        s->joint[i].enabled   = p->joint[i].enabled;
    }
}

void mainLoop(int vcan) {
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
    mds_joint_param_t H_param;
    mds_collide_t H_collide;
    memset( &H_ref,   0, sizeof(H_ref));
    memset( &H_state, 0, sizeof(H_state));
    memset( &H_ref_filter,   0, sizeof(H_ref_filter));
    memset( &H_param,   0, sizeof(H_param));
    memset( &H_collide,   0, sizeof(H_collide));

    ach_put(&chan_collide, &H_collide, sizeof(H_collide));

    /* Get latest ACH message */
    size_t fs = 0;
    int rr = ach_get( &chan_param, &H_param, sizeof(H_param), &fs, NULL, ACH_O_LAST );
    if(ACH_OK != rr) {
        if(debug) {
                fprintf(stderr, "param r = %s\n",ach_result_to_string(rr));}
        }
    else{    assert( sizeof(H_param) == fs); 
    }

    setNameToState(&H_state, &H_param);
    setAddressToState(&H_state, &H_param);
    setMdsParamState(&H_state, &H_param);

    // get current time
    //clock_gettime( CLOCK_MONOTONIC,&t);
    clock_gettime( 0,&t);



/* Create CAN Frame */
    struct can_frame frame;
//   Is this really how the frame should be initialized?
    memset( &frame.data,0, sizeof(frame.data));
    //sprintf( frame.data, "1234578" );
    frame.can_dlc = strlen( frame.data );

    /* buffer for return arg of decode */
    char buff[80];
    memset(&buff, 0, sizeof(buff));


    int startFlag = 1;







    /* ------------------------------------------------ */
    /* Put current location from robot on state and ref */
    /* ------------------------------------------------ */
//    if (vcan == 0) {
    if (vcan == -1) {
      /* get position of robot in joint space */
      for (int i = 0; i < 3; i++) {
        /* Request State (Enc pos) */
        requestState( can_skt );
        /* Read all data on CAN */
        clearCanBuff(can_skt, &H_state, &H_param, buff,  &frame);
        /* Read all data on CAN */
        clearCanBuff(can_skt, &H_state, &H_param, buff,  &frame);
      }
      
      /* set position of robot in joint space to ref */
      for (int i = 0; i < MDS_JOINT_COUNT; i++){
        H_ref.joint[i].ref = H_state.joint[i].pos;
      }
      /* Put on ACH Channels */
      ach_put(&chan_ref, &H_ref, sizeof(H_ref));
    }
//    else if (vcan == 1){
    else {
      for (int i = 0; i < MDS_JOINT_COUNT; i++){
        H_ref.joint[i].ref = -1.0*H_state.joint[i].offset * H_state.joint[i].direction;
      }
      ach_put(&chan_ref, &H_ref, sizeof(H_ref));
    }











    printf("Start MDS Loop\n");
    while(1) {
        // wait until next shot
        clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);

        fs = 0;

        /* Get latest ACH message */
        int r = ach_get( &chan_collide, &H_collide, sizeof(H_collide), &fs, NULL, ACH_O_LAST );
        if(ACH_OK != r) {
            if(debug) {
                    fprintf(stderr, "collide r = %s\n",ach_result_to_string(r));}
            }
        else{    assert( sizeof(H_collide) == fs); 
        }


        r = ach_get( &chan_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
        if(ACH_OK != r) {
            if(debug) {
                    fprintf(stderr, "Ref r = %s\n",ach_result_to_string(r));}
            }
        else{    assert( sizeof(H_ref) == fs); 
        }
        


        memcpy(&H_state.collide, &H_collide, sizeof(H_collide));
//        for (int i = 0; i < MDS_COLLIDE_JNT_NUM, i++){
//            H_state.collide[i].collision = H_collide.joint[i].collision;
//            H_state.collide[i].isCollide = H_collide.joint[i].isCollide;
//            H_state.collide[i].time      = H_collide.joint[i].time;
//        }
//        H_state.collide.isCollide = H_collide.isCollide;
//        H_state.collide.time      = H_collide.time;




        //int address = 0x004c; // LEB
        int address = 0x005F; // LSP
        double radss = 0.0872664626; // 5 deg/s^2
        double rad = 0.523598776; // 30 deg

        /* Set all Ref */
        setRefAll(&H_ref, &H_state, &H_ref_filter, &H_param, MDS_REF_FILTER_LENGTH, can_skt, &frame);

        /* Request State (Enc pos) */
        requestState( can_skt );
        /* Read all data on CAN */
        clearCanBuff(can_skt, &H_state, &H_param, buff,  &frame);


        // Get current timestamp to send out with the state struct
        clock_gettime( CLOCK_MONOTONIC, &time );
        tsec = (double)time.tv_sec;
        tsec += (double)(time.tv_nsec)/1.0e9;
        H_state.time = tsec; // add time based on system time

       // printf("\ndeg = %f  joint = %d time = %f \n",H_state.joint[0x004c].pos, 0x004c, H_state.time);
        /* put data back in ACH channel */
        ach_put( &chan_state, &H_state, sizeof(H_state));

        t.tv_nsec+=interval;
        tsnorm(&t);
        fflush(stdout);
        fflush(stderr);
    }

}


void setRefAll(mds_ref_t *r, mds_state_t *s, mds_ref_t *fi, mds_joint_param_t *p, int L, int skt, struct can_frame *f){
        
       refFilterMode(r, s, fi, L); 
       int j = 0;
      // for(int i = 0; i < MDS_JOINT_COUNT; i++) {
       for(int i = MDS_JOINT_COUNT-1; i >= 0; i--) {
          /* This worked but no offset */
          //double rad = s->joint[i].ref;
          /* with offset */
          double rad = s->joint[i].direction * (s->joint[i].ref + (s->joint[i].offset*s->joint[i].direction));
//          s->joint[i].pos = rad;

          //double radss = 0.0872664626; // 5 deg/s^2
          //double radss = 0.0872664626 * 2.0; // 10 deg/s^2
          //double radss = 0.0872664626 * 2.0 * 4.0; // 40 deg/s^2

          /* this is the acceleration */
          double radss = 0.0872664626 * 2.0 * 8.0; // 80 deg/s^2
          int address = p->joint[i].address;



        /* send CAN only if there is no colisions */
        if( s->collide.isCollide == 0){
          s->joint[i].ref_c_send = s->joint[i].ref;
          /* Send CAN only if commanded pos is not the same as previous pos */
          if (rad != deg_hist[i]){
            if (address > 0){
              setRef(rad, radss, address, p, skt, f);
              j = j+1;
            }
          }
          deg_hist[i] = rad;
        }
       }
       //printf("%d\n",j);
}


void setRef(double rad, double radss, int address, mds_joint_param_t *p, int skt, struct can_frame *f){
//        int address = 0x005F; // LSP
//        double radss = 0.0872664626; // 5 deg/s^2
//        double rad = 0.523598776; // 30 deg
        f_setAcc(radss, address, p, skt, f);
        f_setRef(rad, address, p, skt, f);
        f_setRefTrajectoryModeDefault(address, p, skt, f);
        f_setRefTrajectoryPeriodDefault(address, p, skt, f);
}

void requestState(int skt){
       struct can_frame txframe;
       memset( &txframe.data,0, sizeof(txframe.data));
       txframe.data[7] = 0x17;
       txframe.data[6] = 0x04;
       txframe.can_id = 0x11;
       txframe.can_dlc = 8;
       sendCan(skt,&txframe);
       
}

void clearCanBuff(int skt, mds_state_t *s, mds_joint_param_t *p, char *buff, struct can_frame *f) {
        for(int i = 0; i < MDS_CAN_BUFFER_CLEAR_I; i++) {
          int bytes_read = readCan( skt, f, 0);
          decodeFrame(s, p , buff, f);
        }
}



void refFilterMode(mds_ref_t *r, mds_state_t *s, mds_ref_t *f, int L) {
    int i = 0;
    double e = 0.0;
    for(i = 0; i < MDS_JOINT_COUNT; i++) {
        int c = (int)r->joint[i].mode;
//dan      switch (c) {
      switch (MDS_REF_MODE_REF) {
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

        s->joint[i].ref_r = r->joint[i].ref;
        s->joint[i].ref   = f->joint[i].ref; 
    } 
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

int getCmdPos(mds_state_t *s, mds_joint_param_t *p, char *buff, char *delm){
    /* Gets the position of the actuator from the "PareseResponse" buffer */
    /* items are converted into radians and put into the state channel */
    if(strstr(buff, "Desired Position") != NULL) { 
        char * pch;
        pch = strtok(buff,delm);
        double d = 0.0;
        int address = 0;
        int i = 0;
        while (pch != NULL){
          if ( i == 1){
           /* get address */
           address = (int)strtol(pch, NULL, 0);
           //printf ("%s\n",pch);
          }
          else if ( i == 4){
           /* get location */ 
           sscanf(pch,"%lf",&d);
          }
          pch = strtok (NULL, delm); 
          i++;
        }
        double rat = 2.0 * M_PI / (p->joint[address].encoderresolution * 
                                   p->joint[address].gearratio *
                                   p->joint[address].encoderconfig_tickspercount);
        s->joint[address].ref_c = d * rat;
//       printf("\ndeg = %f  joint = %d\n",d, address);
    }
    else return -1;
    
    return 0;
}


int getPos(mds_state_t *s, mds_joint_param_t *p, char *buff, char *delm){
    /* Gets the position of the actuator from the "PareseResponse" buffer */
    /* items are converted into radians and put into the state channel */
    if(strstr(buff, "Actual Position") != NULL) { 
        char * pch;
        pch = strtok(buff,delm);
        double d = 0.0;
        int address = 0;
        int i = 0;
        while (pch != NULL){
          if ( i == 1){
           /* get address */
           address = (int)strtol(pch, NULL, 0);
           //printf ("%s\n",pch);
          }
          else if ( i == 4){
           /* get location */ 
           sscanf(pch,"%lf",&d);
          }
          pch = strtok (NULL, delm); 
          i++;
        }
        double rat = 2.0 * M_PI / (p->joint[address].encoderresolution * 
                                   p->joint[address].gearratio *
                                   p->joint[address].encoderconfig_tickspercount);
        /* received state */
        //s->joint[address].pos = d * rat;
        
        /* state with offset */
        s->joint[address].pos = s->joint[i].direction * ((d * rat) - s->joint[i].offset);

//       printf("\ndeg = %f  joint = %d\n",d, address);
    }
    else return -1;
    
    return 0;
}

double rad2deg(double rad){
    return rad * 180.0 / M_PI;
}


float rad2enc(double radss, int address, mds_joint_param_t *p){
     return  deg2enc(rad2deg(radss), address, p);
}


float deg2enc(double deg, int address, mds_joint_param_t *p){
        double rat = 360.0 / (p->joint[address].encoderresolution * 
                                   p->joint[address].gearratio *
                                   p->joint[address].encoderconfig_tickspercount);
        return (float)(deg / rat);
}


void f_setAcc(double radss, int address, mds_joint_param_t *p, int skt, struct can_frame *f){
    /* sets the acceleration in rad/s^2 */
    float ftmp = rad2enc(radss, address, p);
    //float ftmp = deg2enc(rad2deg(radss), address, p);
    memcpy(&f->data, &ftmp, 4);
    f->can_id = address;
    f->can_dlc = 8;
    f->data[7] = COMMAND_SET;
    f->data[6] = ARGUMENT_STATE_TRAJECTORY_ACCELERATION;
    f->data[5] = 0;
    f->data[4] = 0;
    sendCan(skt,f);
}

void f_setRef(double ref, int address, mds_joint_param_t *p, int skt, struct can_frame *f){
    float ftmp = rad2enc(ref, address, p);
    memcpy(&f->data, &ftmp, 4);
    f->can_id = address;
    f->can_dlc = 8;
    f->data[7] = COMMAND_SET;
    f->data[6] = ARGUMENT_STATE_MOVETO_TARGET;
    f->data[5] = 0;
    f->data[4] = 0;
    sendCan(skt,f);
}

void f_setRefTrajectoryModeDefault(int address, mds_joint_param_t *p, int skt, struct can_frame *f){
    f_setRefTrajectoryMode(TRAJECTORY_MODE_MOVETO, address, p, skt, f);
}
void f_setRefTrajectoryMode(int mode, int address, mds_joint_param_t *p, int skt, struct can_frame *f){
    f->can_id = address;
    f->can_dlc = 8;
    f->data[7] = COMMAND_SET;
    f->data[6] = ARGUMENT_STATE_TRAJECTORY_MODE;
    f->data[5] = 0;
    f->data[4] = 0;
    f->data[3] = 0;
    f->data[2] = 0;
    f->data[1] = 0;
    f->data[0] = mode;
    sendCan(skt,f);
}

void f_setRefTrajectoryPeriodDefault(int address, mds_joint_param_t *p, int skt, struct can_frame *f){

    f_setRefTrajectoryPeriod(TRAJECTORY_PERIOD_19PT53125MS, address, p, skt, f);
}

void f_setRefTrajectoryPeriod(int period, int address, mds_joint_param_t *p, int skt, struct can_frame *f){
    int16_t itmp = (int16_t)period;
    memcpy(&f->data, &itmp, 2);
    f->can_id = address;
    f->can_dlc = 8;
    f->data[7] = COMMAND_SET;
    f->data[6] = ARGUMENT_STATE_TRAJECTORY_PERIOD;
    f->data[5] = 0;
    f->data[4] = 0;
    f->data[3] = 0;
    f->data[2] = 0;
    sendCan(skt,f);
}


int decodeFrame(mds_state_t *s, mds_joint_param_t *p, char *buff, struct can_frame *f) {
    int fs = (int)f->can_id;
    int cmd = (int)f->data[0];
    memset(buff, 0, sizeof(*buff));
    ParseResponse(buff,f->data,(unsigned long)floor(1234*1000.0));
    sprintf(buff,"%s\t%x",buff,f->can_id);
    char * delm = " ,\t";

    /* will check for the correct feedback inside of the funciton */
    getPos( s, p, buff, delm);
    getCmdPos( s, p, buff, delm);
//    printf("\ndeg = %f  joint = %d\n",s->joint[0x004c].pos, 0x004c);
    return 0; 
}

int main(int argc, char **argv) {

    // Parse user input
    debug = 0;
    int vcan = 0;


    int i = 1;
    while(argc > i)
    {
        if(strcmp(argv[i], "-d") == 0) {
            debug = 1;
        }
        if(strcmp(argv[i], "-v") == 0){
           /* virtural mode (no can) */
           vcan = 1;
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
    
    /* ini deg history */
    memset( &deg_hist, 0, sizeof(deg_hist));

    // open reference
    int r = ach_open(&chan_ref, MDS_CHAN_REF_NAME, NULL);
    assert( ACH_OK == r);

    // open state
    r = ach_open(&chan_state, MDS_CHAN_STATE_NAME, NULL);
    assert( ACH_OK == r);

    // open param channel
    r = ach_open(&chan_param, MDS_CHAN_PARAM_NAME, NULL);
    assert( ACH_OK == r);

    // open collision
    r = ach_open(&chan_collide, MDS_CHAN_COLLIDE_NAME, NULL);
    assert( ACH_OK == r);


    // initilize control channel
//    r = ach_open(&chan_board_cmd, MDS_CHAN_BOARD_CMD_NAME, NULL);
//    assert( ACH_OK == r);

    /* Open all CAN */
//    openAllCAN(0);
    if (vcan == 1) can_skt = openCAN("vcan42");
    else can_skt = openCAN("can0");

    /* Put on ACH Channels */
    //ach_put(&chan_ref, &H_ref, sizeof(H_ref));
//    ach_put(&chan_board_cmd, &H_cmd, sizeof(H_cmd));
//    ach_put(&chan_hubo_state, &H_state, sizeof(H_state));

    // run main loop
    mainLoop(vcan);

//    hubo_daemon_close();
    
    return 0;
}

