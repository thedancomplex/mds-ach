/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <boost/bind.hpp>
#include <ctime>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <iostream>

// For Ach
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include <ach.h>
#include <string.h>
#include "../include/mds.h"



mds_collide_t jnt;
int tstart = 0.0;

/* time that you must be collision free */
double tcut = 0.1;
int icut = 80;

/* Ach channels */
ach_channel_t chan_collide;  // collide channel




/////////////////////////////////////////////////
// Function is called everytime a message is received.
//void cb(gazebo::physics::ContactPtr &_msg)
//void cb(gazebo::msgs::Contact &_msg)
//void cb(gazebo::msgs::Contact &_msg)
//void cb(const std::basic_string<char> &_msg)
//void cb( gazebo::physics::WorldPtr &_msg)
void cb(const std::string &_msg)
{
    /* get time */
    int ustime = clock() - tstart;
    double sysTime = ustime/1000000.0;

    int jnti = -1;

    if((int)_msg.find("RHAND") > -1)           jnti = COLLISION_R_HAND;
    if((int)_msg.find("LHAND") > -1)           jnti = COLLISION_L_HAND;
    if((int)_msg.find("RSY") > -1)             jnti = COLLISION_R_SHOULDER;
    if((int)_msg.find("LSY") > -1)             jnti = COLLISION_L_SHOULDER;
    if((int)_msg.find("REP_body_fake") > -1)   jnti = COLLISION_R_FOREARM;
    else if ((int)_msg.find("REP_body") >-1)   jnti = COLLISION_R_ELBOW;
    if((int)_msg.find("LEP_body_fake") > -1)   jnti = COLLISION_L_FOREARM;
    else if ((int)_msg.find("LEP_body") >-1)   jnti = COLLISION_L_ELBOW;
    if((int)_msg.find("RWR") > -1)             jnti = COLLISION_R_WRIST;
    if((int)_msg.find("LWR") > -1)             jnti = COLLISION_L_WRIST;

    /* update joint and time */
    jnt.time_dt = sysTime - jnt.time;
    jnt.time = sysTime;
    if( jnti > -1){
      jnt.joint[jnti].collision = jnt.joint[jnti].collision + 1;
      jnt.joint[jnti].time_dt = sysTime - jnt.joint[jnti].time;
      jnt.joint[jnti].time = sysTime;
    }

    /* incroment collision */
    for( int i = 0; i < MDS_COLLIDE_JNT_NUM; i++){
      if((sysTime - jnt.joint[i].time) > tcut) jnt.joint[i].collision = jnt.joint[i].collision-1;
      if(jnt.joint[i].collision < 0) jnt.joint[i].collision = -1;
    }

    /* say if collided */
    for( int i = 0; i < MDS_COLLIDE_JNT_NUM; i++){
      if(jnt.joint[i].collision > icut) {
        jnt.joint[i].isCollide = 1;
      }
      else jnt.joint[i].isCollide = 0;
      if(jnt.joint[i].collision > icut*3) {
         jnt.joint[i].collision = icut*3;
      }
    }

    /* get total collions */
    int totalCollisions = 0;
    for( int i = 0; i < MDS_COLLIDE_JNT_NUM; i++){
       if (jnt.joint[i].isCollide == 1) totalCollisions = totalCollisions + 1;  
    }

    if (totalCollisions > 0) jnt.isCollide = 1;
    else jnt.isCollide = 0;

    /* put on ach channel */
    ach_put(&chan_collide, &jnt, sizeof(jnt));

}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{

  // ini
  memset(&jnt, 0, sizeof(jnt));
  tstart = clock();

  // open collision
  int r = ach_open(&chan_collide, MDS_CHAN_COLLIDE_NAME, NULL);
  assert( ACH_OK == r);
  ach_put(&chan_collide, &jnt, sizeof(jnt));


  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo world_stats topic
  //gazebo::transport::SubscriberPtr LHAND = node->Subscribe("/gazebo/default/MDS-Lofaro-Labs/LHAND_body/collision_sensor", cb);

  printf("MDS-Ach Collision Checker Running: Lofaro Labs\n");
  gazebo::transport::SubscriberPtr contact_all = node->Subscribe("~/physics/contacts", cb);
 /*
  gazebo::transport::SubscriberPtr LE = node->Subscribe("/gazebo/default/MDS-Lofaro-Labs/LEP_body/collision_sensor", cb);
  gazebo::transport::SubscriberPtr LA_B = node->Subscribe("/gazebo/default/MDS-Lofaro-Labs/LEP_body_fake/collision_sensor", cb);
  gazebo::transport::SubscriberPtr LHAND = node->Subscribe("/gazebo/default/MDS-Lofaro-Labs/LHAND_body/collision_sensor", cb);
  gazebo::transport::SubscriberPtr LS = node->Subscribe("/gazebo/default/MDS-Lofaro-Labs/LSY_body/collision_sensor", cb);
  gazebo::transport::SubscriberPtr LW = node->Subscribe("/gazebo/default/MDS-Lofaro-Labs/LWR_body/collision_sensor", cb);

  gazebo::transport::SubscriberPtr RE = node->Subscribe("/gazebo/default/MDS-Lofaro-Labs/REP_body/collision_sensor", cb);
  gazebo::transport::SubscriberPtr RA_B = node->Subscribe("/gazebo/default/MDS-Lofaro-Labs/REP_body_fake/collision_sensor", cb);
  gazebo::transport::SubscriberPtr RHAND = node->Subscribe("/gazebo/default/MDS-Lofaro-Labs/RHAND_body/collision_sensor", cb);
  gazebo::transport::SubscriberPtr RS = node->Subscribe("/gazebo/default/MDS-Lofaro-Labs/RSY_body/collision_sensor", cb);
  gazebo::transport::SubscriberPtr RW = node->Subscribe("/gazebo/default/MDS-Lofaro-Labs/RWR_body/collision_sensor", cb);
*/
  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
