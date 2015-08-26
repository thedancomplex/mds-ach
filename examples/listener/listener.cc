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

#define jntLen 10
typedef struct collision_jnt{
   int collision;
   int collide;
   double time;
}__attribute((packed)) jnt_t;

jnt_t jnt[jntLen];
int tstart = 0.0;

/* time that you must be collision free */
double tcut = 0.7;
int icut = 50;

#define LHAND    0
#define RHAND    1
#define RSY      2
#define LSY      3
#define REP_fake 4
#define LEP_fake 5
#define REP_body 6
#define LEP_body 7
#define RWR      8
#define LWR      9





/////////////////////////////////////////////////
// Function is called everytime a message is received.
//void cb(gazebo::physics::ContactPtr &_msg)
//void cb(gazebo::msgs::Contact &_msg)
//void cb(gazebo::msgs::Contact &_msg)
//void cb(const std::basic_string<char> &_msg)
//void cb( gazebo::physics::WorldPtr &_msg)
void cb(const std::string &_msg)
{
   
  // Dump the message contents to stdout.
//  std::cout << _msg->DebugString();
    //gazebo::physics::ContactManager con[100];
//    std::cout << con.gazebo::physics::ContactManager::GetContactCount();
    //gazebo::msgs::Contact con = gazebo::msgs::Contact();
    //gazebo::sensors::ContactSensor con;
    //unsigned int c = con.GetCollisionContactCount(_msg);
    //int c = _msg->gazebo::physics::ContactManager::GetContactCount();

    /* get time */
    int ustime = clock() - tstart;
    double sysTime = ustime/1000000.0;
//    std::cout << "System time = " << sysTime << "\n";


    int jnti = -1;

    if((int)_msg.find("RHAND") > -1)           jnti = RHAND;
    if((int)_msg.find("LHAND") > -1)           jnti = LHAND;
    if((int)_msg.find("RSY") > -1)             jnti = RSY;
    if((int)_msg.find("LSY") > -1)             jnti = LSY;
    if((int)_msg.find("REP_body_fake") > -1)   jnti = REP_fake;
    else if ((int)_msg.find("REP_body") >-1)   jnti = REP_body;
    if((int)_msg.find("LEP_body_fake") > -1)   jnti = LEP_fake;
    else if ((int)_msg.find("LEP_body") >-1)   jnti = LEP_body;
    if((int)_msg.find("RWR") > -1)             jnti = RWR;
    if((int)_msg.find("LWR") > -1)             jnti = LWR;

    /* update joint and time */
    if( jnti > -1){
      jnt[jnti].collision = jnt[jnti].collision + 1;
      jnt[jnti].time = sysTime;
    }

    /* incroment collision */
    for( int i = 0; i < jntLen; i++){
      if((sysTime - jnt[i].time) > tcut) jnt[i].collision = jnt[i].collision-1;
      if(jnt[i].collision < 0) jnt[i].collision = -1;
    }

    /* say if collided */
    for( int i = 0; i < jntLen; i++){
      if(jnt[i].collision > icut) jnt[i].collide = 1;
      else jnt[i].collide = 0;
    }

    /* print */
    int totalCollisions = 0;
    printf("Col  = ");
    for( int i = 0; i < jntLen; i++){
       if (jnt[i].collide == 1){
       totalCollisions = totalCollisions + 1;  
       printf(" - %d ",i);
       }
    }
    printf("\n");


//    printf("%d\n",totalCollisions);
//    printf("%d\n",_msg.size());
//    printf("%d\n",_msg.find("LHAND"));




   // printf("%s\n",_msg.c_str());
//    gazebo::msgs::Contact contact;
//    std::map<std::string, gazebo::msgs::Contact> con = _msg;
//    memcpy(&contact, &_msg, sizeof(contact));
//    printf("%d\n",contact.time().sec());
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{

  // ini
  memset(&jnt, 0, sizeof(jnt));
  tstart = clock();


  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo world_stats topic
  //gazebo::transport::SubscriberPtr LHAND = node->Subscribe("/gazebo/default/MDS-Lofaro-Labs/LHAND_body/collision_sensor", cb);
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
