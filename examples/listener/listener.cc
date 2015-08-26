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

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <iostream>

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
    printf("%d\n",_msg.size());
    printf("%d\n",_msg.find("MDS"));
   // printf("%s\n",_msg.c_str());
//    gazebo::msgs::Contact contact;
    std::map<std::string, gazebo::msgs::Contact> con = _msg;
//    memcpy(&contact, &_msg, sizeof(contact));
//    printf("%d\n",contact.time().sec());
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
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
