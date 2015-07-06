/* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
/*
Copyright (c) 2015, Daniel M. Lofaro
Copyright (c) 2014, Daniel M. Lofaro
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

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>

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
#include "../../include/mds.h"
/*
// ach channels
ach_channel_t chan_diff_drive_ref;      // hubo-ach
ach_channel_t chan_time;

int debug = 0;
double H_ref[2] = {};
double ttime = 0.0;
*/

mds_state_t H_state;
ach_channel_t chan_state;     // mds-ach-state


namespace gazebo
{   
  class MdsPlugin : public ModelPlugin
  {

    // Function is called everytime a message is received.
//void cb(gazebo::msgs::Image &_msg)
//void cb(const std::string& _msg)
//void cb(gazebo::msgs::ImageStamped &_msg)
//void cb(ConstWorldStatisticsPtr &_msg)
//void cb(const std::string& _msg)
    public: void cb(ConstWorldStatisticsPtr &_msg)
    {
       gazebo::common::Time simTime  = gazebo::msgs::Convert(_msg->sim_time());
       //size_t size;
       double ttime = simTime.Double();
       //ach_put(&chan_time, _msg->image().data().c_str() , _msg->image().data().size());

    }


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) 
    {

      // Open Ach channel
        /* open ach channel */
        size_t fs;
        int r = ach_open(&chan_state, MDS_CHAN_STATE_NAME, NULL);
        memset( &H_state, 0, sizeof(H_state));

        r = ach_get( &chan_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
        


/*  --------
        memset( &H_ref,   0, sizeof(H_ref));
        int r = ach_open(&chan_diff_drive_ref, "robot-diff-drive" , NULL);
        assert( ACH_OK == r );
        ach_put(&chan_diff_drive_ref, &H_ref , sizeof(H_ref));

        
    
        memset( &ttime,   0, sizeof(ttime));
        r = ach_open(&chan_time, "robot-time" , NULL);
        assert( ACH_OK == r );
        ach_put(&chan_time, &ttime , sizeof(ttime));
----------- */

      // Store the pointer to the model
      this->model = _parent;

      // Get then name of the parent model
//      std::string modelName = _sdf->GetParent()->Get<std::string>("name");

      // Get the world name.
//      std::string worldName = _sdf->GetName();
     this->world = physics::get_world("default");


      // Load parameters for this plugin
      if (this->LoadParams(_sdf))
      {
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&MdsPlugin::OnUpdate, this));
      }

      // subscribe to thread
//      gazebo::transport::NodePtr node(new gazebo::transport::Node());
//      node->Init();
//      gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/world_stats", cb);
    }

    public: bool LoadParams(sdf::ElementPtr _sdf) 
    {
      if (this->FindJointByParam(_sdf, this->joint_LSP_,      "LSP") &&
          this->FindJointByParam(_sdf, this->joint_LSR_,      "LSR") &&
          this->FindJointByParam(_sdf, this->joint_LSY_,      "LSY") &&
          this->FindJointByParam(_sdf, this->joint_LEP_,      "LEP") &&
          this->FindJointByParam(_sdf, this->joint_LWR_,      "LWR") &&
          this->FindJointByParam(_sdf, this->joint_LWY_,      "LWY") &&

          this->FindJointByParam(_sdf, this->joint_RSP_,      "RSP") &&
          this->FindJointByParam(_sdf, this->joint_RSR_,      "RSR") &&
          this->FindJointByParam(_sdf, this->joint_RSY_,      "RSY") &&
          this->FindJointByParam(_sdf, this->joint_REP_,      "REP") &&
          this->FindJointByParam(_sdf, this->joint_RWR_,      "RWR") &&
          this->FindJointByParam(_sdf, this->joint_RWY_,      "RWY"))// &&

       //   this->FindJointByParam(_sdf, this->joint_TSY_,      "TSY"))

        return true;
      else
        return false;
    }

    public: bool FindJointByParam(sdf::ElementPtr _sdf,
                                  physics::JointPtr &_joint,
                                  std::string _param)
    {
      if (!_sdf->HasElement(_param))
      {
        gzerr << "param [" << _param << "] not found\n";
        return false;
      }
      else
      {
        _joint = this->model->GetJoint(
          _sdf->GetElement(_param)->Get<std::string>());
         // _sdf->GetElement(_param)->GetValueString());

        if (!_joint)
        {
          gzerr << "joint by name ["
                << _sdf->GetElement(_param)->Get<std::string>()
                //<< _sdf->GetElement(_param)->GetValueString()
                << "] not found in model\n";
          return false;
        }
      }
      return true;
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      double maxTorque = 500;
      double iniAngle = 0.2;

      size_t fs;
      int r = ach_get( &chan_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
      
      


      this->joint_LSP_->SetParam("friction",0, maxTorque);
      this->joint_LSR_->SetParam("friction",0, maxTorque);
      this->joint_LSY_->SetParam("friction",0, maxTorque);
      this->joint_LEP_->SetParam("friction",0, maxTorque);
      this->joint_LWR_->SetParam("friction",0, maxTorque);
      this->joint_LWY_->SetParam("friction",0, maxTorque);

      this->joint_RSP_->SetParam("friction",0, maxTorque);
      this->joint_RSR_->SetParam("friction",0, maxTorque);
      this->joint_RSY_->SetParam("friction",0, maxTorque);
      this->joint_REP_->SetParam("friction",0, maxTorque);
      this->joint_RWR_->SetParam("friction",0, maxTorque);
      this->joint_RWY_->SetParam("friction",0, maxTorque);

     // this->joint_TSY_->SetParam("friction",0, maxTorque);


      for( int i = 0; i < MDS_JOINT_COUNT; i++){
         double pos = H_state.joint[i].ref;
         std::string strName1= std::string(H_state.joint[i].nameshort);
         std::string strName = strName1.c_str();
//         printf("state = %s\n",strName.c_str());
         if( strName.compare(0,3,"LSP") == 0) this->joint_LSP_->SetPosition(0, pos);
         if( strName.compare(0,3,"LSR") == 0) this->joint_LSR_->SetPosition(0, pos);
         if( strName.compare(0,3,"LSY") == 0) this->joint_LSY_->SetPosition(0, pos);
         if( strName.compare(0,3,"LEP") == 0) this->joint_LEP_->SetPosition(0, pos);
         if( strName.compare(0,3,"LWR") == 0) this->joint_LWR_->SetPosition(0, pos);
         if( strName.compare(0,3,"LWY") == 0) this->joint_LWY_->SetPosition(0, pos);
         if( strName.compare(0,3,"RSP") == 0) this->joint_RSP_->SetPosition(0, pos);
         if( strName.compare(0,3,"RSR") == 0) this->joint_RSR_->SetPosition(0, pos);
         if( strName.compare(0,3,"RSY") == 0) this->joint_RSY_->SetPosition(0, pos);
         if( strName.compare(0,3,"REP") == 0) this->joint_REP_->SetPosition(0, pos);
         if( strName.compare(0,3,"RWR") == 0) this->joint_RWR_->SetPosition(0, pos);
         if( strName.compare(0,3,"RWY") == 0) this->joint_RWY_->SetPosition(0, pos);
      }
/*
      this->joint_LSP_->SetPosition(0, iniAngle);
      this->joint_LSR_->SetPosition(0, iniAngle);
      this->joint_LSY_->SetPosition(0, iniAngle);
      this->joint_LEP_->SetPosition(0, H_state.joint[0x004c].ref);
      this->joint_LWR_->SetPosition(0, iniAngle);
      this->joint_LWY_->SetPosition(0, iniAngle);
      
      this->joint_RSP_->SetPosition(0, iniAngle);
      this->joint_RSR_->SetPosition(0, iniAngle);
      this->joint_RSY_->SetPosition(0, iniAngle);
      this->joint_REP_->SetPosition(0, iniAngle);
      this->joint_RWR_->SetPosition(0, iniAngle);
      this->joint_RWY_->SetPosition(0, iniAngle);
      
   //   this->joint_TSY_->SetPosition(0, iniAngle);

*/


/*  ---- Ach Control ---
      // Get Ach chan data
    size_t fs;
    int r = ach_get( &chan_diff_drive_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );

    if(ACH_OK != r | ACH_STALE_FRAMES != r | ACH_MISSED_FRAME != r) {
                if(debug) {
                  //      printf("Ref ini r = %s\n",ach_result_to_string(r));}
                   printf("ref int r = %i \n\r",r);
		 }
        }
        else{   assert( sizeof(H_ref) == fs ); }

      this->left_wheel_joint_->SetParam("friction",0, 7);
      this->right_wheel_joint_->SetParam("friction",0, 7);
      this->right_wheel_joint_->SetVelocity(0, H_ref[0]);
      this->left_wheel_joint_->SetVelocity(0, H_ref[1]);

      ttime = this->world->GetSimTime().Double();
      //printf("- %f\n\r", ttime);

      ach_put(&chan_time, &ttime, sizeof(ttime));
//      double tmp = this->GetSimTime().double();
//      this->right_wheel_joint_->SetForce(0, H_ref[0]);
//      this->left_wheel_joint_->SetForce(0, H_ref[1]);

     //this->left_wheel_joint_->SetParam("friction",0, 10);
      //this->right_wheel_joint_->SetParam("friction",0, 10);
      //this->left_wheel_joint_->SetForce(0, -0.5);
//      this->left_wheel_joint_->SetForce(0, 0.2);
      //this->right_wheel_joint_->SetVelocity(0,0.5);
//      this->right_wheel_joint_->SetForce(0, -0.2);

      
--- Ach control end --*/



    }

    // Pointer to the model
    private: physics::ModelPtr model;
    private: physics::WorldPtr world;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: physics::JointPtr joint_LSP_;
    private: physics::JointPtr joint_LSR_;
    private: physics::JointPtr joint_LSY_;
    private: physics::JointPtr joint_LEP_;
    private: physics::JointPtr joint_LWR_;
    private: physics::JointPtr joint_LWY_;

    private: physics::JointPtr joint_RSP_;
    private: physics::JointPtr joint_RSR_;
    private: physics::JointPtr joint_RSY_;
    private: physics::JointPtr joint_REP_;
    private: physics::JointPtr joint_RWR_;
    private: physics::JointPtr joint_RWY_;

  //  private: physics::JointPtr joint_TSY_;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MdsPlugin)
}
