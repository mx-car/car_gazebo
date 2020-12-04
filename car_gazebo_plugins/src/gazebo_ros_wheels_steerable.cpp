/*
 * Copyright (c) 2020, Markus Bader
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_wheels_steerable.h>

#ifdef ENABLE_PROFILER
#include <ignition/common/Profiler.hh>
#endif

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>

namespace gazebo
{


GazeboRosWheelsSteerable::GazeboRosWheelsSteerable() {}

// Destructor
GazeboRosWheelsSteerable::~GazeboRosWheelsSteerable()
{
    FiniChild();
}

// Load the controller
void GazeboRosWheelsSteerable::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{

    this->parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "WheelsSteerable" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<std::string> ( topic_cmd_twist_,          "topicTwist",  "cmd_vel" );
    gazebo_ros_->getParameter<std::string> ( topic_odom_,               "topicOdom", "odom" );
    gazebo_ros_->getParameter<std::string> ( frame_odom_,               "frameOdom", "odom" );
    gazebo_ros_->getParameter<std::string> ( frame_base_,               "frameBase", "base_link" );
    gazebo_ros_->getParameter<std::string> ( joint_rear_left_,          "jointRearLeft", "wheel_axis_rear_left_joint" );
    gazebo_ros_->getParameter<std::string> ( joint_rear_right_,         "jointRearRight", "wheel_axis_rear_right_joint" );
    gazebo_ros_->getParameter<std::string> ( joint_steering_left_,      "jointSteeringLeft", "wheel_mount_front_left_joint" );
    gazebo_ros_->getParameter<std::string> ( joint_steering_right_,     "jointSteeringRight", "wheel_mount_front_right_joint" );
    
    gazebo_ros_->getParameter<double>      ( update_rate_controller_,   "updateRateController", 100.0 );
    
    gazebo_ros_->getParameter<double> ( torque_max_wheel_, "torqueMaxWheel", 5.0 );

    joints_rotation_.resize(4);
    
    joints_rotation_[REAR_LEFT ] = _parent->GetJoint(joint_rear_left_);
    if (!joints_rotation_[REAR_LEFT ]) {
        char error[200];
        snprintf(error, 200, "%s: couldn't get wheel hinge joint named %s", gazebo_ros_->info(), joint_rear_left_.c_str());
        gzthrow(error);
    }
    joints_rotation_[REAR_RIGHT ] = _parent->GetJoint(joint_rear_right_);
    if (!joints_rotation_[REAR_RIGHT ]) {
        char error[200];
        snprintf(error, 200, "%s: couldn't get wheel hinge joint named %s", gazebo_ros_->info(), joint_rear_right_.c_str());
        gzthrow(error);
    }
    
    joints_rotation_[FRONT_LEFT ] = _parent->GetJoint(joint_steering_left_);
    if (!joints_rotation_[FRONT_LEFT]) {
        char error[200];
        snprintf(error, 200, "%s: couldn't get wheel hinge joint named %s", gazebo_ros_->info(), joint_steering_left_.c_str());
        gzthrow(error);
    }
    joints_rotation_[FRONT_RIGHT] = _parent->GetJoint(joint_steering_right_);
    if (!joints_rotation_[FRONT_RIGHT]) {
        char error[200];
        snprintf(error, 200, "%s: couldn't get wheel hinge joint named %s", gazebo_ros_->info(), joint_steering_right_.c_str());
        gzthrow(error);
    }
    
    joints_rotation_[REAR_LEFT ]->SetParam ( "fmax", 0, torque_max_wheel_ );
    joints_rotation_[REAR_RIGHT]->SetParam ( "fmax", 0, torque_max_wheel_ );
    joints_rotation_[FRONT_LEFT ]->SetParam ( "fmax", 0, torque_max_wheel_ );
    joints_rotation_[FRONT_RIGHT]->SetParam ( "fmax", 0, torque_max_wheel_ );

    ROS_WARN("WheelsSteerable list");
    auto joints = _parent->GetJoints();
    for(const auto &j: joints){
        ROS_INFO_NAMED("joints", "%s", j->GetName().c_str());
    }

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(topic_cmd_twist_, 1,
                boost::bind(&GazeboRosWheelsSteerable::callbackTopicCMD, this, _1),
                ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
    ROS_INFO_NAMED("WheelsSteerable", "%s: Subscribe to %s", gazebo_ros_->info(), topic_cmd_twist_.c_str());


    // start custom queue for diff drive
    this->callback_queue_thread_ =
        boost::thread ( boost::bind ( &GazeboRosWheelsSteerable::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
        event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosWheelsSteerable::UpdateChild, this ) );

    alive_ = true;


    //setup joint controllers for steering
    this->joint_controller_ = this->parent->GetJointController();
    common::PID parameter = common::PID();
    parameter.SetPGain(100);
    parameter.SetIGain(10);
    parameter.SetDGain(1);
    parameter.SetCmdMax(100);
    parameter.SetCmdMin(-100);


    this->joint_controller_->SetPositionPID(
        joints_rotation_[FRONT_LEFT ]->GetScopedName(), parameter);
    this->joint_controller_->SetPositionPID(
        joints_rotation_[FRONT_RIGHT ]->GetScopedName(), parameter);
}

void GazeboRosWheelsSteerable::Reset()
{
    last_update_time_ = parent->GetWorld()->SimTime();
}


// Update the controller
void GazeboRosWheelsSteerable::UpdateChild()
{
#ifdef ENABLE_PROFILER
    IGN_PROFILE("GazeboRosWheelsSteerable::UpdateChild");
    IGN_PROFILE_BEGIN("update");
#endif

    if(cmd_twist_) {
        joints_rotation_[REAR_LEFT  ]->SetParam ( "vel", 0,  -cmd_twist_->linear.x );
        joints_rotation_[REAR_RIGHT ]->SetParam ( "vel", 0,   cmd_twist_->linear.x );
        joints_rotation_[FRONT_LEFT  ]->SetParam ( "vel", 0,  -cmd_twist_->linear.x );
        joints_rotation_[FRONT_RIGHT ]->SetParam ( "vel", 0,   cmd_twist_->linear.x );
        
        //ROS_INFO_NAMED("WheelsSteerable", "Set front right wheel to position %lf", -(M_PI/6) * cmd_twist_->angular.z);
        this->joint_controller_->SetPositionTarget(joints_rotation_[FRONT_RIGHT]->GetScopedName(),  -(M_PI/6) * cmd_twist_->angular.z);
        this->joint_controller_->SetPositionTarget(joints_rotation_[FRONT_LEFT]->GetScopedName(), M_PI/6 * cmd_twist_->angular.z);
        double currentPos = joints_rotation_[FRONT_LEFT]->Position(0);
        //ROS_INFO_NAMED("WheelsSteerable", "Current front right wheel position %lf", currentPos);
        this->joint_controller_->Update();
    }
    
#ifdef ENABLE_PROFILER
    IGN_PROFILE_END();
#endif
}

// Finalize the controller
void GazeboRosWheelsSteerable::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}
void GazeboRosWheelsSteerable::QueueThread()
{
    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

void GazeboRosWheelsSteerable::callbackTopicCMD ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
    cmd_twist_ = cmd_msg;
}

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosWheelsSteerable )
}
