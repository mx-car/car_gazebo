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

/*
 * @file  gazebo_ros_wheel_steerable.h
 * @brief A driver to control one or more steerable wheels
 */

#ifndef WHEEL_STEERABLE_PLUGIN_HH
#define WHEEL_STEERABLE_PLUGIN_HH

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

  class Joint;
  class Entity;

  class GazeboRosWheelsSteerable : public ModelPlugin {

    enum Wheel {
        FRONT_LEFT = 0,
        FRONT_RIGHT = 1,
        REAR_LEFT = 2,
        REAR_RIGHT = 3,
    };
    public:
      GazeboRosWheelsSteerable();
      ~GazeboRosWheelsSteerable();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void Reset();

    protected:
      virtual void UpdateChild();
      virtual void FiniChild();
      virtual void UpdateOdometryEncoder();
      virtual void PublishOdometry();
      virtual void GetGroundTruth();
      virtual void SetCovariance();


    private:

      gazebo::physics::JointControllerPtr joint_controller_;

      GazeboRosPtr gazebo_ros_;
      physics::ModelPtr parent;
      event::ConnectionPtr update_connection_;


      std::vector< physics::JointPtr> joints_rotation_;

      // ROS STUFF
      ros::Subscriber cmd_vel_subscriber_;
      ros::Subscriber pid_msg_subscriber_;
      ros::Publisher odometry_publisher_;
      ros::Publisher ground_truth_odometry_publisher_;
      ros::Publisher current_position_publisher_;
      ros::Publisher target_position_publisher_;
      ros::Publisher applied_force_publisher_;
      geometry_msgs::TwistConstPtr cmd_twist_;
      control_msgs::JointControllerState pid_msg;

      boost::mutex lock;
      bool alive_;

      std::string topic_cmd_twist_;
      std::string topic_pid_msg_;
      std::string topic_odom_;
      std::string frame_odom_;
      std::string topic_ground_truth_;
      std::string frame_ground_truth_;
      std::string frame_base_;
      std::string joint_rear_left_;
      std::string joint_rear_right_;
      std::string joint_steering_left_;
      std::string joint_steering_right_;
      std::string topic_current_position_;
      std::string topic_target_position_;
      std::string topic_applied_force_;

      double wheelbase_distance_;
      double kingpin_distance_;
      double max_steering_angle_;
      
      //PID controller parameter

      double max_effort_pid_;
      double pid_p_;
      double pid_i_;
      double pid_d_;
      double dt;
      struct PID_Controller_State{
          double max_effort_pid_;
          double pid_p_;
          double pid_i_;
          double pid_d_;
          double dt;

          double _integral;
          double _pre_error;
      };

      PID_Controller_State pid_controller_front_left;
      PID_Controller_State pid_controller_front_right;
      void setPIDParameters(PID_Controller_State &state, double p, double i, double d, double maxEffort, double dt);
      double calculatePID(PID_Controller_State state, double setValue, double currentValue);


      double update_rate_controller_;
      double torque_max_wheel_;
      

      //Odometry
      nav_msgs::Odometry odom_;
      geometry_msgs::Pose2D pose_encoder_;
      double wheel_radius_;
      double update_period_;
      common::Time last_odom_update_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      // DiffDrive stuff
      void callbackTopicCMD(const geometry_msgs::Twist::ConstPtr& cmd_msg);
      void callbackTopicPID(const control_msgs::JointControllerState::ConstPtr &pid_msg);

      // Update Rate
      common::Time last_update_time_;
      

  };

}

#endif
