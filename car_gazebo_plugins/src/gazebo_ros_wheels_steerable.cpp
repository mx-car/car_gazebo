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
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

#include <math.h>
#include <ros/ros.h>

namespace gazebo
{

        GazeboRosWheelsSteerable::GazeboRosWheelsSteerable()
        {
        }

        // Destructor
        GazeboRosWheelsSteerable::~GazeboRosWheelsSteerable()
        {
                FiniChild();
        }

        // Load the controller
        void GazeboRosWheelsSteerable::Load(physics::ModelPtr _parent,
                                            sdf::ElementPtr _sdf)
        {

                this->parent = _parent;
                gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "WheelsSteerable"));
                // Make sure the ROS node for Gazebo has already been initialized
                gazebo_ros_->isInitialized();

                gazebo_ros_->getParameter<std::string>(topic_cmd_twist_, "topicTwist",
                                                       "cmd_vel");
                gazebo_ros_->getParameter<std::string>(topic_pid_msg_, "topicPidMsg",
                                                       "pid_msg");
                gazebo_ros_->getParameter<std::string>(topic_odom_, "topicOdom", "odom");
                gazebo_ros_->getParameter<std::string>(frame_odom_, "frameOdom", "odom");
                gazebo_ros_->getParameter<std::string>(topic_ground_truth_, "topicGroundTruth", "ground_truth");
                gazebo_ros_->getParameter<std::string>(frame_ground_truth_, "frameGroundTruth", "ground_truth");
                gazebo_ros_->getParameter<std::string>(frame_base_, "frameBase", "base_link");
                gazebo_ros_->getParameter<std::string>(joint_rear_left_, "jointRearLeft",
                                                       "wheel_axis_rear_left_joint");
                gazebo_ros_->getParameter<std::string>(joint_rear_right_, "jointRearRight",
                                                       "wheel_axis_rear_right_joint");
                gazebo_ros_->getParameter<std::string>(joint_steering_left_,
                                                       "jointSteeringLeft",
                                                       "wheel_mount_front_left_joint");
                gazebo_ros_->getParameter<std::string>(joint_steering_right_,
                                                       "jointSteeringRight",
                                                       "wheel_mount_front_right_joint");

                gazebo_ros_->getParameter<double>(update_rate_controller_,
                                                  "updateRateController", 100.0);

                gazebo_ros_->getParameter<double>(torque_max_wheel_, "torqueMaxWheel", 5.0);

                gazebo_ros_->getParameter<double>(wheelbase_distance_, "wheelbaseDistance",
                                                  0.26);
                gazebo_ros_->getParameter<double>(kingpin_distance_, "kingpinDistance", 0.15);
                gazebo_ros_->getParameter<double>(max_steering_angle_, "maxSteeringAngle",
                                                  M_PI / 6);

                gazebo_ros_->getParameter<double>(max_effort_pid_, "maxEffortSteeringPid",
                                                  5.12);
                gazebo_ros_->getParameter<double>(pid_p_, "pidP", 15.0);
                gazebo_ros_->getParameter<double>(pid_i_, "pidI", 500.0);
                gazebo_ros_->getParameter<double>(pid_d_, "pidD", 20.0);

                gazebo_ros_->getParameter<double>(wheel_radius_, "wheelRadius", 0.03225);

                gazebo_ros_->getParameter<std::string>(topic_current_position_, "topicCurrentPosition", "current_position");
                gazebo_ros_->getParameter<std::string>(topic_target_position_, "topicTargetPosition", "target_position");
                gazebo_ros_->getParameter<std::string>(topic_applied_force_, "topicAppliedForce", "applied_force");

                joints_rotation_.resize(4);

                joints_rotation_[REAR_LEFT] = _parent->GetJoint(joint_rear_left_);
                if (!joints_rotation_[REAR_LEFT])
                {
                        char error[200];
                        snprintf(error, 200, "%s: couldn't get wheel hinge joint named %s",
                                 gazebo_ros_->info(), joint_rear_left_.c_str());
                        gzthrow(error);
                }
                joints_rotation_[REAR_RIGHT] = _parent->GetJoint(joint_rear_right_);
                if (!joints_rotation_[REAR_RIGHT])
                {
                        char error[200];
                        snprintf(error, 200, "%s: couldn't get wheel hinge joint named %s",
                                 gazebo_ros_->info(), joint_rear_right_.c_str());
                        gzthrow(error);
                }

                joints_rotation_[FRONT_LEFT] = _parent->GetJoint(joint_steering_left_);
                if (!joints_rotation_[FRONT_LEFT])
                {
                        char error[200];
                        snprintf(error, 200, "%s: couldn't get wheel hinge joint named %s",
                                 gazebo_ros_->info(), joint_steering_left_.c_str());
                        gzthrow(error);
                }
                joints_rotation_[FRONT_RIGHT] = _parent->GetJoint(joint_steering_right_);
                if (!joints_rotation_[FRONT_RIGHT])
                {
                        char error[200];
                        snprintf(error, 200, "%s: couldn't get wheel hinge joint named %s",
                                 gazebo_ros_->info(), joint_steering_right_.c_str());
                        gzthrow(error);
                }

                joints_rotation_[REAR_LEFT]->SetParam("fmax", 0, torque_max_wheel_);
                joints_rotation_[REAR_RIGHT]->SetParam("fmax", 0, torque_max_wheel_);
                joints_rotation_[FRONT_LEFT]->SetParam("fmax", 0, torque_max_wheel_);
                joints_rotation_[FRONT_RIGHT]->SetParam("fmax", 0, torque_max_wheel_);

                ROS_WARN("WheelsSteerable list");
                auto joints = _parent->GetJoints();
                for (const auto &j : joints)
                {
                        ROS_INFO_NAMED("joints", "%s", j->GetName().c_str());
                }

                ros::SubscribeOptions so =
                    ros::SubscribeOptions::create<geometry_msgs::Twist>(
                        topic_cmd_twist_, 1,
                        boost::bind(&GazeboRosWheelsSteerable::callbackTopicCMD, this, _1),
                        ros::VoidPtr(), &queue_);

                cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
                ROS_INFO_NAMED("WheelsSteerable", "%s: Subscribe to %s", gazebo_ros_->info(),
                               topic_cmd_twist_.c_str());

                ros::SubscribeOptions so1 =
                    ros::SubscribeOptions::create<control_msgs::JointControllerState>(
                        topic_pid_msg_, 1,
                        boost::bind(&GazeboRosWheelsSteerable::callbackTopicPID, this, _1),
                        ros::VoidPtr(), &queue_);

                pid_msg_subscriber_ = gazebo_ros_->node()->subscribe(so1);
                ROS_INFO_NAMED("WheelsSteerable", "%s: Subscribe to %s", gazebo_ros_->info(),
                               topic_pid_msg_.c_str());

                // start custom queue for diff drive
                this->callback_queue_thread_ =
                    boost::thread(boost::bind(&GazeboRosWheelsSteerable::QueueThread, this));

                // listen to the update event (broadcast every simulation iteration)
                this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&GazeboRosWheelsSteerable::UpdateChild, this));

                alive_ = true;

                odometry_publisher_ =
                    gazebo_ros_->node()->advertise<nav_msgs::Odometry>(topic_odom_, 1);
                ROS_INFO_NAMED("WheelsSteerable", "%s: Advertise odom on %s ",
                               gazebo_ros_->info(), topic_odom_.c_str());

                ground_truth_odometry_publisher_ =
                    gazebo_ros_->node()->advertise<nav_msgs::Odometry>(topic_ground_truth_, 1);

                current_position_publisher_ =
                    gazebo_ros_->node()->advertise<std_msgs::Float64>(topic_current_position_, 1);

                target_position_publisher_ =
                    gazebo_ros_->node()->advertise<std_msgs::Float64>(topic_target_position_, 1);

                applied_force_publisher_ =
                    gazebo_ros_->node()->advertise<std_msgs::Float64>(topic_applied_force_, 1);

                update_period_ = 0.1;
                pose_encoder_.x = 0.0;
                pose_encoder_.y = 0.0;
                pose_encoder_.theta = 0.0;

                // setup PID controllers for steering
                setPIDParameters(pid_controller_front_left, pid_p_, pid_i_, pid_d_, max_effort_pid_, update_period_);
                setPIDParameters(pid_controller_front_right, pid_p_, pid_i_, pid_d_, max_effort_pid_, update_period_);             

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

                UpdateOdometryEncoder();

#if GAZEBO_MAJOR_VERSION >= 8
                common::Time current_time = parent->GetWorld()->SimTime();
#else
                common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
                double seconds_since_last_update =
                    (current_time - last_update_time_).Double();

                if (seconds_since_last_update > update_period_)
                {
                        PublishOdometry();
                }

                if (cmd_twist_)
                {
                        double curve_radius =
                            wheelbase_distance_ / tan(max_steering_angle_ * cmd_twist_->angular.z);
                        double angular_velocity = cmd_twist_->linear.x / curve_radius;

                        ROS_INFO_NAMED("WheelsSteerable", "Calculated angular velocity %lf",
                                       angular_velocity);
                        ROS_INFO_NAMED("WheelsSteerable", "Calculated curve radius %lf",
                                       curve_radius);

                        double rear_right_velocity = 0;
                        double rear_left_velocity = 0;
                        if (isinf(curve_radius))
                        {
                                rear_right_velocity = cmd_twist_->linear.x;
                                rear_left_velocity = -cmd_twist_->linear.x;
                        }
                        else
                        {
                                rear_right_velocity =
                                    angular_velocity * (curve_radius - (kingpin_distance_ / 2));
                                rear_left_velocity =
                                    -angular_velocity * (curve_radius + (kingpin_distance_ / 2));
                        }

                        double front_right_angle =
                            atan(wheelbase_distance_ / (curve_radius - (kingpin_distance_ / 2)));
                        double front_left_angle =
                            atan(wheelbase_distance_ / (curve_radius + (kingpin_distance_ / 2)));



                        joints_rotation_[REAR_LEFT]->SetParam("vel", 0, rear_left_velocity);
                        joints_rotation_[REAR_RIGHT]->SetParam("vel", 0, rear_right_velocity);

                        std_msgs::Float64 targetPosition;
                        targetPosition.data = front_left_angle;
                        target_position_publisher_.publish(targetPosition);

                        double currentPosFrontLeft = joints_rotation_[FRONT_LEFT]->Position(0);
                        double front_left_force = calculatePID(pid_controller_front_left, front_left_angle, currentPosFrontLeft);
                        joints_rotation_[FRONT_LEFT]->SetForce(0, front_left_force);

                        double currentPosFrontRight = joints_rotation_[FRONT_RIGHT]->Position(0);
                        double front_right_force = calculatePID(pid_controller_front_right, -front_right_angle ,currentPosFrontRight);
                        joints_rotation_[FRONT_RIGHT]->SetForce(0, front_right_force);

                        std_msgs::Float64 currentPosistion;
                        currentPosistion.data = currentPosFrontLeft;
                        current_position_publisher_.publish(currentPosistion);

                        ROS_INFO_NAMED("WheelsSteerable", "Scoped name: %s force: %lf", joints_rotation_[FRONT_LEFT]->GetScopedName().c_str(), joints_rotation_[FRONT_LEFT]->GetForce(0));

                        std_msgs::Float64 appliedForce;
                        appliedForce.data = front_left_force;
                        applied_force_publisher_.publish(appliedForce);
                }

#ifdef ENABLE_PROFILER
                IGN_PROFILE_END();
#endif
        }

        void GazeboRosWheelsSteerable::UpdateOdometryEncoder()
        {
                // Get angular velocities of rear joints
                double vl = (-1) * joints_rotation_[REAR_LEFT]->GetVelocity(0);
                double vr = joints_rotation_[REAR_RIGHT]->GetVelocity(0);

#if GAZEBO_MAJOR_VERSION >= 8
                common::Time current_time = parent->GetWorld()->SimTime();
#else
                common::Time current_time = parent->GetWorld()->GetSimTime();
#endif

                ROS_INFO_NAMED("WheelsSteerable", "Get rear left wheel velocity %lf", vl);

                double seconds_since_last_update =
                    (current_time - last_odom_update_).Double();
                last_odom_update_ = current_time;

                double b = kingpin_distance_;

                // Book: Sigwart 2011 Autonompus Mobile Robots page:337
                // Calculate left and right wheel distance
                double sl = vl * wheel_radius_ * seconds_since_last_update;
                double sr = vr * wheel_radius_ * seconds_since_last_update;
                double ssum = sl + sr;

                double sdiff = sr - sl;

                double dx = (ssum) / 2.0 * cos(pose_encoder_.theta + (sdiff) / (2.0 * b));
                double dy = (ssum) / 2.0 * sin(pose_encoder_.theta + (sdiff) / (2.0 * b));
                double dtheta = (sdiff) / b;

                pose_encoder_.x += dx;
                pose_encoder_.y += dy;
                pose_encoder_.theta += dtheta;

                double w = dtheta / seconds_since_last_update;
                double v = sqrt(dx * dx + dy * dy) / seconds_since_last_update;

                tf::Quaternion qt;
                tf::Vector3 vt;
                qt.setRPY(0, 0, pose_encoder_.theta);
                vt = tf::Vector3(pose_encoder_.x, pose_encoder_.y, 0);

                odom_.pose.pose.position.x = vt.x();
                //correct orientation of y coordinate
                odom_.pose.pose.position.y = (-1) * vt.y();
                odom_.pose.pose.position.z = vt.z();

                odom_.pose.pose.orientation.x = qt.x();
                odom_.pose.pose.orientation.y = qt.y();
                odom_.pose.pose.orientation.z = qt.z();
                odom_.pose.pose.orientation.w = qt.w();

                odom_.twist.twist.angular.z = w;
                odom_.twist.twist.linear.x = v;
                odom_.twist.twist.linear.y = 0;
        }

        void GazeboRosWheelsSteerable::GetGroundTruth()
        {
// getting data from gazebo world
#if GAZEBO_MAJOR_VERSION >= 8
                ignition::math::Pose3d pose = parent->WorldPose();
#else
                ignition::math::Pose3d pose = parent->GetWorldPose().Ign();
#endif

                tf::Quaternion qt;
                tf::Vector3 vt;
                qt = tf::Quaternion(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(),
                                    pose.Rot().W());
                vt = tf::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

                odom_.pose.pose.position.x = vt.x();
                odom_.pose.pose.position.y = vt.y();
                odom_.pose.pose.position.z = vt.z();

                odom_.pose.pose.orientation.x = qt.x();
                odom_.pose.pose.orientation.y = qt.y();
                odom_.pose.pose.orientation.z = qt.z();
                odom_.pose.pose.orientation.w = qt.w();

                // get velocity in /odom frame
                ignition::math::Vector3d linear;

#if GAZEBO_MAJOR_VERSION >= 8
                linear = parent->WorldLinearVel();
                odom_.twist.twist.angular.z = parent->WorldAngularVel().Z();
#else
                linear = parent->GetWorldLinearVel().Ign();
                odom_.twist.twist.angular.z = parent->GetWorldAngularVel().Ign().Z();
#endif

                // convert velocity to child_frame_id (aka base_footprint)
                float yaw = pose.Rot().Yaw();
                odom_.twist.twist.linear.x =
                    cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
                odom_.twist.twist.linear.y =
                    cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
        }

        void GazeboRosWheelsSteerable::SetCovariance()
        {
                // set covariance
                odom_.pose.covariance[0] = 0.00001;
                odom_.pose.covariance[7] = 0.00001;
                odom_.pose.covariance[14] = 10000000.0;
                odom_.pose.covariance[21] = 10000000.0;
                odom_.pose.covariance[28] = 10000000.0;
                odom_.pose.covariance[35] = 0.001;
        }

        void GazeboRosWheelsSteerable::PublishOdometry()
        {

                ros::Time current_time = ros::Time::now();
                std::string odom_frame = gazebo_ros_->resolveTF(frame_odom_);
                std::string base_footprint_frame = gazebo_ros_->resolveTF(frame_base_);

                tf::Quaternion qt;
                tf::Vector3 vt;

                // getting data form encoder integration
                qt = tf::Quaternion(
                    odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y,
                    odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w);
                vt = tf::Vector3(odom_.pose.pose.position.x, odom_.pose.pose.position.y,
                                 odom_.pose.pose.position.z);

                SetCovariance();

                // set header
                odom_.header.stamp = current_time;
                odom_.header.frame_id = odom_frame;
                odom_.child_frame_id = base_footprint_frame;

                odometry_publisher_.publish(odom_);

                GetGroundTruth();

                // set header
                odom_.header.stamp = current_time;
                odom_.header.frame_id = frame_ground_truth_;
                odom_.child_frame_id = base_footprint_frame;

                ground_truth_odometry_publisher_.publish(odom_);
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

                while (alive_ && gazebo_ros_->node()->ok())
                {
                        queue_.callAvailable(ros::WallDuration(timeout));
                }
        }

        void GazeboRosWheelsSteerable::callbackTopicCMD(
            const geometry_msgs::Twist::ConstPtr &cmd_msg)
        {
                cmd_twist_ = cmd_msg;
        }

        void GazeboRosWheelsSteerable::callbackTopicPID(
            const control_msgs::JointControllerState::ConstPtr &pid_msg)
        {
                setPIDParameters(pid_controller_front_left, pid_msg->p, pid_msg->i, pid_msg->d, pid_msg->i_clamp, update_period_);
                setPIDParameters(pid_controller_front_right, pid_msg->p, pid_msg->i, pid_msg->d, pid_msg->i_clamp, update_period_);

                ROS_INFO_NAMED("WheelsSteerable", "Set P to %lf, I to %lf, D to %lf", pid_msg->p, pid_msg->i, pid_msg->d);
        }

        // see https://gist.github.com/bradley219/5373998
        double GazeboRosWheelsSteerable::calculatePID(PID_Controller_State state, double setValue, double currentValue )
        {
            
            // Calculate error
            double error = setValue - currentValue;

            // Proportional term
            double Pout = state.pid_p_ * error;

            // Integral term
            state._integral += error * state.dt;
            double Iout = state.pid_i_ * state._integral;

            // Derivative term
            double derivative = (error - state._pre_error) / state.dt;
            double Dout = state.pid_d_ * derivative;

            // Calculate total output
            double output = Pout + Iout + Dout;

            // Restrict to max/min
            if( output > state.max_effort_pid_ )
                output = state.max_effort_pid_;
            else if( output < (-state.max_effort_pid_))
                output = (-state.max_effort_pid_);

            // Save error to previous error
            state._pre_error = error;

            return output;
        }

        void GazeboRosWheelsSteerable::setPIDParameters(PID_Controller_State &state, double p, double i, double d, double maxEffort, double dt)
        {
            pid_controller_front_left.max_effort_pid_= maxEffort;
            pid_controller_front_left.pid_p_= p;
            pid_controller_front_left.pid_i_= i;
            pid_controller_front_left.pid_d_= d;
            pid_controller_front_left.dt = dt;
            pid_controller_front_left._integral = 0;
            pid_controller_front_left._pre_error = 0;
        }

        GZ_REGISTER_MODEL_PLUGIN(GazeboRosWheelsSteerable)
} // namespace gazebo
