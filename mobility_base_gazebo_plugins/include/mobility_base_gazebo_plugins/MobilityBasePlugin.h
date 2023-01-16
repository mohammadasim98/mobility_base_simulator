/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2017, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef MOBILITY_BASE_PLUGIN_H_
#define MOBILITY_BASE_PLUGIN_H_

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_plugins/PubQueue.h>

// ROS messages
#include <mobility_base_core_msgs/BumperState.h>
#include <mobility_base_core_msgs/Mode.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

// Boost
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

#ifdef _CONST
#undef _CONST
#endif
#if __cplusplus >= 201103L
#define _CONST constexpr
#else
#define _CONST const
#endif

namespace gazebo
{
class MobilityBasePlugin : public ModelPlugin
{
public:
  MobilityBasePlugin();
  ~MobilityBasePlugin();

  static _CONST unsigned int NUM_WHEELS = 4;
  static _CONST unsigned int NUM_ROLLERS = 12;
  static _CONST double WHEEL_RADIUS = 8.0 / 2 * 0.0254;
  static _CONST double WHEEL_BASE_WIDTH = 0.680466;
  static _CONST double WHEEL_BASE_LENGTH = 0.505466;
  static _CONST double RADIANS_PER_SECOND_MAX = 14000 * (2.0 * M_PI / (200 * 48.5 / 2));
  static _CONST double CMD_TIMEOUT = 0.2;
  static _CONST double ACCEL_LIMIT_FAST_VXY = 4.0;
  static _CONST double ACCEL_LIMIT_FAST_WZ = 3.0 * M_PI;
  static _CONST double ACCEL_LIMIT_SLOW_VXY = 1.0;
  static _CONST double ACCEL_LIMIT_SLOW_WZ = 1.0 * M_PI;
  static _CONST double ACCEL_INSTANT_VXY = 0.5;
  static _CONST double ACCEL_INSTANT_WZ = 0.5 * M_PI;
  static _CONST double TORQUE_MAX_GLOBAL = 50.0;
  static _CONST double TORQUE_MAX_ALARM = 5.0;

  static _CONST double PUB_FREQ_VEHICLE = 250.0;
  static _CONST double PUB_FREQ_IMU = 100.0;
  static _CONST double PUB_FREQ_JOYSTICK = 50.0;
  static _CONST double PUB_FREQ_BUMPERS = 10.0;
  static _CONST double PUB_FREQ_MODE = 2.0;

  static _CONST double GAIN_X = 1.0;
  static _CONST double GAIN_Y = 1.0;
  static _CONST double GAIN_Z = 1.0;

  void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf);

protected:
  virtual void UpdateChild(const common::UpdateInfo & _info);
  virtual void FiniChild();

private:
  // ROS callbacks
  void recvCmdVel(const geometry_msgs::Twist::ConstPtr& msg);
  void recvCmdVelRaw(const geometry_msgs::Twist::ConstPtr& msg);
  void recvSuppressWireless(const std_msgs::Empty::ConstPtr& msg);

  void publishMode(const ros::Time &stamp);

  // Gazebo
  event::ConnectionPtr update_connection_;
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::JointPtr joint_wheels_[NUM_WHEELS];
  physics::JointPtr joint_rollers_[NUM_WHEELS][NUM_ROLLERS];
  physics::LinkPtr link_base_footprint_;
  common::Time previous_stamp_;
  bool first_update_;

  // Command source cmd_vel
  math::Vector3 cmd_vel_;
  common::Time cmd_vel_stamp_;
  boost::mutex cmd_vel_mutex_;

  // Command source cmd_vel_raw
  math::Vector3 cmd_vel_raw_;
  common::Time cmd_vel_raw_stamp_;
  boost::mutex cmd_vel_raw_mutex_;

  // Drive control
  common::Time stamp_vehicle_;
  common::Time stamp_imu_;
  common::Time stamp_joystick_;
  common::Time stamp_bumpers_;
  common::Time stamp_mode_;
  math::Vector3 cmd_vel_history_;
  mobility_base_core_msgs::Mode::_mode_type mode_;

  // ROS
  ros::NodeHandle *nh_;
  tf::TransformBroadcaster *tf_broadcaster_;
  sensor_msgs::JointState joint_state_wheels_;
  sensor_msgs::JointState joint_state_rollers_;

  // ROS spin thread
  void spin();
  boost::thread *spinner_thread_;

  // Subscribed topics
  ros::Subscriber sub_cmd_vel_;
  ros::Subscriber sub_cmd_vel_raw_;

  // Published topics
  ros::Publisher pub_imu_data_;
  ros::Publisher pub_imu_is_calibrated_;
  ros::Publisher pub_imu_mag_;
  ros::Publisher pub_twist_;
  ros::Publisher pub_joystick_;
  ros::Publisher pub_wrench_;
  ros::Publisher pub_joint_states_;
  ros::Publisher pub_bumper_states_;
  ros::Publisher pub_mode_;

  // Publisher Queues
  PubMultiQueue pmq_;
  PubQueue<sensor_msgs::Imu>::Ptr pmq_imu_data_;
  PubQueue<std_msgs::Bool>::Ptr pmq_imu_is_calibrated_;
  PubQueue<sensor_msgs::MagneticField>::Ptr pmq_imu_mag_;
  PubQueue<geometry_msgs::TwistStamped>::Ptr pmq_twist_;
  PubQueue<geometry_msgs::TwistStamped>::Ptr pmq_joystick_;
  PubQueue<geometry_msgs::WrenchStamped>::Ptr pmq_wrench_;
  PubQueue<sensor_msgs::JointState>::Ptr pmq_joint_states_;
  PubQueue<mobility_base_core_msgs::BumperState>::Ptr pmq_bumper_states_;
  PubQueue<mobility_base_core_msgs::Mode>::Ptr pmq_mode_;

  // Calculated constant parameters
  double omni_a_;
  double omni_b_;
  double omni_c_;
  const char *frame_id_;

  // Parameters
  bool fast_;
  std::string parent_frame_id_;
  std::string child_frame_id_;


  void omniFromCartesian(double vx, double vy, double wz, double w[4]) const;
  void omniToCartesian(const double w[4], double *vx, double *vy, double *wz) const;
  static inline bool omniSaturate(double limit, double speeds[4])
  {
      unsigned int i;
      limit = fabs(limit);
      double max = 0;
      double val;
      for (i = 0; i < 4; i++) {
          val = fabs(speeds[i]);
          if (val > max) {
              max = val;
          }
      }
      if (max > limit) {
          val = limit / max;
          for (i = 0; i < 4; i++) {
              speeds[i] *= val;
          }
          return true;
      }
      return false;
  }
  static inline double limitDelta(double input, double previous, double limit)
  {
    limit = fabs(limit);
    double delta = input - previous;
    if (delta > limit) {
        delta = limit;
    }
    if (delta < -limit) {
        delta = -limit;
    }
    return previous + delta;
  }

};
} //namespace gazebo

#undef _CONST

#endif /* MOBILITY_BASE_PLUGIN_H_ */
