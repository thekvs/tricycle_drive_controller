/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
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
 *   * Neither the name of the PAL Robotics nor the names of its
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

/*
 * Author: Bence Magyar
 */

#include <cmath>

#include <tf/transform_datatypes.h>

#include <urdf_parser/urdf_parser.h>

#include <boost/assign.hpp>

#include <tricycle_drive_controller/tricycle_drive_controller.h>

static double euclideanOfVectors(const urdf::Vector3& vec1, const urdf::Vector3& vec2)
{
  return std::sqrt(std::pow(vec1.x - vec2.x, 2) + std::pow(vec1.y - vec2.y, 2) + std::pow(vec1.z - vec2.z, 2));
}

/*
 * \brief Check if the link is modeled as a cylinder
 * \param link Link
 * \return true if the link is modeled as a Cylinder; false otherwise
 */
static bool isCylinder(const boost::shared_ptr<const urdf::Link>& link)
{
  if (!link)
  {
    ROS_ERROR("Link == NULL.");
    return false;
  }

  if (!link->collision)
  {
    ROS_ERROR_STREAM(
        "Link " << link->name << " does not have collision description. Add collision description for link to urdf.");
    return false;
  }

  if (!link->collision->geometry)
  {
    ROS_ERROR_STREAM(
        "Link " << link->name << " does not have collision geometry description. Add collision geometry description for link to urdf.");
    return false;
  }

  if (link->collision->geometry->type != urdf::Geometry::CYLINDER)
  {
    ROS_ERROR_STREAM("Link " << link->name << " does not have cylinder geometry");
    return false;
  }

  return true;
}

/*
 * \brief Get the wheel radius
 * \param [in]  wheel_link   Wheel link
 * \param [out] wheel_radius Wheel radius [m]
 * \return true if the wheel radius was found; false otherwise
 */
static bool getWheelRadius(const boost::shared_ptr<const urdf::Link>& wheel_link, double& wheel_radius)
{
  if (!isCylinder(wheel_link))
  {
    ROS_ERROR_STREAM("Wheel link " << wheel_link->name << " is NOT modeled as a cylinder!");
    return false;
  }

  wheel_radius = (static_cast<urdf::Cylinder*>(wheel_link->collision->geometry.get()))->radius;
  return true;
}

namespace tricycle_drive_controller
{

TricycleDriveController::TricycleDriveController() :
    open_loop_(false),
    real_hw_(false),
    command_struct_(),
    wheel_base_(0.0),
    wheel_radius_(0.0),
    front_wheel_offset_to_center_(0.0),
    front_wheel_steering_angle_offset_(0.0),
    front_wheel_steering_angle_speed_multiplier_(1.0),
    wheel_separation_multiplier_(1.0),
    wheel_radius_multiplier_(1.0),
    wheel_radius_multiplier_odom_(4.0),
    ackermann_cmd_timeout_(0.5),
    base_frame_id_("base_link"),
    enable_odom_tf_(true),
    wheel_joints_size_(0),
    odom_topic_("/odom")
{
}

bool TricycleDriveController::init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh,
                                   ros::NodeHandle& controller_nh)
{
  typedef std::vector<std::string>::const_iterator NamesIterator;

  const std::string complete_ns = controller_nh.getNamespace();
  std::size_t id = complete_ns.find_last_of("/");
  name_ = complete_ns.substr(id + 1);

  // Get joint names from the parameter server
  std::vector<std::string> front_wheel_names, front_wheel_caster_names;

  if (!getWheelNames(controller_nh, "front_wheel", front_wheel_names))
  {
    return false;
  }
  if (!getWheelNames(controller_nh, "front_wheel_caster", front_wheel_caster_names))
  {
    return false;
  }

  if (front_wheel_names.size() != front_wheel_caster_names.size())
  {
    ROS_ERROR_STREAM_NAMED(
        name_,
        "#front wheels (" << front_wheel_names.size() << ") != " << "#front wheel casters (" << front_wheel_caster_names.size() << ").");
    return false;
  }
  else
  {
    wheel_joints_size_ = front_wheel_names.size();

    // front_wheel_cmd.resize(wheel_joints_size_);
    // front_wheel_caster_cmd.resize(wheel_joints_size_);
  }

  // Odometry related:
  double publish_rate;
  controller_nh.param("publish_rate", publish_rate, 50.0);
  ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at " << publish_rate << "Hz.");
  publish_period_ = ros::Duration(1.0 / publish_rate);

  controller_nh.param("open_loop", open_loop_, false);
  ROS_INFO_STREAM_NAMED(name_, "open_loop parameter set to " << open_loop_);

  controller_nh.param("real_hw", real_hw_, false);
  ROS_INFO_STREAM_NAMED(name_, "real_hw parameter set to " << real_hw_);

  if (open_loop_ && real_hw_)
  {
    ROS_ERROR_STREAM("parameters 'open_loop' and 'real_hw' can't be true at the same moment");
    return false;
  }

  controller_nh.param("wheel_separation_multiplier", wheel_separation_multiplier_, wheel_separation_multiplier_);
  ROS_INFO_STREAM_NAMED(name_, "Wheel separation will be multiplied by " << wheel_separation_multiplier_ << ".");

  controller_nh.param("odom_topic", odom_topic_, odom_topic_);
  ROS_INFO_STREAM_NAMED(name_, "Odom topic set to" << odom_topic_ << ".");

  controller_nh.param("wheel_radius_multiplier", wheel_radius_multiplier_, wheel_radius_multiplier_);
  ROS_INFO_STREAM_NAMED(name_, "Wheel radius will be multiplied by " << wheel_radius_multiplier_ << ".");

  controller_nh.param("wheel_radius_multiplier_odom", wheel_radius_multiplier_odom_, wheel_radius_multiplier_odom_);
  ROS_INFO_STREAM_NAMED(name_, "Wheel radius will be multiplied by " << wheel_radius_multiplier_odom_ << ".");

  int velocity_rolling_window_size = 10;
  controller_nh.param("velocity_rolling_window_size", velocity_rolling_window_size, velocity_rolling_window_size);
  ROS_INFO_STREAM_NAMED(name_, "Velocity rolling window size of " << velocity_rolling_window_size << ".");

  odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

  // Twist command related:
  controller_nh.param("ackermann_cmd_timeout", ackermann_cmd_timeout_, ackermann_cmd_timeout_);
  ROS_INFO_STREAM_NAMED(
      name_, "Velocity commands will be considered old if they are older than " << ackermann_cmd_timeout_ << "s.");

  controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
  ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

  controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
  ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_ ? "enabled" : "disabled"));

  // Velocity and acceleration limits:
  controller_nh.param("linear/x/has_velocity_limits", limiter_lin_.has_velocity_limits,
                      limiter_lin_.has_velocity_limits);
  controller_nh.param("linear/x/has_acceleration_limits", limiter_lin_.has_acceleration_limits,
                      limiter_lin_.has_acceleration_limits);
  controller_nh.param("linear/x/has_jerk_limits", limiter_lin_.has_jerk_limits, limiter_lin_.has_jerk_limits);
  controller_nh.param("linear/x/max_velocity", limiter_lin_.max_velocity, limiter_lin_.max_velocity);
  controller_nh.param("linear/x/min_velocity", limiter_lin_.min_velocity, -limiter_lin_.max_velocity);
  controller_nh.param("linear/x/max_acceleration", limiter_lin_.max_acceleration, limiter_lin_.max_acceleration);
  controller_nh.param("linear/x/min_acceleration", limiter_lin_.min_acceleration, -limiter_lin_.max_acceleration);
  controller_nh.param("linear/x/max_jerk", limiter_lin_.max_jerk, limiter_lin_.max_jerk);
  controller_nh.param("linear/x/min_jerk", limiter_lin_.min_jerk, -limiter_lin_.max_jerk);

  controller_nh.param("angular/z/has_velocity_limits", limiter_ang_.has_velocity_limits,
                      limiter_ang_.has_velocity_limits);
  controller_nh.param("angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits,
                      limiter_ang_.has_acceleration_limits);
  controller_nh.param("angular/z/has_jerk_limits", limiter_ang_.has_jerk_limits, limiter_ang_.has_jerk_limits);
  controller_nh.param("angular/z/max_velocity", limiter_ang_.max_velocity, limiter_ang_.max_velocity);
  controller_nh.param("angular/z/min_velocity", limiter_ang_.min_velocity, -limiter_ang_.max_velocity);
  controller_nh.param("angular/z/max_acceleration", limiter_ang_.max_acceleration, limiter_ang_.max_acceleration);
  controller_nh.param("angular/z/min_acceleration", limiter_ang_.min_acceleration, -limiter_ang_.max_acceleration);
  controller_nh.param("angular/z/max_jerk", limiter_ang_.max_jerk, limiter_ang_.max_jerk);
  controller_nh.param("angular/z/min_jerk", limiter_ang_.min_jerk, -limiter_ang_.max_jerk);

  // If either parameter is not available, we need to look up the value in the URDF
  bool lookup_wheel_radius = !controller_nh.getParam("wheel_radius", wheel_radius_);
  bool wheel_base_exists = controller_nh.getParam("wheel_base", wheel_base_);
  controller_nh.getParam("front_wheel_offset_to_center", front_wheel_offset_to_center_);
  controller_nh.getParam("front_wheel_steering_angle_offset", front_wheel_steering_angle_offset_);
  controller_nh.getParam("front_wheel_steering_angle_speed_multiplier", front_wheel_steering_angle_speed_multiplier_);

  if (!wheel_base_exists)
  {
    ROS_ERROR_STREAM("Obligatory wheel_base configuration parameter is not provided");
    return false;
  }

  if (!setOdomParamsFromUrdf(root_nh, front_wheel_names[0], lookup_wheel_radius))
  {
    return false;
  }

  // Regardless of how we got radius, use it to set the odometry parameters
  const double wr = wheel_radius_multiplier_odom_ * wheel_radius_;
  odometry_.setWheelParams(wheel_base_, wr, front_wheel_offset_to_center_);

  ROS_INFO_STREAM_NAMED(name_, "Odometry params : wheel radius " << wr << ", wheel base " << wheel_base_);

  setOdomPubFields(root_nh, controller_nh);

  hardware_interface::VelocityJointInterface* vel_iface = hw->get<hardware_interface::VelocityJointInterface>();
  hardware_interface::PositionJointInterface* pos_iface = hw->get<hardware_interface::PositionJointInterface>();

  if (vel_iface)
  {
    for (NamesIterator it = front_wheel_names.begin(); it != front_wheel_names.end(); ++it)
    {
      front_wheel_cmd.push_back(vel_iface->getHandle(*it));
    }
  }

  if (pos_iface)
  {
    for (NamesIterator it = front_wheel_caster_names.begin(); it != front_wheel_caster_names.end(); ++it)
    {
      front_wheel_caster_cmd.push_back(pos_iface->getHandle(*it));
    }
  }

  sub_command_ = controller_nh.subscribe("/cmd_vel", 1, &TricycleDriveController::cmdVelCallback, this);

  return true;
}

void TricycleDriveController::update(const ros::Time& time, const ros::Duration& period)
{
  // COMPUTE AND PUBLISH ODOMETRY
  if (real_hw_)
  {
    double front_wheel_velocity = front_wheel_cmd.front().getVelocity();
    double front_wheel_steering_angle = front_wheel_caster_cmd.front().getPosition()
        + front_wheel_steering_angle_offset_;

    odometry_.updateFromHWEncoders(front_wheel_velocity, front_wheel_steering_angle, time);
  }
  else if (open_loop_)
  {
    odometry_.updateOpenLoop(last0_cmd_.speed, last0_cmd_.angle, time);
  }
  else
  {
    double front_wheel_pos = 0.0;
    double front_wheel_caster_pos = 0.0;
    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      const double fp = front_wheel_cmd[i].getPosition();
      const double cp = front_wheel_caster_cmd[i].getPosition();
      if (std::isnan(fp) || std::isnan(cp))
        return;

      front_wheel_pos += fp;
      front_wheel_caster_pos += cp;
    }
    front_wheel_pos /= wheel_joints_size_;
    front_wheel_caster_pos /= wheel_joints_size_;

    // Estimate linear and angular velocity using joint information
    odometry_.update(front_wheel_pos, front_wheel_caster_pos, time);
  }

  // Publish odometry message
  if (last_state_publish_time_ + publish_period_ < time)
  {
    last_state_publish_time_ += publish_period_;
    // Compute and store orientation info
    const geometry_msgs::Quaternion orientation(tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

    // Populate odom message and publish
    if (odom_pub_->trylock())
    {
      odom_pub_->msg_.header.stamp = time;
      odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
      odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
      odom_pub_->msg_.pose.pose.orientation = orientation;
      odom_pub_->msg_.twist.twist.linear.x = odometry_.getLinear();
      odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
      odom_pub_->unlockAndPublish();
    }

    // Publish tf /odom frame
    if (enable_odom_tf_ && tf_odom_pub_->trylock())
    {
      geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
      odom_frame.header.stamp = time;
      odom_frame.transform.translation.x = odometry_.getX();
      odom_frame.transform.translation.y = odometry_.getY();
      odom_frame.transform.rotation = orientation;
      tf_odom_pub_->unlockAndPublish();
    }
  }

  // MOVE ROBOT
  // Retreive current velocity command and time step:
  Commands curr_cmd = *(command_.readFromRT());
  const ros::Duration dt = time - curr_cmd.stamp;

  // Brake if cmd_vel has timeout:
  if (dt.toSec() > ackermann_cmd_timeout_)
  {
    curr_cmd.speed = 0.0;
  }

  // Limit velocities and accelerations:
  const double cmd_dt(period.toSec());

  limiter_lin_.limit(curr_cmd.speed, last0_cmd_.speed, last1_cmd_.speed, cmd_dt);
  limiter_ang_.limit(curr_cmd.angle, last0_cmd_.angle, last1_cmd_.angle, cmd_dt);

  last1_cmd_ = last0_cmd_;
  last0_cmd_ = curr_cmd;

  // Apply multiplier:
  const double wr = wheel_radius_multiplier_ * wheel_radius_;

  double front_steering_angle = front_wheel_caster_cmd.front().getPosition() + front_wheel_steering_angle_offset_;
  double diff_angles_cmd_odom = std::abs(curr_cmd.angle - front_steering_angle);
  double multiplier_wheel_speed = std::min(1.0 / (front_wheel_steering_angle_speed_multiplier_ * diff_angles_cmd_odom/M_PI) , 1.0);

  ROS_DEBUG_STREAM("diff_angles_cmd_odom: " << diff_angles_cmd_odom);
  ROS_DEBUG_STREAM("multiplier_wheel_speed: " << multiplier_wheel_speed);

  // Set wheel velocity and steering angle:
  for (size_t i = 0; i < wheel_joints_size_; ++i)
  {
    front_wheel_cmd[i].setCommand((curr_cmd.speed / wr) * multiplier_wheel_speed);
    front_wheel_caster_cmd[i].setCommand(curr_cmd.angle);
  }
}

void TricycleDriveController::starting(const ros::Time& time)
{
  brake();

  // Register starting time used to keep fixed rate
  last_state_publish_time_ = time;

  odometry_.init(time);
}

void TricycleDriveController::stopping(const ros::Time& /*time*/)
{
  brake();
}

void TricycleDriveController::brake()
{
  const double vel = 0.0;
  for (size_t i = 0; i < wheel_joints_size_; ++i)
  {
    front_wheel_cmd[i].setCommand(vel);
    front_wheel_caster_cmd[i].setCommand(vel);
  }
}

void TricycleDriveController::cmdVelCallback(const geometry_msgs::Twist& command)
{
  if (isRunning())
  {
    // convert vehicle velocity to wheel velocity
    if (command.angular.z == 0)
    {
      command_struct_.speed = command.linear.x;
      command_struct_.angle = 0;
    }
    else
    {

      double radius = command.linear.x / command.angular.z;

      if (radius == 0)
      {
        command_struct_.angle = M_PI_2;
        command_struct_.angle = boost::math::copysign(command_struct_.angle, command.angular.z);
      }
      else {
        radius = boost::math::copysign(radius, command.angular.z);
        command_struct_.angle = std::atan(wheel_base_ / radius);
      }

      if (std::fabs(command_struct_.angle) > 1.55)
      {
        command_struct_.speed = boost::math::copysign(wheel_base_ * command.angular.z, command.linear.x);

      }
      else
      {
        command_struct_.speed = command.linear.x / std::cos(command_struct_.angle);
      }

    }

    if (command_struct_.speed < 0)
    {
      command_struct_.angle = -command_struct_.angle;
    }

    command_struct_.angle = command_struct_.angle - front_wheel_steering_angle_offset_;

    command_struct_.stamp = ros::Time::now();
    command_.writeFromNonRT(command_struct_);
    ROS_DEBUG_STREAM_NAMED(
        name_,
        "Added values to command. " << "Ang: " << command_struct_.angle << ", " << "Speed: " << command_struct_.speed << ", " << "Stamp: " << command_struct_.stamp);
  }
  else
  {
    ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
  }
}

bool TricycleDriveController::getWheelNames(ros::NodeHandle& controller_nh, const std::string& wheel_param,
                                            std::vector<std::string>& wheel_names)
{
  XmlRpc::XmlRpcValue wheel_list;
  if (!controller_nh.getParam(wheel_param, wheel_list))
  {
    ROS_ERROR_STREAM_NAMED(name_, "Couldn't retrieve wheel param '" << wheel_param << "'.");
    return false;
  }

  if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (wheel_list.size() == 0)
    {
      ROS_ERROR_STREAM_NAMED(name_, "Wheel param '" << wheel_param << "' is an empty list");
      return false;
    }

    for (int i = 0; i < wheel_list.size(); ++i)
    {
      if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
      {
        ROS_ERROR_STREAM_NAMED(name_, "Wheel param '" << wheel_param << "' #" << i << " isn't a string.");
        return false;
      }
    }

    wheel_names.resize(wheel_list.size());
    for (int i = 0; i < wheel_list.size(); ++i)
    {
      wheel_names[i] = static_cast<std::string>(wheel_list[i]);
    }
  }
  else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString)
  {
    wheel_names.push_back(wheel_list);
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(name_, "Wheel param '" << wheel_param << "' is neither a list of strings nor a string.");
    return false;
  }

  return true;
}

bool TricycleDriveController::setOdomParamsFromUrdf(ros::NodeHandle& root_nh, const std::string& front_wheel_name,
                                                    bool lookup_wheel_radius)
{
  if (!lookup_wheel_radius)
  {
    // Short-circuit in case we don't need to look up anything, so we don't have to parse the URDF
    return true;
  }

  // Parse robot description
  const std::string model_param_name = "robot_description";
  bool res = root_nh.hasParam(model_param_name);
  std::string robot_model_str = "";
  if (!res || !root_nh.getParam(model_param_name, robot_model_str))
  {
    ROS_ERROR_NAMED(name_, "Robot descripion couldn't be retrieved from param server.");
    return false;
  }

  boost::shared_ptr<urdf::ModelInterface> model(urdf::parseURDF(robot_model_str));
  boost::shared_ptr<const urdf::Joint> front_wheel_joint(model->getJoint(front_wheel_name));

  if (lookup_wheel_radius)
  {
    // Get wheel radius
    if (!getWheelRadius(model->getLink(front_wheel_joint->child_link_name), wheel_radius_))
    {
      ROS_ERROR_STREAM_NAMED(name_, "Couldn't retrieve " << front_wheel_name << " wheel radius");
      return false;
    }
  }

  return true;
}

void TricycleDriveController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // Get and check params for covariances
  XmlRpc::XmlRpcValue pose_cov_list;
  controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
  ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(pose_cov_list.size() == 6);
  for (int i = 0; i < pose_cov_list.size(); ++i)
    ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  XmlRpc::XmlRpcValue twist_cov_list;
  controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
  ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(twist_cov_list.size() == 6);
  for (int i = 0; i < twist_cov_list.size(); ++i)
    ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  // clang-format off
  // Setup odometry realtime publisher + odom message constant fields
  odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, odom_topic_, 100));
  odom_pub_->msg_.header.frame_id = "odom";
  odom_pub_->msg_.child_frame_id = base_frame_id_;
  odom_pub_->msg_.pose.pose.position.z = 0;
  odom_pub_->msg_.pose.covariance = boost::assign::list_of(static_cast<double>(pose_cov_list[0]))(0)(0)(0)(0)(0)(0)(
      static_cast<double>(pose_cov_list[1]))(0)(0)(0)(0)(0)(0)(static_cast<double>(pose_cov_list[2]))(0)(0)(0)(0)(0)(0)(
      static_cast<double>(pose_cov_list[3]))(0)(0)(0)(0)(0)(0)(static_cast<double>(pose_cov_list[4]))(0)(0)(0)(0)(0)(0)(
      static_cast<double>(pose_cov_list[5]));
  odom_pub_->msg_.twist.twist.linear.y = 0;
  odom_pub_->msg_.twist.twist.linear.z = 0;
  odom_pub_->msg_.twist.twist.angular.x = 0;
  odom_pub_->msg_.twist.twist.angular.y = 0;
  odom_pub_->msg_.twist.covariance = boost::assign::list_of(static_cast<double>(twist_cov_list[0]))(0)(0)(0)(0)(0)(0)(
      static_cast<double>(twist_cov_list[1]))(0)(0)(0)(0)(0)(0)(static_cast<double>(twist_cov_list[2]))(0)(0)(0)(0)(0)(
      0)(static_cast<double>(twist_cov_list[3]))(0)(0)(0)(0)(0)(0)(static_cast<double>(twist_cov_list[4]))(0)(0)(0)(0)(
      0)(0)(static_cast<double>(twist_cov_list[5]));
  tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
  tf_odom_pub_->msg_.transforms.resize(1);
  tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
  tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
  tf_odom_pub_->msg_.transforms[0].header.frame_id = "odom";
  // clang-format on
}

} // namespace tricycle_drive_controller
