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
 * Author: Enrique Fernández
 */

#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <tricycle_drive_controller/odometry.h>
#include <tricycle_drive_controller/speed_limiter.h>


namespace tricycle_drive_controller
{

/**
 * This class makes some assumptions on the model of the robot:
 *  - the rotation axes of wheels are collinear
 *  - the wheels are identical in radius
 * Additional assumptions to not duplicate information readily available in the URDF:
 *  - the wheels have the same parent frame
 *  - a wheel collision geometry is a cylinder in the urdf
 *  - a wheel joint frame center's vertical projection on the floor must lie within the contact patch
 */
class TricycleDriveController : public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface,
                                    hardware_interface::PositionJointInterface>
{
public:
    TricycleDriveController();

    /**
     * \brief Initialize controller
     * \param hw            Velocity joint interface for the wheels
     * \param root_nh       Node handle at root namespace
     * \param controller_nh Node handle inside the controller namespace
     */
    bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    /**
     * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
     * \param time   Current time
     * \param period Time since the last called to update
     */
    void update(const ros::Time& time, const ros::Duration& period);

    /**
     * \brief Starts controller
     * \param time Current time
     */
    void starting(const ros::Time& time);

    /**
     * \brief Stops controller
     * \param time Current time
     */
    void stopping(const ros::Time& /*time*/);

private:
    std::string name_;

    /// Odometry related:
    ros::Duration publish_period_;
    ros::Time last_state_publish_time_;
    bool open_loop_;
    bool real_hw_;

    /// Hardware handles:
    std::vector<hardware_interface::JointHandle> front_wheel_cmd;
    std::vector<hardware_interface::JointHandle> front_wheel_caster_cmd;

    /// Velocity command related:
    struct Commands {
        double speed;
        double angle;
        ros::Time stamp;

        Commands()
            : speed(0.0)
            , angle(0.0)
            , stamp(0.0)
        {
        }
    };
    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands command_struct_;
    ros::Subscriber sub_command_;

    /// Odometry related:
    boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;
    Odometry odometry_;

    /// Wheel base
    double wheel_base_;

    /// Wheel radius (assuming it's the same for the left and right wheels):
    double wheel_radius_;

    /// Offset of the front wheel center from the caster joint axis [m]
    double front_wheel_offset_to_center_;

    /// Angular offset of the steering angle
    double front_wheel_steering_angle_offset_;

    // Will dynamically adjust wheel speed depending on difference
    // between commanded angle and current odom angle
    double front_wheel_steering_angle_speed_multiplier_;

    /// Wheel separation and radius calibration multipliers:
    double wheel_separation_multiplier_;
    double wheel_radius_multiplier_;
    double wheel_radius_multiplier_odom_;

    /// Timeout to consider cmd_vel commands old:
    double ackermann_cmd_timeout_;

    /// Frame to use for the robot base:
    std::string base_frame_id_;
    std::string odom_topic_;

    /// Whether to publish odometry to tf or not:
    bool enable_odom_tf_;

    /// Number of wheel joints:
    size_t wheel_joints_size_;

    /// Speed limiters:
    Commands last1_cmd_;
    Commands last0_cmd_;
    SpeedLimiter limiter_lin_;
    SpeedLimiter limiter_ang_;

private:
    /**
     * \brief Brakes the wheels, i.e. sets the velocity to 0
     */
    void brake();

    /**
     * \brief Velocity command callback
     * \param command Velocity command message (AckermannDrive)
     */
    void cmdVelCallback(const geometry_msgs::Twist& command);

    /**
     * \brief Get the wheel names from a wheel param
     * \param [in]  controller_nh Controller node handler
     * \param [in]  wheel_param   Param name
     * \param [out] wheel_names   Vector with the whel names
     * \return true if the wheel_param is available and the wheel_names are
     *        retrieved successfully from the param server; false otherwise
     */
    bool getWheelNames(ros::NodeHandle& controller_nh, const std::string& wheel_param, std::vector<std::string>& wheel_names);

    /**
     * \brief Sets odometry parameters from the URDF, i.e. the front wheel radius
     * \param root_nh Root node handle
     * \param front_wheel_name Name of the front wheel joint
     */
    bool setOdomParamsFromUrdf(ros::NodeHandle& root_nh, const std::string& front_wheel_name, bool lookup_wheel_radius);

    /**
     * \brief Sets the odometry publishing fields
     * \param root_nh Root node handle
     * \param controller_nh Node handle inside the controller namespace
     */
    void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
};

PLUGINLIB_EXPORT_CLASS(tricycle_drive_controller::TricycleDriveController, controller_interface::ControllerBase);
} // namespace tricycle_drive_controller
