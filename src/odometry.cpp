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
 * Author: Luca Marchionni
 * Author: Bence Magyar
 * Author: Enrique Fernández
 * Author: Paul Mathieu
 */

#include <tricycle_drive_controller/odometry.h>

#include <boost/bind.hpp>

namespace tricycle_drive_controller
{
namespace bacc = boost::accumulators;

Odometry::Odometry(size_t velocity_rolling_window_size)
    : timestamp_(0.0)
    , x_(0.0)
    , y_(0.0)
    , heading_(0.0)
    , linear_(0.0)
    , angular_(0.0)
    , wheel_base_(0.0)
    , wheel_radius_(0.0)
    , front_wheel_old_pos_(0.0)
    , velocity_rolling_window_size_(velocity_rolling_window_size)
    , linear_acc_(RollingWindow::window_size = velocity_rolling_window_size)
    , angular_acc_(RollingWindow::window_size = velocity_rolling_window_size)
    , integrate_fun_(boost::bind(&Odometry::integrateExact, this, _1, _2, _3))
{
}

void
Odometry::init(const ros::Time& time)
{
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = time;
}

bool
Odometry::update(double front_wheel_pos, double front_wheel_caster_pos, const ros::Time& time)
{
    /// Get current wheel joint positions:
    const double front_wheel_cur_pos = front_wheel_pos * wheel_radius_;

    /// Estimate velocity of wheels using old and current position:
    const double front_wheel_est_vel = front_wheel_cur_pos - front_wheel_old_pos_;

    /// Update old position with current:
    front_wheel_old_pos_ = front_wheel_cur_pos;

    /// Compute linear and angular diff:
    const double cos_phi = std::cos(front_wheel_caster_pos);
    const double sin_phi = std::sin(front_wheel_caster_pos);

    const double linear = front_wheel_est_vel * cos_phi;
    const double angular = front_wheel_est_vel * sin_phi / wheel_base_;

    double radius = 1e6;

    if (fabs(angular) >= 1e-6) {
        const double cot_phi = cos_phi / sin_phi;
        radius = cot_phi * wheel_base_;
    }

    /// Integrate odometry:
    integrate_fun_(linear, angular, radius);

    /// We cannot estimate the speed with very small time intervals:
    const double dt = (time - timestamp_).toSec();
    if (dt < 0.0001)
        return false; // Interval too small to integrate with

    timestamp_ = time;

    /// Estimate speeds using a rolling mean to filter them out:
    linear_acc_(linear / dt);
    angular_acc_(angular / dt);

    linear_ = bacc::rolling_mean(linear_acc_);
    angular_ = bacc::rolling_mean(angular_acc_);

    return true;
}

void
Odometry::updateOpenLoop(double linear, double angular, const ros::Time& time)
{
    /// Save last linear and angular velocity:
    linear_ = linear;
    angular_ = angular;

    /// Integrate odometry:
    const double dt = (time - timestamp_).toSec();
    timestamp_ = time;
    integrate_fun_(linear * dt, angular * dt, linear_ / angular_);
}

bool
Odometry::updateFromHWEncoders(double front_wheel_velocity, double front_wheel_angle, const ros::Time& time)
{
    /// Compute linear and angular diff:
    const double cos_phi = std::cos(front_wheel_angle);
    const double sin_phi = std::sin(front_wheel_angle);

    const double linear = front_wheel_velocity * cos_phi;
    const double angular = front_wheel_velocity * sin_phi / wheel_base_;

    double radius = 1e6;

    if (fabs(angular) >= 1e-6) {
        const double cot_phi = cos_phi / sin_phi;
        radius = cot_phi * wheel_base_;
    }

    /// Integrate odometry:
    integrate_fun_(linear, angular, radius);

    /// We cannot estimate the speed with very small time intervals:
    const double dt = (time - timestamp_).toSec();
    if (dt < 0.0001) {
        return false; // Interval too small to integrate with
    }

    timestamp_ = time;

    /// Estimate speeds using a rolling mean to filter them out:
    linear_acc_(linear / dt);
    angular_acc_(angular / dt);

    linear_ = bacc::rolling_mean(linear_acc_);
    angular_ = bacc::rolling_mean(angular_acc_);

    return true;
}

void
Odometry::setWheelParams(double wheel_base, double wheel_radius)
{
    wheel_base_ = wheel_base;
    wheel_radius_ = wheel_radius;
}

void
Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    resetAccumulators();
}

void
Odometry::integrateRungeKutta2(double linear, double angular)
{
    const double direction = heading_ + angular * 0.5;

    /// Runge-Kutta 2nd order integration:
    x_ += linear * std::cos(direction);
    y_ += linear * std::sin(direction);
    heading_ += angular;
}

/**
 * \brief Other possible integration method provided by the class
 * \param linear
 * \param angular
 */
void
Odometry::integrateExact(double linear, double angular, double radius)
{
    if (fabs(angular) < 1e-6)
        integrateRungeKutta2(linear, angular);
    else {
        /// Exact integration (should solve problems when angular is zero):
        const double heading_old = heading_;
        heading_ += angular;
        x_ += radius * (std::sin(heading_) - std::sin(heading_old));
        y_ += -radius * (std::cos(heading_) - std::cos(heading_old));
    }
}

void
Odometry::resetAccumulators()
{
    linear_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    angular_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
}

} // namespace tricycle_drive_controller
