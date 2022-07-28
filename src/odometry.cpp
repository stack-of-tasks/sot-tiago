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

#include "odometry.h"

#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/command-direct-getter.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/factory.h>

#include <boost/bind.hpp>
#include <cmath>

namespace dynamicgraph {
namespace details {
namespace bacc = boost::accumulators;

Odometry::Odometry(size_t velocity_rolling_window_size)
    : x_(0.0),
      y_(0.0),
      heading_(0.0),
      linear_(0.0),
      angular_(0.0),
      wheelSeparation_(0.0),
      left_wheelRadius_(0.0),
      right_wheelRadius_(0.0),
      left_wheel_old_pos_(0.0),
      right_wheel_old_pos_(0.0),
      velocity_rolling_window_size_(velocity_rolling_window_size),
      linear_acc_(RollingWindow::window_size = velocity_rolling_window_size),
      angular_acc_(RollingWindow::window_size = velocity_rolling_window_size),
      integrate_fun_(boost::bind(&Odometry::integrateExact, this, _1, _2)) {}

void Odometry::init() {
  // Reset accumulators:
  resetAccumulators();
}

bool Odometry::update(double left_pos, double right_pos, double dt) {
  /// Get current wheel joint positions:
  const double left_wheel_cur_pos = left_pos * left_wheelRadius_;
  const double right_wheel_cur_pos = right_pos * right_wheelRadius_;

  /// Estimate velocity of wheels using old and current position:
  const double left_wheel_est_vel = left_wheel_cur_pos - left_wheel_old_pos_;
  const double right_wheel_est_vel = right_wheel_cur_pos - right_wheel_old_pos_;

  /// Update old position with current:
  left_wheel_old_pos_ = left_wheel_cur_pos;
  right_wheel_old_pos_ = right_wheel_cur_pos;

  /// Compute linear and angular diff:
  const double linear = (right_wheel_est_vel + left_wheel_est_vel) * 0.5;
  const double angular =
      (right_wheel_est_vel - left_wheel_est_vel) / wheelSeparation_;

  /// Integrate odometry:
  integrate_fun_(linear, angular);

  /// We cannot estimate the speed with very small time intervals:
  if (dt < 0.0001) return false;  // Interval too small to integrate with

  /// Estimate speeds using a rolling mean to filter them out:
  linear_acc_(linear / dt);
  angular_acc_(angular / dt);

  linear_ = bacc::rolling_mean(linear_acc_);
  angular_ = bacc::rolling_mean(angular_acc_);

  return true;
}

void Odometry::updateOpenLoop(double linear, double angular, double dt) {
  /// Save last linear and angular velocity:
  linear_ = linear;
  angular_ = angular;

  /// Integrate odometry:
  integrate_fun_(linear * dt, angular * dt);
}

void Odometry::setWheelParams(double wheelSeparation, double left_wheelRadius,
                              double right_wheelRadius) {
  wheelSeparation_ = wheelSeparation;
  left_wheelRadius_ = left_wheelRadius;
  right_wheelRadius_ = right_wheelRadius;
}

void Odometry::setVelocityRollingWindowSize(
    size_t velocity_rolling_window_size) {
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear, double angular) {
  const double direction = heading_ + angular * 0.5;

  /// Runge-Kutta 2nd order integration:
  x_ += linear * cos(direction);
  y_ += linear * sin(direction);
  heading_ += angular;
}

/**
 * \brief Other possible integration method provided by the class
 * \param linear
 * \param angular
 */
void Odometry::integrateExact(double linear, double angular) {
  if (fabs(angular) < 1e-6)
    integrateRungeKutta2(linear, angular);
  else {
    /// Exact integration (should solve problems when angular is zero):
    const double heading_old = heading_;
    const double r = linear / angular;
    heading_ += angular;
    x_ += r * (sin(heading_) - sin(heading_old));
    y_ += -r * (cos(heading_) - cos(heading_old));
  }
}

void Odometry::resetAccumulators() {
  linear_acc_ = RollingMeanAcc(RollingWindow::window_size =
                                   velocity_rolling_window_size_);
  angular_acc_ = RollingMeanAcc(RollingWindow::window_size =
                                    velocity_rolling_window_size_);
}

}  // namespace details

namespace bacc = boost::accumulators;

Odometry::Odometry(const std::string& name)
    : Entity(name),
      wheelsPositionSIN(
          NULL, "Odometry(" + name + ")::input(vector)::wheelsPosition"),
      baseVelocityEstimationSOUT(
          boost::bind(&Odometry::computeVelocityEstimation, this, _1, _2),
          wheelsPositionSIN,
          "Odometry(" + name + ")::output(vector)::baseVelocityEstimation"),
      baseVelocityFilteredEstimationSOUT(
          boost::bind(&Odometry::computeVelocityFilteredEstimation, this, _1,
                      _2),
          baseVelocityEstimationSOUT,
          "Odometry(" + name +
              ")::output(vector)::baseVelocityFilteredEstimation"),
      baseVelocitySIN(NULL,
                      "Odometry(" + name + ")::input(vector)::baseVelocity"),
      baseConfigSOUT(boost::bind(&Odometry::computeBaseConfig, this, _1, _2),
                     baseVelocitySIN,
                     "Odometry(" + name + ")::output(vector)::baseConfig"),
      basePoseSOUT(
          boost::bind(&Odometry::computeBasePose, this, _1, _2), baseConfigSOUT,
          "Odometry(" + name + ")::output(matrixHomogeneous)::basePose")

      ,
      dt_(1.),
      x_(0.0),
      y_(0.0),
      heading_(0.0),
      wheelSeparationInv_(0.0),
      wheelRadii_(0.0, 0.0),
      wheelOldPos_(0.0, 0.0),
      rollingWindowSize_(10),
      linear_acc_(RollingWindow::window_size = rollingWindowSize_),
      angular_acc_(RollingWindow::window_size = rollingWindowSize_) {
  signalRegistration(wheelsPositionSIN << baseVelocityEstimationSOUT
                                       << baseVelocityFilteredEstimationSOUT
                                       << baseVelocitySIN << baseConfigSOUT
                                       << basePoseSOUT);

  // By default, use the velocity estimation
  // (i.e. loop closed with wheel position sensors).
  baseVelocitySIN.plug(&baseVelocityEstimationSOUT);

  addCommand("setWheelParams",
             command::makeCommandVoid3(
                 *this, &Odometry::setWheelParams,
                 command::docCommandVoid3(
                     "Sets the wheel parameters: radius and separation",
                     "double: Separation between left and right wheels [m]",
                     "double: Left wheel radius [m]",
                     "double: Right wheel radius [m]")));
  addCommand("setBasePose",
             command::makeCommandVoid3(
                 *this, &Odometry::setBasePose,
                 command::docCommandVoid3(
                     "Set the base pose to integrate from.", "double: x [m]",
                     "double: y [m]", "double: heading [rad]")));
  addCommand("setPeriod",
             command::makeDirectSetter(*this, &dt_,
                                       "Set period (only affects the velocity "
                                       "estimation and nothing else)."));
  addCommand("getPeriod", command::makeDirectGetter(*this, &dt_, "the period"));
}

Vector& Odometry::computeVelocityEstimation(Vector& vel, int time) {
  vel.resize(2);
  const Vector& wheelCurAng = wheelsPositionSIN(time);

  /// Get current wheel joint positions:
  Eigen::Array2d wheelCurPos = wheelCurAng.array() * wheelRadii_;

  /// Estimate velocity of wheels using old and current position:
  Eigen::Array2d wheelEstVel = (wheelCurPos - wheelOldPos_) / dt_;

  /// Update old position with current:
  wheelOldPos_ = wheelCurPos;

  /// Compute linear and angular diff:
  vel[0] = (wheelEstVel[1] + wheelEstVel[0]) * 0.5;
  vel[1] = (wheelEstVel[1] - wheelEstVel[0]) * wheelSeparationInv_;

  /// Estimate speeds using a rolling mean to filter them out:
  linear_acc_(vel[0]);
  angular_acc_(vel[1]);

  return vel;
}

Vector& Odometry::computeVelocityFilteredEstimation(Vector& vel, int time) {
  baseVelocityEstimationSOUT.recompute(time);

  vel.resize(2);
  vel[0] = bacc::rolling_mean(linear_acc_);
  vel[1] = bacc::rolling_mean(angular_acc_);

  return vel;
}

Vector& Odometry::computeBaseConfig(Vector& base, int time) {
  base.resize(3);
  const Vector& vel = baseVelocitySIN(time);
  integrateExact(vel[0] * dt_, vel[1] * dt_);

  base << x_, y_, heading_;
  return base;
}

sot::MatrixHomogeneous& Odometry::computeBasePose(sot::MatrixHomogeneous& M,
                                                  int time) {
  const Vector& base = baseConfigSOUT(time);
  M.setIdentity();
  M.translation().head<2>() = base.head<2>();
  double c = cos(base(2)), s = sin(base(2));
  M.linear().topLeftCorner<2, 2>() << c, -s, s, c;
  return M;
}

void Odometry::setBasePose(const double& x, const double& y,
                           const double& heading) {
  x_ = x;
  y_ = y;
  heading_ = heading;

  resetAccumulators();
}

void Odometry::setWheelParams(const double& wheelSeparation,
                              const double& leftWheelRadius,
                              const double& rightWheelRadius) {
  wheelSeparationInv_ = 1 / wheelSeparation;
  wheelRadii_ << leftWheelRadius, rightWheelRadius;
}

void Odometry::setVelocityRollingWindowSize(const size_t& rollingWindowSize) {
  rollingWindowSize_ = rollingWindowSize;
  resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear, double angular) {
  const double direction = heading_ + angular * 0.5;

  /// Runge-Kutta 2nd order integration:
  x_ += linear * cos(direction);
  y_ += linear * sin(direction);
  heading_ += angular;
}

/**
 * \brief Other possible integration method provided by the class
 * \param linear
 * \param angular
 */
void Odometry::integrateExact(double linear, double angular) {
  if (fabs(angular) < 1e-6)
    integrateRungeKutta2(linear, angular);
  else {
    /// Exact integration (should solve problems when angular is zero):
    const double heading_old = heading_;
    const double r = linear / angular;
    heading_ += angular;
    x_ += r * (sin(heading_) - sin(heading_old));
    y_ += -r * (cos(heading_) - cos(heading_old));
  }
}

void Odometry::resetAccumulators() {
  linear_acc_ = RollingMeanAcc(RollingWindow::window_size = rollingWindowSize_);
  angular_acc_ =
      RollingMeanAcc(RollingWindow::window_size = rollingWindowSize_);
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Odometry, "Odometry");

}  // namespace dynamicgraph
