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
 * Author: Bence Magyar, Enrique Fernández
 */

#include <diff_drive_controller.h>
#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/command-direct-getter.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/factory.h>

#include <cmath>

namespace dynamicgraph {

DiffDriveController::DiffDriveController(const std::string& name)
    : Entity(name),
      baseVelSIN(NULL,
                 "DiffDriveController(" + name + ")::input(vector)::baseVelIn"),
      wheelsVelSOUT("DiffDriveController(" + name +
                    ")::output(vector)::wheelsVelOut"),
      basePoseSOUT("DiffDriveController(" + name +
                   ")::output(vector)::basePoseOut"),
      baseVelSOUT("DiffDriveController(" + name +
                  ")::output(vector)::baseVelOut"),
      wheelsPosSIN(NULL, "DiffDriveController(" + name +
                             ")::input(vector)::wheelsPosIn"),
      dt_(-1),
      openLoop_(true),
      wheelSeparation_(0.0),
      wheelRadius_(0.0),
      wheelSeparationMultiplier_(1.0),
      leftWheelRadiusMultiplier_(1.0),
      rightWheelRadiusMultiplier_(1.0) {
  wheelsVelSOUT.setFunction(
      boost::bind(&DiffDriveController::computeControl, this, _1, _2));
  basePoseSOUT.setFunction(
      boost::bind(&DiffDriveController::computeBasePose, this, _1, _2));
  baseVelSOUT.setFunction(
      boost::bind(&DiffDriveController::computeBaseVel, this, _1, _2));
  signalRegistration(baseVelSIN << wheelsVelSOUT << basePoseSOUT << baseVelSOUT
                                << wheelsPosSIN);

  wheelsVelSOUT.addDependency(baseVelSIN);
  wheelsVelSOUT.addDependency(baseVelSOUT);

  baseVelSOUT.addDependency(basePoseSOUT);

  basePoseSOUT.addDependency(baseVelSIN);

  // Add commands
  addCommand("setOpenLoop",
             command::makeDirectSetter(
                 *this, &openLoop_,
                 "Enable / disable closed loop control of wheels position.\n"
                 "When closed loop control is enable, the true base position "
                 "must be \n"
                 "provided in signal wheelsPosSIN."));
  addCommand("getOpenLoop",
             command::makeDirectGetter(
                 *this, &openLoop_,
                 "Enable / disable closed loop control of wheels position.\n"
                 "When closed loop control is enable, the true base position "
                 "must be \n"
                 "provided in signal wheelsPosSIN."));
  addCommand("setWheelSeparation",
             command::makeDirectSetter(
                 *this, &wheelSeparation_,
                 "Wheel separation, wrt the midpoint of the wheel width"));
  addCommand("getWheelSeparation",
             command::makeDirectGetter(
                 *this, &wheelSeparation_,
                 "Wheel separation, wrt the midpoint of the wheel width"));
  addCommand("setWheelRadius",
             command::makeDirectSetter(*this, &wheelRadius_,
                                       "Wheel radius (assuming it's the same "
                                       "for the left and right wheels"));
  addCommand("getWheelRadius",
             command::makeDirectGetter(*this, &wheelRadius_,
                                       "Wheel radius (assuming it's the same "
                                       "for the left and right wheels"));
  addCommand("setPeriod", command::makeCommandVoid1(
                              *this, &DiffDriveController::setPeriod,
                              command::docCommandVoid1("Set period.",
                                                       " double: the period")));
  addCommand("getPeriod", command::makeDirectGetter(*this, &dt_, "the period"));
  addCommand(
      "resetOdometryAccumulators",
      command::makeCommandVoid0(
          *this, &DiffDriveController::resetOdometryAccumulators,
          command::docCommandVoid0("Reset accumulators used for velocity "
                                   "estimation from encoders.")));

  addCommand(
      "setAngularLimits",
      command::makeCommandVoid4(
          *this, &DiffDriveController::setAngularLimits,
          command::docCommandVoid4(
              "Set angular limits.",
              " int (order): 1, 2 or 3 for velocity, acceleration or jerk",
              " bool: to enable/disable the limits", " double: the lower limit",
              " double: the upper limit")));
  addCommand(
      "setLinearLimits",
      command::makeCommandVoid4(
          *this, &DiffDriveController::setLinearLimits,
          command::docCommandVoid4(
              "Set linear limits.",
              " int (order): 1, 2 or 3 for velocity, acceleration or jerk",
              " bool: to enable/disable the limits", " double: the lower limit",
              " double: the upper limit")));
}

Vector& DiffDriveController::computeControl(Vector& control, const int& time) {
  control.resize(2);
  baseVelSOUT.recompute(time);

  // Apply (possibly new) multipliers:
  const double ws = wheelSeparationMultiplier_ * wheelSeparation_;
  const double lwr = leftWheelRadiusMultiplier_ * wheelRadius_;
  const double rwr = rightWheelRadiusMultiplier_ * wheelRadius_;

  // MOVE ROBOT
  // Retreive current velocity command:
  const Vector& baseVelVect = baseVelSIN.access(time);
  assert(baseVelVect.size() >= 6);
  // TODO check that baseVel is of form (lin_a, lin_b, 0, 0, 0, ang)
  //      such that (lin_a, lin_b) is parallel to (odometry_.getX(), getY())
  Command baseVel;
  // Project the linear velocity onto the current X axis
  // baseVel.lin = baseVelVect[0];
  double heading = odometry_.getHeading();
  Eigen::RowVector2d cs(cos(heading), sin(heading));
  baseVel.lin = cs * baseVelVect.head<2>();
  baseVel.ang = baseVelVect[5];
  baseVel.time = time;

  // Limit velocities and accelerations:
  limiterLin_.limit(baseVel.lin, lastBaseVel_.lin, penultimateBaseVel_.lin,
                    dt_);
  limiterAng_.limit(baseVel.ang, lastBaseVel_.ang, penultimateBaseVel_.ang,
                    dt_);

  penultimateBaseVel_ = lastBaseVel_;
  lastBaseVel_ = baseVel;

  // Compute wheels velocities:
  control.resize(2);
  control[0] = (baseVel.lin - baseVel.ang * ws / 2.0) / lwr;  // Left
  control[1] = (baseVel.lin + baseVel.ang * ws / 2.0) / rwr;  // Right

  return control;
}

Vector& DiffDriveController::computeBaseVel(Vector& baseVel, const int& time) {
  baseVel.resize(2);
  basePoseSOUT.recompute(time);
  baseVel[0] = odometry_.getLinear();
  baseVel[1] = odometry_.getAngular();
  return baseVel;
}

Vector& DiffDriveController::computeBasePose(Vector& basePos, const int& time) {
  basePos.resize(3);
  if (!computeOdometry(time)) {
    basePos.setZero();
    return basePos;
  }
  basePos[0] = odometry_.getX();
  basePos[1] = odometry_.getY();
  basePos[2] = odometry_.getHeading();
  return basePos;
}

bool DiffDriveController::computeOdometry(const int& time) {
  if (dt_ < 0) {
    SEND_ERROR_STREAM_MSG("Period must be set to a position value.");
    return false;
  }
  // // Check if odometry has already been computed.
  // if (   wheelsVelSOUT.getTime() >= time
  // || basePoseSOUT .getTime() >= time
  // || baseVelSOUT  .getTime() >= time)
  // return true;

  // Apply (possibly new) multipliers:
  const double ws = wheelSeparationMultiplier_ * wheelSeparation_;
  const double lwr = leftWheelRadiusMultiplier_ * wheelRadius_;
  const double rwr = rightWheelRadiusMultiplier_ * wheelRadius_;

  odometry_.setWheelParams(ws, lwr, rwr);

  // COMPUTE ODOMETRY
  bool openLoop = openLoop_;
  if (!openLoop_ && wheelsPosSIN.isPlugged()) {
    SEND_ERROR_STREAM_MSG(
        "wheelsPosSIN must be plugged for closed loop control.");
    openLoop = true;
  }
  // TODO lastBaseVel_ should be reset to zero when the velocity has not been
  // applied.
  // double dt = dt_ * (time - lastBaseVel_.time);
  double dt = dt_;
  if (openLoop) {
    odometry_.updateOpenLoop(lastBaseVel_.lin, lastBaseVel_.ang, dt);
  } else {
    const Vector& wheelsPos = wheelsPosSIN.access(time);
    // TODO
    // if (std::isnan(lp) || std::isnan(rp))
    // return;

    // Estimate linear and angular velocity using joint information
    odometry_.update(wheelsPos[0], wheelsPos[1], dt);
  }
  return true;
}

void DiffDriveController::setPeriod(const double& dt) {
  if (dt <= 0) throw std::invalid_argument("Period should be positive.");
  dt_ = dt;
}

void DiffDriveController::resetOdometryAccumulators() { odometry_.init(); }

void DiffDriveController::setAngularLimits(const int& order, const bool& enable,
                                           const double& min,
                                           const double& max) {
  switch (order) {
    case 1:
      limiterAng_.has_velocity_limits = enable;
      limiterAng_.max_velocity = max;
      limiterAng_.min_velocity = min;
      break;
    case 2:
      limiterAng_.has_acceleration_limits = enable;
      limiterAng_.max_acceleration = max;
      limiterAng_.min_acceleration = min;
      break;
    case 3:
      limiterAng_.has_jerk_limits = enable;
      limiterAng_.max_jerk = max;
      limiterAng_.min_jerk = min;
      break;
    default:
      throw std::invalid_argument("Order must be either 1, 2 or 3");
  }
}

void DiffDriveController::setLinearLimits(const int& order, const bool& enable,
                                          const double& min,
                                          const double& max) {
  switch (order) {
    case 1:
      limiterLin_.has_velocity_limits = enable;
      limiterLin_.max_velocity = max;
      limiterLin_.min_velocity = min;
      break;
    case 2:
      limiterLin_.has_acceleration_limits = enable;
      limiterLin_.max_acceleration = max;
      limiterLin_.min_acceleration = min;
      break;
    case 3:
      limiterLin_.has_jerk_limits = enable;
      limiterLin_.max_jerk = max;
      limiterLin_.min_jerk = min;
      break;
    default:
      throw std::invalid_argument("Order must be either 1, 2 or 3");
  }
}

Vector DiffDriveController::getAngularLimits(const int& order) {
  Vector res(2);
  switch (order) {
    case 1:
      if (!limiterAng_.has_velocity_limits) return Vector();
      res << limiterAng_.min_velocity, limiterAng_.max_velocity;
      break;
    case 2:
      if (!limiterAng_.has_acceleration_limits) return Vector();
      res << limiterAng_.min_acceleration, limiterAng_.max_acceleration;
      break;
    case 3:
      if (!limiterAng_.has_jerk_limits) return Vector();
      res << limiterAng_.min_jerk, limiterAng_.max_jerk;
      break;
    default:
      throw std::invalid_argument("Order must be either 1, 2 or 3");
  }
  return res;
}

Vector DiffDriveController::getLinearLimits(const int& order) {
  Vector res(2);
  switch (order) {
    case 1:
      if (!limiterLin_.has_velocity_limits) return Vector();
      res << limiterLin_.min_velocity, limiterLin_.max_velocity;
      break;
    case 2:
      if (!limiterLin_.has_acceleration_limits) return Vector();
      res << limiterLin_.min_acceleration, limiterLin_.max_acceleration;
      break;
    case 3:
      if (!limiterLin_.has_jerk_limits) return Vector();
      res << limiterLin_.min_jerk, limiterLin_.max_jerk;
      break;
    default:
      throw std::invalid_argument("Order must be either 1, 2 or 3");
  }
  return res;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DiffDriveController, "DiffDriveController");
}  // namespace dynamicgraph
