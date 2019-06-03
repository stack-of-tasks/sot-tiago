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

#include "odometry.h"
#include "speed_limiter.h"

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/linear-algebra.h>

#include <sot/core/matrix-geometry.hh>

namespace dynamicgraph {
  /**
   * This class makes some assumptions on the model of the robot:
   *  - the rotation axes of wheels are collinear
   *  - the wheels are identical in radius
   * Additional assumptions to not duplicate information readily available in the URDF:
   *  - the wheels have the same parent frame
   *  - a wheel collision geometry is a cylinder or sphere in the urdf
   *  - a wheel joint frame center's vertical projection on the floor must lie within the contact patch
   */
  class DiffDriveController : public Entity
  {
  public:
    DYNAMIC_GRAPH_ENTITY_DECL();

    DiffDriveController(const std::string& name);

    /// Header documentation of the python class
    virtual std::string getDocString () const
    {
      return
        "Compute the wheels velocities of a wheeled platform, from a desired velocity of the platform.";
    }

    /// \param order either 1, 2 or 3 for velocity, acceleration and jerk limits.
    void setAngularLimits (const int& order, const bool& enable, const double& min, const double& max);

    /// \param order either 1, 2 or 3 for velocity, acceleration and jerk limits.
    void setLinearLimits  (const int& order, const bool& enable, const double& min, const double& max);

    /// \param order either 1, 2 or 3 for velocity, acceleration and jerk limits.
    /// \return [ min, max ] if a limit for this order is set, otherwise an empty vector.
    Vector getAngularLimits (const int& order);

    /// \param order either 1, 2 or 3 for velocity, acceleration and jerk limits.
    /// \return [ min, max ] if a limit for this order is set, otherwise an empty vector.
    Vector getLinearLimits  (const int& order);

    void setOpenLoop (const bool& openLoop)
    {
      openLoop_ = openLoop;
    }

    bool getOpenLoop () const
    {
      return openLoop_;
    }

    void setWheelSeparation (const double& ws)
    {
      wheelSeparation_ = ws;
    }

    double getWheelSeparation () const
    {
      return wheelSeparation_;
    }

    void setWheelRadius (const double& ws)
    {
      wheelRadius_ = ws;
    }

    double getWheelRadius () const
    {
      return wheelRadius_;
    }

    void setPeriod(const double& dt);

    /// Reset accumulators used for velocity estimation from encoders.
    void resetOdometryAccumulators();

  private:
    /// Compute the wheels velocities.
    /// \returns the vector ( left wheel velocity, right wheel velocity )
    Vector& computeControl  (Vector& control, const int& time);

    Vector& computeBasePose (Vector& basePos, const int& time);

    Vector& computeBaseVel  (Vector& baseVel, const int& time);

    bool computeOdometry (const int& time);

    /// Velocity command related:
    struct Command
    {
      double lin;
      double ang;
      int time;

      Command() : lin(0.0), ang(0.0), time(0) {}
    };

    SignalPtr <Vector, int> baseVelSIN;
    /// Last base velocity before baseVelSIN.
    Command lastBaseVel_;
    /// Last base velocity before lastBaseVelSIN.
    Command penultimateBaseVel_;

    SignalTimeDependent <Vector, int> wheelsVelSOUT;
    SignalTimeDependent <Vector, int> basePoseSOUT;
    SignalTimeDependent <Vector, int> baseVelSOUT;

    /// For closed loop control
    SignalPtr <Vector, int> wheelsPosSIN;

    double dt_;

    /// Odometry related:
    bool openLoop_;

    /// Odometry related:
    details::Odometry odometry_;

    /// Wheel separation, wrt the midpoint of the wheel width:
    double wheelSeparation_;

    /// Wheel radius (assuming it's the same for the left and right wheels):
    double wheelRadius_;

    /// Wheel separation and radius calibration multipliers:
    double wheelSeparationMultiplier_;
    double leftWheelRadiusMultiplier_;
    double rightWheelRadiusMultiplier_;

    /// Speed limiters:
    SpeedLimiter limiterLin_;
    SpeedLimiter limiterAng_;
  };

} // namespace diff_drive_controller
