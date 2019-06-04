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

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <sot/core/matrix-geometry.hh>

namespace dynamicgraph
{
namespace details
{
  namespace bacc = boost::accumulators;

  /**
   * \brief The Odometry class handles odometry readings
   * (2D pose and velocity with related timestamp)
   * \deprecated TO BE DELETED
   */
  class Odometry
  {
  public:

    /// Integration function, used to integrate the odometry:
    typedef boost::function<void(double, double)> IntegrationFunction;

    /**
     * \brief Constructor
     * Timestamp will get the current time value
     * Value will be set to zero
     * \param velocity_rolling_window_size Rolling window size used to compute the velocity mean
     */
    Odometry(size_t velocity_rolling_window_size = 10);

    /**
     * \brief Initialize the odometry
     * \param time Current time
     */
    void init();

    /**
     * \brief Updates the odometry class with latest wheels position
     * \param left_pos  Left  wheel position [rad]
     * \param right_pos Right wheel position [rad]
     * \param dt      the elapsed time since last call.
     * \return true if the odometry is actually updated
     */
    bool update(double left_pos, double right_pos, double dt);

    /**
     * \brief Updates the odometry class with latest velocity command
     * \param linear  Linear velocity [m/s]
     * \param angular Angular velocity [rad/s]
     * \param dt      the elapsed time since last call.
     */
    void updateOpenLoop(double linear, double angular, double dt);

    /**
     * \brief heading getter
     * \return heading [rad]
     */
    double getHeading() const
    {
      return heading_;
    }

    /**
     * \brief x position getter
     * \return x position [m]
     */
    double getX() const
    {
      return x_;
    }

    /**
     * \brief y position getter
     * \return y position [m]
     */
    double getY() const
    {
      return y_;
    }

    /**
     * \brief linear velocity getter
     * \return linear velocity [m/s]
     */
    double getLinear() const
    {
      return linear_;
    }

    /**
     * \brief angular velocity getter
     * \return angular velocity [rad/s]
     */
    double getAngular() const
    {
      return angular_;
    }

    /**
     * \brief Sets the wheel parameters: radius and separation
     * \param wheel_separation   Separation between left and right wheels [m]
     * \param left_wheelRadius  Left wheel radius [m]
     * \param right_wheelRadius Right wheel radius [m]
     */
    void setWheelParams(double wheel_separation, double left_wheelRadius, double right_wheelRadius);

    /**
     * \brief Velocity rolling window size setter
     * \param velocity_rolling_window_size Velocity rolling window size
     */
    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

  private:

    /// Rolling mean accumulator and window:
    typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
    typedef bacc::tag::rolling_window RollingWindow;

    /**
     * \brief Integrates the velocities (linear and angular) using 2nd order Runge-Kutta
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
    void integrateRungeKutta2(double linear, double angular);

    /**
     * \brief Integrates the velocities (linear and angular) using exact method
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
    void integrateExact(double linear, double angular);

    /**
     *  \brief Reset linear and angular accumulators
     */
    void resetAccumulators();

    /// Current pose:
    double x_;        //   [m]
    double y_;        //   [m]
    double heading_;  // [rad]

    /// Current velocity:
    double linear_;  //   [m/s]
    double angular_; // [rad/s]

    /// Wheel kinematic parameters [m]:
    double wheelSeparation_;
    double left_wheelRadius_;
    double right_wheelRadius_;

    /// Previou wheel position/state [rad]:
    double left_wheel_old_pos_;
    double right_wheel_old_pos_;

    /// Rolling mean accumulators for the linar and angular velocities:
    size_t velocity_rolling_window_size_;
    RollingMeanAcc linear_acc_;
    RollingMeanAcc angular_acc_;

    /// Integration funcion, used to integrate the odometry:
    IntegrationFunction integrate_fun_;
  };

} // namespace details

  class Odometry : public Entity
  {
    public:
      DYNAMIC_GRAPH_ENTITY_DECL();

      Odometry (const std::string& name);

      virtual std::string getDocString () const
      {
        return "Odometry using the wheels position sensors.";
      }

    private:
      /// A vector of size 2 containing the left and right wheel velocities, in
      /// this order [rad].
      SignalPtr <Vector, int> wheelsPositionSIN;

      /// Estimation of the velocity of the base using the wheel positions [m/s,rad/s].
      SignalTimeDependent <Vector, int> baseVelocityEstimationSOUT;

      /// Filtered estimation of the velocity of the base using the wheel positions [m/s,rad/s].
      SignalTimeDependent <Vector, int> baseVelocityFilteredEstimationSOUT;

      /// The base velocity to integrate [m/s,rad/s].
      /// To enable open-loop odometry, plug this to the control sent to the robot base.
      SignalPtr <Vector, int> baseVelocitySIN;

      /// Outputs the position of the base as
      /// \f$ (x, y, \theta) \f$ [m,rad].
      SignalTimeDependent <Vector, int> baseConfigSOUT;

      /// Outputs the position of the base as an homogeneous transformation.
      SignalTimeDependent <sot::MatrixHomogeneous, int> basePoseSOUT;

      Vector& computeVelocityEstimation (Vector& vel, int time);

      Vector& computeVelocityFilteredEstimation (Vector& vel, int time);

      Vector& computeBaseConfig (Vector& base, int time);

      sot::MatrixHomogeneous& computeBasePose (sot::MatrixHomogeneous& M, int time);

      /// Set the base pose to integrate from.
      void setBasePose (const double& x, const double& y, const double& heading);

      /**
       * \brief Sets the wheel parameters: radius and separation
       * \param wheel_separation   Separation between left and right wheels [m]
       * \param left_wheelRadius  Left wheel radius [m]
       * \param right_wheelRadius Right wheel radius [m]
       */
      void setWheelParams(const double& wheel_separation, const double& left_wheelRadius, const double& right_wheelRadius);

      /**
       * \brief Velocity rolling window size setter
       * \param velocity_rolling_window_size Velocity rolling window size
       */
      void setVelocityRollingWindowSize(const size_t& velocity_rolling_window_size);

    private:

      /// Rolling mean accumulator and window:
      typedef boost::accumulators::accumulator_set<double,
              boost::accumulators::stats<boost::accumulators::tag::rolling_mean>
                > RollingMeanAcc;
      typedef boost::accumulators::tag::rolling_window RollingWindow;

      /**
       * \brief Integrates the velocities (linear and angular) using 2nd order Runge-Kutta
       * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
       * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
       */
      void integrateRungeKutta2(double linear, double angular);

      /**
       * \brief Integrates the velocities (linear and angular) using exact method
       * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
       * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
       */
      void integrateExact(double linear, double angular);

      double dt_;

      /// Current pose:
      double x_;        //   [m]
      double y_;        //   [m]
      double heading_;  // [rad]

      /// Wheel kinematic parameters [m]:
      double wheelSeparationInv_;
      /// Left and right wheel radii [m].
      Eigen::Array2d wheelRadii_;

      /// Previous wheel position/state [m]:
      Eigen::Array2d wheelOldPos_;

      /// Rolling mean accumulators for the linar and angular velocities:
      size_t rollingWindowSize_;
      RollingMeanAcc linear_acc_;
      RollingMeanAcc angular_acc_;

      void resetAccumulators();
  };
}

#endif /* ODOMETRY_H_ */
