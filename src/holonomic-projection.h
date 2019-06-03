/*
 * Author: Joseph Mirabel
 */

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/linear-algebra.h>

#include <sot/core/matrix-geometry.hh>

namespace dynamicgraph {
  /// Create a constraint to take into account:
  /// - the holonomic system: the base velocity, expressed in the base frame,
  ///                         is of the form ( v_lin, 0, 0, 0, 0, w_ang)
  /// - the relation between the wheels velocity and the base velocity.
  ///
  /// At the moment, only two wheels are considered.
  ///
  /// The signal projectionSOUT should be plugged into SoT.proj0. The signal
  /// value is a matrix K such that \f$ \dot{q} = K * u \f$
  /// where:
  /// - \f$ \dot{q} \in \mathcal{se}(3)\times\mathcal{R}^n \f$ is the velocity
  ///   of the underactuated robot,
  /// - \f$ u \in \mathcal{R}^2 \times \mathcal{R}^{n-2} \f$ is the velocity of
  ///   the actuated robot, the two first being the linear and angular velocity.
  class HolonomicProjection : public Entity
  {
  public:
    DYNAMIC_GRAPH_ENTITY_DECL();

    HolonomicProjection(const std::string& name);

    /// Header documentation of the python class
    virtual std::string getDocString () const
    {
      return "Compute the projection matrix for a wheeled platform.";
    }

    void setSize (int nv)
    {
      nv_ = nv;
    }

  private:
    /// Compute the projection matrix
    /// \return proj a matrix of size nv_ * (nv_ - 4)
    Matrix& computeProjection  (Matrix& proj, const int& time);

    /// Number of DoF of the robot
    int nv_;
    SignalPtr <sot::MatrixHomogeneous, int> basePoseSIN;

    /// The index of the wheels DoF.
    int leftWheelIdx_, rightWheelIdx_;

    /// Wheel separation, wrt the midpoint of the wheel width:
    double wheelSeparation_;

    /// Wheel radius (assuming it's the same for the left and right wheels):
    double wheelRadius_;

    /// Calibration of the wheels separation and radii.
    SignalPtr <double, int> wheelSeparationMultiplierSIN,
                            leftWheelRadiusMultiplierSIN,
                            rightWheelRadiusMultiplierSIN;

    SignalTimeDependent <Matrix, int> projectionSOUT;
  };

} // namespace diff_drive_controller
