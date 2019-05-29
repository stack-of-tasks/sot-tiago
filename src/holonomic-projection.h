/*
 * Author: Joseph Mirabel
 */

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/linear-algebra.h>

#include <sot/core/matrix-geometry.hh>

namespace dynamicgraph {
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
    SignalTimeDependent <Matrix, int> projectionSOUT;
  };

} // namespace diff_drive_controller
