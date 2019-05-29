#include <holonomic-projection.h>

#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/command-direct-getter.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/factory.h>

#include <sot/core/matrix-geometry.hh>

namespace dynamicgraph {

  HolonomicProjection::HolonomicProjection(const std::string& name)
    : Entity (name)
    , nv_ (-1)
    , basePoseSIN (NULL, "HolonomicProjection("+name+")::input(MatrixHomogeneous)::basePose")
    , projectionSOUT ( boost::bind (&HolonomicProjection::computeProjection, this, _1, _2),
        basePoseSIN, "HolonomicProjection("+name+")::output(Matrix)::projection")
  {
    signalRegistration (basePoseSIN << projectionSOUT);

    // Add commands
    addCommand ("setSize",
        command::makeDirectSetter (*this, &nv_,
          "Number of DoF of the robot")
        );
    addCommand ("getSize",
        command::makeDirectGetter (*this, &nv_,
          "Number of DoF of the robot")
        );
  }

  Matrix& HolonomicProjection::computeProjection (Matrix& proj, const int& time)
  {
    if (nv_ < 6) {
      SEND_ERROR_STREAM_MSG("Size must be superior to 6.");
      proj.resize(0,0);
      return proj;
    }

    const sot::MatrixHomogeneous& oMb = basePoseSIN.access (time);
    sot::MatrixTwist twist;
    sot::buildFrom (oMb, twist);

    const int nr = nv_, nc = nv_ - 4;
    proj.resize (nr, nc);

    proj <<
      // 6 first rows
      twist.col(0), twist.col(5), Matrix::Zero(6, nv_-6),
      // Other rows
      Matrix::Zero(nv_-6, 2), Matrix::Identity(nv_-6, nv_-6);

    return proj;
  }

  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (HolonomicProjection, "HolonomicProjection");
} // namespace dynamicgraph
