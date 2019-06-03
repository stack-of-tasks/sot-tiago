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

    , leftWheelIdx_ (-1)
    , rightWheelIdx_ (-1)
    , wheelSeparation_(0.0)
    , wheelRadius_(0.0)
    , wheelSeparationMultiplierSIN  (NULL,"HolonomicProjection("+name+")::input(double)::wheelSeparationMultiplierSIN" )
    , leftWheelRadiusMultiplierSIN  (NULL,"HolonomicProjection("+name+")::input(double)::leftWheelRadiusMultiplierSIN" )
    , rightWheelRadiusMultiplierSIN (NULL,"HolonomicProjection("+name+")::input(double)::rightWheelRadiusMultiplierSIN")

    , projectionSOUT ( boost::bind (&HolonomicProjection::computeProjection, this, _1, _2),
        wheelSeparationMultiplierSIN  << leftWheelRadiusMultiplierSIN  << rightWheelRadiusMultiplierSIN
        << basePoseSIN,
        "HolonomicProjection("+name+")::output(Matrix)::projection")
  {
    signalRegistration (basePoseSIN << projectionSOUT
        << wheelSeparationMultiplierSIN
        << leftWheelRadiusMultiplierSIN
        << rightWheelRadiusMultiplierSIN);

    wheelSeparationMultiplierSIN .setConstant (1);
    leftWheelRadiusMultiplierSIN .setConstant (1);
    rightWheelRadiusMultiplierSIN.setConstant (1);

    // Add commands
    addCommand ("setSize",
        command::makeDirectSetter (*this, &nv_, "Number of DoF of the robot"));
    addCommand ("getSize",
        command::makeDirectGetter (*this, &nv_, "Number of DoF of the robot"));

    addCommand ("setLeftWheel",
        command::makeDirectSetter (*this, &leftWheelIdx_, "Left wheel index."));
    addCommand ("getLeftWheel",
        command::makeDirectGetter (*this, &leftWheelIdx_, "Left wheel index."));

    addCommand ("setRightWheel",
        command::makeDirectSetter (*this, &rightWheelIdx_, "Right wheel index."));
    addCommand ("getRightWheel",
        command::makeDirectGetter (*this, &rightWheelIdx_, "Right wheel index."));

    addCommand ("setWheelSeparation",
        command::makeDirectSetter (*this, &wheelSeparation_,
          "Wheel separation, wrt the midpoint of the wheel width")
        );
    addCommand ("getWheelSeparation",
        command::makeDirectGetter (*this, &wheelSeparation_,
          "Wheel separation, wrt the midpoint of the wheel width")
        );

    addCommand ("setWheelRadius",
        command::makeDirectSetter (*this, &wheelRadius_,
          "Wheel radius (assuming it's the same for the left and right wheels")
        );
    addCommand ("getWheelRadius",
        command::makeDirectGetter (*this, &wheelRadius_,
          "Wheel radius (assuming it's the same for the left and right wheels")
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

    bool hasWheels = (leftWheelIdx_ >= 0 && rightWheelIdx_ >= 0);

    const int nr = nv_,
              nc = (hasWheels ? nv_ - 6 : nv_ - 4);
    proj = Matrix::Zero (nr, nc);

    // 6 first rows
    proj.topLeftCorner<6, 2>() << twist.col(0), twist.col(5);

    // Other rows
    if (hasWheels) {
      const double ws  = wheelSeparationMultiplierSIN  (time) * wheelSeparation_;
      const double lwr = leftWheelRadiusMultiplierSIN  (time) * wheelRadius_;
      const double rwr = rightWheelRadiusMultiplierSIN (time) * wheelRadius_;
      proj(leftWheelIdx_ , 0) = 1. / lwr;
      proj(leftWheelIdx_ , 1) = - ws / (2.0 * lwr); // Left
      proj(rightWheelIdx_, 0) = 1. / rwr;
      proj(rightWheelIdx_, 1) =   ws / (2.0 * rwr); // Right
      int c = 2;
      for (int r = 6; r < nv_; ++r) {
        if (r == leftWheelIdx_ || r == rightWheelIdx_) continue;
        proj(r, c) = 1.;
        ++c;
      }
      assert (c == nc);
    } else {
      proj.bottomRightCorner (nv_-6, nv_-6).setIdentity();
    }

    return proj;
  }

  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (HolonomicProjection, "HolonomicProjection");
} // namespace dynamicgraph
