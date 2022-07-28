/*
 * Copyright 2016,
 *
 * Olivier Stasse
 *
 * LAAS, CNRS
 *
 * This file is part of TiagoController.
 * TiagoController is not a free software,
 * it contains information related to Tiago which involves
 * that you either purchased the proper license to havec access to
 * those informations, or that you signed the appropriate
 * Non-Disclosure agreement.
 *
 *
 */

#ifndef _SOT_TiagoDevice_H_
#define _SOT_TiagoDevice_H_

#include <dynamic-graph/entity.h>
#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal.h>

#include <sot/core/abstract-sot-external-interface.hh>
#include <sot/core/device.hh>
#include <sot/core/matrix-geometry.hh>

namespace dgsot = dynamicgraph::sot;
namespace dg = dynamicgraph;

class SoTTiagoDevice : public dgsot::Device {
 public:
  static const std::string CLASS_NAME;
  static const double TIMESTEP_DEFAULT;

  virtual const std::string &getClassName() const { return CLASS_NAME; }

  SoTTiagoDevice(std::string RobotName);
  virtual ~SoTTiagoDevice();

  void setSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void setupSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void nominalSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void cleanupSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void getControl(std::map<std::string, dgsot::ControlValues> &anglesOut);

  void setLeftWheelIndex(int idx);

  void setRightWheelIndex(int idx);

  /// \todo this should go into the parent class, in sot-core package
  void setTimeStep(double dt) { timestep_ = dt; }

 protected:
  void setClosedLoop(const bool &closedLoop) { closedLoop_ = closedLoop; };

  /// \brief Whether the control of the base should be expressed in odometry
  ///        frame of base frame.
  bool closedLoop_;

  /// \brief Previous robot configuration.
  dg::Vector previousState_;

  /// Intermediate variables to avoid allocation during control
  std::vector<double> baseff_;

  /// Accelerations measured by accelerometers
  dynamicgraph::Signal<dg::Vector, int> accelerometerSOUT_;
  /// Rotation velocity measured by gyrometers
  dynamicgraph::Signal<dg::Vector, int> gyrometerSOUT_;
  /// motor currents
  dynamicgraph::Signal<dg::Vector, int> currentSOUT_;

  /// proportional and derivative position-control gains
  dynamicgraph::Signal<dg::Vector, int> p_gainsSOUT_;

  dynamicgraph::Signal<dg::Vector, int> d_gainsSOUT_;

  /// Intermediate variables to avoid allocation during control
  dg::Vector dgforces_;
  dg::Vector dgRobotState_;
  dgsot::MatrixRotation pose;
  dg::Vector accelerometer_;
  dg::Vector gyrometer_;
  dg::Vector torques_;
  dg::Vector currents_;
  dg::Vector p_gains_;
  dg::Vector d_gains_;

  int leftWheelIdx_, rightWheelIdx_;
};
#endif /* _SOT_TiagoDevice_H_*/
