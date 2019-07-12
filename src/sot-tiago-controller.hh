/*
 * Copyright 2016,
 *
 * Olivier Stasse
 *
 * LAAS, CNRS
 *
 * This file is part of TiagoController.
 */

#ifndef _SOT_TiagoController_H_
#define _SOT_TiagoController_H_

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/device.hh>
#include <sot/core/abstract-sot-external-interface.hh>

#include "sot-tiago-device.hh"
#include <dynamic_graph_bridge/ros_interpreter.hh>
namespace dgsot=dynamicgraph::sot;

class SoTTiagoController: public 
  dgsot::AbstractSotExternalInterface
{
 public:

  static const std::string LOG_PYTHON;
  
  SoTTiagoController();
  SoTTiagoController(const char robotName[]);
  SoTTiagoController(std::string robotName);
  virtual ~SoTTiagoController();

  void setupSetSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);

  void nominalSetSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);

  void cleanupSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void getControl(std::map<std::string, dgsot::ControlValues> &anglesOut);

  void setNoIntegration(void);
  void setSecondOrderIntegration(void);

  /// Embedded python interpreter accessible via Corba/ros
  boost::shared_ptr<dynamicgraph::Interpreter> interpreter_;

 protected:
  // Update output port with the control computed from the
  // dynamic graph.
  void updateRobotState(std::vector<double> &anglesIn);
  
  void runPython(std::ostream& file,
		 const std::string& command,
		 dynamicgraph::Interpreter& interpreter);
  
  virtual void startupPython();
    
  void init();

  SoTTiagoDevice* device_;
};

#endif /* _SOT_TiagoController_H_ */

