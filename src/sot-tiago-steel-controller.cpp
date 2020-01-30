/*
 * Copyright 2018,
 *
 * Joseph Mirabel
 *
 * LAAS, CNRS
 *
 * This file is part of TIAGOController.
 * TIAGOController is a free software, 
 *
 */

#include <pinocchio/fwd.hpp>
#include <sot/core/debug.hh>

/* TiagoSteel is the instance of TIAGO named "steel" */
#define ROBOTNAME std::string("TIAGOSTEEL")

#include "sot-tiago-steel-controller.hh"

const std::string SoTTiagoSteelController::LOG_PYTHON_TIAGOSTEEL="/tmp/TiagoSteelController_python.out";

SoTTiagoSteelController::SoTTiagoSteelController():
  SoTTiagoController(ROBOTNAME),
  withWheels_ (false)
{
  ros::NodeHandle nh;
  nh.getParam("/sot_controller/use_mobile_base", withWheels_);
  ROS_INFO_STREAM("Loading SoT Tiago steel controller with"
      << (withWheels_ ? "" : "out") <<  "wheel");
  if (withWheels_) {
    // Control wheels in velocity.
    // 6 and 7 correspond to left and right wheel joints.
    device_->setLeftWheelIndex (6);
    device_->setLeftWheelIndex (7);
  }
  startupPython();
  interpreter_->startRosService ();
}

void SoTTiagoSteelController::startupPython()
{
  SoTTiagoController::startupPython();
  std::ofstream aof(LOG_PYTHON_TIAGOSTEEL.c_str());
  runPython (aof,
      "import dynamic_graph.sot.tiago.steel.prologue",
      *interpreter_);
  if (withWheels_)
    runPython (aof,
        "robot = dynamic_graph.sot.tiago.steel.prologue.makeRobot (with_wheels=True)",
        *interpreter_);
  else
    runPython (aof,
        "robot = dynamic_graph.sot.tiago.steel.prologue.makeRobot (with_wheels=False)",
        *interpreter_);
  aof.close();
}

extern "C" 
{
  dgsot::AbstractSotExternalInterface * createSotExternalInterface()
  {
    return new SoTTiagoSteelController ();
  }
}

extern "C"
{
  void destroySotExternalInterface(dgsot::AbstractSotExternalInterface *p)
  {
    delete p;
  }
}
