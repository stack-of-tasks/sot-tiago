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

#ifndef TIAGO_STEEL_WITH_WHEELS
# error "You must define TIAGO_STEEL_WITH_WHEELS to 0 or 1"
#endif

#include <sot/core/debug.hh>

/* TiagoSteel is the instance of TIAGO named "steel" */
#define ROBOTNAME std::string("TIAGOSTEEL")

#include "sot-tiago-steel-controller.hh"

const std::string SoTTiagoSteelController::LOG_PYTHON_TIAGOSTEEL="/tmp/TiagoSteelController_python.out";

SoTTiagoSteelController::SoTTiagoSteelController(bool withWheels):
  SoTTiagoController(ROBOTNAME),
  withWheels_ (withWheels)
{
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
#if TIAGO_STEEL_WITH_WHEELS
    return new SoTTiagoSteelController (true );
#else
    return new SoTTiagoSteelController (false);
#endif
  }
}

extern "C"
{
  void destroySotExternalInterface(dgsot::AbstractSotExternalInterface *p)
  {
    delete p;
  }
}
