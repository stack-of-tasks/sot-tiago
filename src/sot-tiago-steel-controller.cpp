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

#include <sot/core/debug.hh>

/* TiagoSteel is the instance of TIAGO named "steel" */
#define ROBOTNAME std::string("TIAGOSTEEL")

#include "sot-tiago-steel-controller.hh"

const std::string SoTTiagoSteelController::LOG_PYTHON_TIAGOSTEEL="/tmp/TiagoSteelController_python.out";

SoTTiagoSteelController::SoTTiagoSteelController():
  SoTTiagoController(ROBOTNAME)
{
  startupPython();
  interpreter_->startRosService ();
}

void SoTTiagoSteelController::startupPython()
{
  SoTTiagoController::startupPython();
  std::ofstream aof(LOG_PYTHON_TIAGOSTEEL.c_str());
  runPython
    (aof,
     "from dynamic_graph_sot_tiago.steel.prologue import robot",
     *interpreter_);
  aof.close();
}

extern "C" 
{
  dgsot::AbstractSotExternalInterface * createSotExternalInterface()
  {
    return new SoTTiagoSteelController;
  }
}

extern "C"
{
  void destroySotExternalInterface(dgsot::AbstractSotExternalInterface *p)
  {
    delete p;
  }
}
