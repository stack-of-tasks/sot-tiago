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
 *
 */

#ifndef _SOT_TIAGOSTEEL_Controller_H_
#define _SOT_TIAGOSTEEL_Controller_H_

#include "sot-tiago-controller.hh"
namespace dgsot=dynamicgraph::sot;

class SoTTiagoSteelController: public SoTTiagoController
{
 public:
  static const std::string LOG_PYTHON_TIAGOSTEEL;

  SoTTiagoSteelController(bool withWheels);
  virtual ~SoTTiagoSteelController() {};


 protected:

  virtual void startupPython();
  
  bool withWheels_;

};

#endif /* _SOTTiagoController_H_ */
