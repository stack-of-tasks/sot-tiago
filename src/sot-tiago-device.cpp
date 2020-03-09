/*
 * Copyright 2016,
 *
 * Olivier Stasse
 * Rohan Budhiraja
 *
 * LAAS, CNRS
 *
 * This file is part of TiagoController.
 * TIAGOController is a free software,
 *
 */

#include <fstream>
#include <map>
#include <stdlib.h>

#if DEBUG
#define ODEBUG(x) std::cout << x << std::endl
#else
#define ODEBUG(x)
#endif
#define ODEBUG3(x) std::cout << x << std::endl

#define DBGFILE "/tmp/sot-tiago-device.txt"

#if 1
#define RESETDEBUG5() { std::ofstream DebugFile;	\
    DebugFile.open(DBGFILE,std::ofstream::out);		\
    DebugFile.close();}
#define ODEBUG5FULL(x) { std::ofstream DebugFile;	\
    DebugFile.open(DBGFILE,std::ofstream::app);		\
    DebugFile << __FILE__ << ":"			\
	      << __FUNCTION__ << "(#"			\
	      << __LINE__ << "):" << x << std::endl;	\
    DebugFile.close();}
#define ODEBUG5(x) { std::ofstream DebugFile;	\
    DebugFile.open(DBGFILE,std::ofstream::app); \
    DebugFile << x << std::endl;		\
    DebugFile.close();}

#else
// Void the macro
#define RESETDEBUG5()
#define ODEBUG5FULL(x)
#define ODEBUG5(x)
#endif

#include <sot/core/debug.hh>

#include "sot-tiago-device.hh"
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>

using namespace std;

Eigen::Matrix3d rpy (const double roll, double pitch, double yaw)
{
  Eigen::Matrix3d R;
  // Build rotation matrix as a vector of colums
  R (0,0) = cos(pitch)*cos(yaw);
  R (1,0) = cos(pitch)*sin(yaw);
  R (2,0) = -sin(pitch);

  R (0,1) = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);
  R (1,1) = sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw);
  R (2,1) = sin(roll)*cos(pitch);

  R (0,2) = cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
  R (1,2) = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
  R (2,2) = cos(roll)*cos(pitch);

  return R;
}

const double SoTTiagoDevice::TIMESTEP_DEFAULT = 0.001;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SoTTiagoDevice,"DeviceTiago");

SoTTiagoDevice::SoTTiagoDevice(std::string RobotName):
  dgsot::Device(RobotName),
  closedLoop_ (false),
  previousState_ (),
  baseff_ (),
  accelerometerSOUT_
  ("StackOfTasks(" + RobotName + ")::output(vector)::accelerometer"),
  gyrometerSOUT_ ("StackOfTasks(" + RobotName + ")::output(vector)::gyrometer"),
  currentSOUT_ ("StackOfTasks(" + RobotName + ")::output(vector)::currents"),
  p_gainsSOUT_ ("StackOfTasks(" + RobotName + ")::output(vector)::p_gains"),
  d_gainsSOUT_ ("StackOfTasks(" + RobotName + ")::output(vector)::d_gains"),
  dgforces_ (6),
  pose (),
  accelerometer_ (3),
  gyrometer_ (3),
  torques_(),
  leftWheelIdx_ (-1),
  rightWheelIdx_ (-1)
{
  RESETDEBUG5();
  timestep_ = TIMESTEP_DEFAULT;
  sotDEBUGIN(25) ;
  for( int i=0;i<4;++i ) { withForceSignals[i] = true; }
  signalRegistration (accelerometerSOUT_ << gyrometerSOUT_
                      << currentSOUT_ << p_gainsSOUT_ << d_gainsSOUT_);
  dg::Vector data (3); data.setZero ();
  accelerometerSOUT_.setConstant (data);
  gyrometerSOUT_.setConstant (data);
  baseff_.resize(7);
  using namespace dynamicgraph::command;
  std::string docstring;
  /* Command increment. */
  docstring =
      "\n"
      "    Integrate dynamics for time step provided as input\n"
      "\n"
      "      take one floating point number as input\n"
      "\n";
  addCommand("increment",
             makeCommandVoid1((Device&)*this,
                              &Device::increment, docstring));

  docstring =
      "    Set the integration in closed loop (express mobile base velocity in odometry frame)\n"
      "\n"
      "      - Input: boolean\n"
      "\n";
  addCommand("setClosedLoop",
             makeCommandVoid1(*this,
                              &SoTTiagoDevice::setClosedLoop, docstring));
  sotDEBUGOUT(25);
}

void SoTTiagoDevice::setLeftWheelIndex(int idx)
{
  leftWheelIdx_ = idx;
}

void SoTTiagoDevice::setRightWheelIndex(int idx)
{
  rightWheelIdx_ = idx;
}

SoTTiagoDevice::~SoTTiagoDevice()
{ }

void SoTTiagoDevice::setSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  sotDEBUGIN(25) ;
  map<string,dgsot::SensorValues>::iterator it;
  int t = stateSOUT.getTime () + 1;
  bool setRobotState = false;

  it = SensorsIn.find("forces");
  if (it!=SensorsIn.end())
  {

    // Implements force recollection.
    const vector<double>& forcesIn = it->second.getValues();
    assert (std::div(forcesIn.size(), 6).rem == 0);
    int K = (int)forcesIn.size() / 6;
    for(int i=0;i<K;++i)
    {
      for(int j=0;j<6;++j)
        dgforces_(j) = forcesIn[i*6+j];
      forcesSOUT[i]->setConstant(dgforces_);
      forcesSOUT[i]->setTime (t);
    }
  }

  //TODO: Confirm if this can be made quaternion
  it = SensorsIn.find("attitude");
  if (it!=SensorsIn.end())
  {
    const vector<double>& attitude = it->second.getValues ();
    for (unsigned int i = 0; i < 3; ++i)
      for (unsigned int j = 0; j < 3; ++j)
        pose (i, j) = attitude [i * 3 + j];
    attitudeSOUT.setConstant (pose);
    attitudeSOUT.setTime (t);
  }

  it = SensorsIn.find("motor-angles");
  if (it!=SensorsIn.end())
  {
    const vector<double>& anglesIn = it->second.getValues();
    dgRobotState_.resize (anglesIn.size () + 6);
    for (unsigned i = 0; i < 6; ++i)
      dgRobotState_ (i) = 0.;
    for (unsigned i = 0; i < anglesIn.size(); ++i)
      dgRobotState_ (i + 6) = anglesIn[i];
    setRobotState = true;
  }

  it = SensorsIn.find("odometry");
  if (it!=SensorsIn.end())
  {
    const vector<double>& odomIn = it->second.getValues();
    dgRobotState_ (0) = odomIn[0];
    dgRobotState_ (1) = odomIn[1];
    dgRobotState_ (5) = odomIn[2];
    setRobotState = true;
  }

  it = SensorsIn.find("accelerometer_0");
  if (it!=SensorsIn.end())
  {
    const vector<double>& accelerometer =
        SensorsIn ["accelerometer_0"].getValues ();
    for (std::size_t i=0; i<3; ++i)
      accelerometer_ (i) = accelerometer [i];
    accelerometerSOUT_.setConstant (accelerometer_);
    accelerometerSOUT_.setTime (t);
  }

  it = SensorsIn.find("gyrometer_0");
  if (it!=SensorsIn.end())
  {
    const vector<double>& gyrometer = SensorsIn ["gyrometer_0"].getValues ();
    for (std::size_t i=0; i<3; ++i)
      gyrometer_ (i) = gyrometer [i];
    gyrometerSOUT_.setConstant (gyrometer_);
    gyrometerSOUT_.setTime (t);
  }

  it = SensorsIn.find("torques");
  if (it!=SensorsIn.end())
  {
    const std::vector<double>& torques = SensorsIn["torques"].getValues();
    torques_.resize(torques.size());
    for(std::size_t i = 0; i < torques.size(); ++i)
      torques_ (i) = torques [i];
    pseudoTorqueSOUT.setConstant(torques_);
    pseudoTorqueSOUT.setTime(t);
  }

  it = SensorsIn.find("currents");
  if (it!=SensorsIn.end())
  {
    const std::vector<double>& currents = SensorsIn["currents"].getValues();
    currents_.resize(currents.size());
    for(std::size_t i = 0; i < currents.size(); ++i)
      currents_ (i) = currents[i];
    currentSOUT_.setConstant(currents_);
    currentSOUT_.setTime(t);
  }

  it = SensorsIn.find("p_gains");
  if (it!=SensorsIn.end())
  {
    const std::vector<double>& p_gains = SensorsIn["p_gains"].getValues();
    p_gains_.resize(p_gains.size());
    for(std::size_t i = 0; i < p_gains.size(); ++i)
      p_gains_ (i) = p_gains[i];
    p_gainsSOUT_.setConstant(p_gains_);
    p_gainsSOUT_.setTime(t);
  }

  it = SensorsIn.find("d_gains");
  if (it!=SensorsIn.end())
  {
    const std::vector<double>& d_gains = SensorsIn["d_gains"].getValues();
    d_gains_.resize(d_gains.size());
    for(std::size_t i = 0; i < d_gains.size(); ++i)
      d_gains_ (i) = d_gains[i];
    d_gainsSOUT_.setConstant(d_gains_);
    d_gainsSOUT_.setTime(t);
  }

  if (setRobotState) {
    robotState_.setConstant(dgRobotState_);
    robotState_.setTime(t);
  }

  sotDEBUGOUT(25);
}

void SoTTiagoDevice::setupSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  setSensors (SensorsIn);

  setState (robotState_);
}

void SoTTiagoDevice::nominalSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  setSensors (SensorsIn);
}


void SoTTiagoDevice::cleanupSetSensors(map<string, dgsot::SensorValues> &SensorsIn)
{
  setSensors (SensorsIn);
}

void SoTTiagoDevice::getControl(map<string,dgsot::ControlValues> &controlOut)
{
  ODEBUG5FULL("start");
  sotDEBUGIN(25) ;
  vector<double> anglesOut;

  Eigen::Matrix3d R;
  if (closedLoop_)
    R = rpy(dgRobotState_[3],dgRobotState_[4],dgRobotState_[5]);
  else
    R = rpy(state_[3],state_[4],state_[5]);

  // Integrate control
  increment(timestep_);
  sotDEBUG (25) << "state = " << state_ << std::endl;
  sotDEBUG (25) << "diff  = " << ((previousState_.size() == state_.size())?
				  (state_ - previousState_) : state_ )
		<< std::endl;
  ODEBUG5FULL("state = "<< state_);
  ODEBUG5FULL("diff  = " << ((previousState_.size() == state_.size())?
			     (state_ - previousState_) : state_ ) );
  previousState_ = state_;

  // Specify the joint values for the controller.
  anglesOut.resize(state_.size()-6);

  for(unsigned int i=6; i < state_.size();++i)
    anglesOut[i-6] = state_(i);
  bool hasWheels = (leftWheelIdx_ >= 0 && rightWheelIdx_ >= 0);
  if (hasWheels) {
    // Control wheels in velocity.
    // 6 and 7 correspond to left and right wheel joints.
    anglesOut[0] = vel_control_(leftWheelIdx_);
    anglesOut[1] = vel_control_(rightWheelIdx_);
  }
  controlOut["control"].setValues(anglesOut);
  // Read zmp reference from input signal if plugged
  if (zmpSIN.isPlugged()) {
    int time = controlSIN.getTime ();
    zmpSIN.recompute (time + 1);
    // Express ZMP in free flyer reference frame
    dg::Vector zmpGlobal (4);
    for (unsigned int i = 0; i < 3; ++i)
      zmpGlobal(i) = zmpSIN(time + 1)(i);
    zmpGlobal(3) = 1.;
    dgsot::MatrixHomogeneous inversePose;

    inversePose = freeFlyerPose().inverse(Eigen::Affine);
    dg::Vector localZmp(4); localZmp = inversePose.matrix() * zmpGlobal;
    vector<double> ZMPRef(3);
    for(unsigned int i=0;i<3;++i)
      ZMPRef[i] = localZmp(i);

    controlOut["zmp"].setName("zmp");
    controlOut["zmp"].setValues(ZMPRef);
  }

  // Update position of freeflyer in global frame
  Eigen::Vector3d transq_(freeFlyerPose().translation());
  dg::sot::VectorQuaternion qt_(freeFlyerPose().linear());

  //translation
  for(int i=0; i<3; i++) baseff_[i] = transq_(i);

  //rotation: quaternion
  baseff_[3] = qt_.w();
  baseff_[4] = qt_.x();
  baseff_[5] = qt_.y();
  baseff_[6] = qt_.z();

  controlOut["baseff"].setValues(baseff_);
  ODEBUG5FULL("end");
  sotDEBUGOUT(25) ;
}
