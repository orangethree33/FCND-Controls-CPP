#pragma once


#include "SandboxController.h"
#include "AttitudeController.h"
#include "FullCascadedController.h"

inline ControllerHandle CreateController(string controllerType, string config)
{
  ControllerHandle ret;

  if (controllerType == "SandboxController")
  {
    ret.reset(new SandboxController(config));
  }

  else if (controllerType == "AttitudeController")
  {
    ret.reset(new AttitudeController(config));
  }

  else if (controllerType == "FullCascadedController")
  {
    ret.reset(new FullCascadedController (config));
  }
  
  return ret;
}