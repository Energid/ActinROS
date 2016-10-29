#ifndef ecRos1RobotControllerPluginConfig_H_
#define ecRos1RobotControllerPluginConfig_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecRos1RobotControllerPluginConfig.h
/// @brief Class to hold a serializable ROS1 Robot Controller configuration.
//
//------------------------------------------------------------------------------
#include <ros1HardwarePlugin/ecRos1HardwarePluginData.h>
#include <xml/ecXmlVectorType.h>

typedef EcXmlVectorType<EcRos1HardwareManipulatorConfig> EcRos1HardwareManipulatorConfigVector;

/// Class to hold a serializable ROS1 Robot Controller configuration.
class EC_ROSPLUGINS_ROS1ROBOTCONTROLLERPLUGIN_DECL EcRos1RobotControllerPluginConfig : public EcRos1HardwarePluginData
{
public:
   ECXMLOBJECT(EcRos1RobotControllerPluginConfig);
   ECXML_XMLOBJECTCREATOR(EcRos1RobotControllerPluginConfig);

   void registerComponents
      (
      );

   EcXmlString                           rosWorldFrameName;   ///< ROS world frame name used for base transforms (default is "world").
   EcRos1HardwareManipulatorConfigVector manipulatorConfigs;  ///< Manipulator configuration
};

#endif // ecRos1RobotControllerPluginConfig_H_
