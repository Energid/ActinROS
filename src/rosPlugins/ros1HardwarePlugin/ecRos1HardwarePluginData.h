#ifndef ecRos1HardwarePluginData_H_
#define ecRos1HardwarePluginData_H_
//     Copyright (c) 2016 Energid Technologies. All rights reserved. ////
//
// Filename:    ecRos1HardwarePluginData.h
//
// Contents:    EcRos1HardwarePluginData class
//
/////////////////////////////////////////////////////////////////////////
#include <rosPlugins/rosPlugins_config.h>  // Required to be first header.
#include <hardwarePlugin/ecHardwarePluginData.h>
#include "ecRos1HardwareManipulatorConfig.h"

class EC_ROSPLUGINS_ROS1HARDWAREPLUGIN_DECL EcRos1HardwarePluginData : public Ec::HardwarePluginData
{
public:
   ECXMLOBJECT(EcRos1HardwarePluginData);
   ECXML_XMLOBJECTCREATOR(EcRos1HardwarePluginData);

   /// @copydoc EcXmlCompoundType::registerComponents()
   void registerComponents
      (
      );

   EcXmlReal                        publish_frequency;            ///< ROS publish_frequency. Defaults to zero or simulation rate (loaded on the parameter server).
   EcRos1HardwareManipulatorConfig  rosManipulatorConfig;         ///< Name of manipulator as seen by ROS (usually manipulator name with dashes & spaces replaced with underscores).
   EcXmlString                      feedbackJointStatesTopicName; ///< ROS topic name for joint states feedback. Default is "joint_states".
   EcXmlString                      publishJointStatesTopicName;  ///< ROS topic name for publishing joint states. Default is "actin_joint_states".
};

#endif // ecRos1HardwarePluginData_H_
