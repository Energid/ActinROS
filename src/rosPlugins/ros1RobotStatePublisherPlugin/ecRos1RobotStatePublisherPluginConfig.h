#ifndef ecRos1RobotStatePublisherPluginConfig_H_
#define ecRos1RobotStatePublisherPluginConfig_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecRos1RobotStatePublisherPluginConfig.h
/// @brief Class to hold a serializable ROS1 Robot State Publisher configuration.
//
//------------------------------------------------------------------------------
#include "ecRos1RobotStatePublisherManipulatorConfig.h"

/// Class to hold a serializable ROS1 Robot State Publisher configuration.
class EC_ROSPLUGINS_ROS1ROBOTSTATEPUBLISHERPLUGIN_DECL EcRos1RobotStatePublisherPluginConfig : public EcXmlCompoundType
{
public:
   ECXMLOBJECT(EcRos1RobotStatePublisherPluginConfig);

   void registerComponents
      (
      );

   EcXmlReal   publish_frequency;            ///< ROS publish_frequency. Defaults to zero or simulation rate (loaded on the parameter server).
   EcXmlString rosWorldFrameName;            ///< ROS world frame name used for base transforms (default is "world").
   EcXmlString feedbackJointStatesTopicName; ///< ROS topic name for joint states feedback. Default is "joint_states".
   EcRos1RobotStatePublisherManipulatorConfigVector manipulatorConfigs;  ///< Manipulator configuration
};

#endif // ecRos1RobotStatePublisherPluginConfig_H_
