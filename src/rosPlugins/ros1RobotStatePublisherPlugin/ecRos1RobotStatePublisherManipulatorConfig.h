#ifndef ecRos1RobotStatePubisherManipulatorConfig_H_
#define ecRos1RobotStatePubisherManipulatorConfig_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecRos1RobotStatePubisherManipulatorConfig.h
/// @brief Class to hold a serializable ROS1 Robot State Publisher manipulator
/// configuration.
//
//------------------------------------------------------------------------------
#include <rosPlugins/rosPlugins_config.h>
#include <xml/ecXmlCompType.h>
#include <xml/ecXmlVectorType.h>

/// Class to hold a serializable ROS1 Robot State Publisher configuration.
class EC_ROSPLUGINS_ROS1ROBOTSTATEPUBLISHERPLUGIN_DECL EcRos1RobotStatePublisherManipulatorConfig : public EcXmlCompoundType
{
public:

   ECXMLOBJECT(EcRos1RobotStatePublisherManipulatorConfig);

   void registerComponents
      (
      );

   EcXmlString manipulatorName;  ///< Name of this manipulator (must match actual manipulator name in the model).
   EcXmlString tf_prefix;        ///< Publish ROS transforms for all links on this manipulator
};

typedef EcXmlVectorType<EcRos1RobotStatePublisherManipulatorConfig> EcRos1RobotStatePublisherManipulatorConfigVector;

#endif // ecRos1RobotStatePubisherManipulatorConfig_H_
