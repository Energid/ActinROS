#ifndef ecRos1PubSubManipulatorConfig_H_
#define ecRos1PubSubManipulatorConfig_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecRos1PubSubManipulatorConfig.h
/// @brief Class to hold a serializable ROS1 Publish/Subscribe manipulator configuration.
//
//------------------------------------------------------------------------------
#include <rosPlugins/rosPlugins_config.h>
#include <xml/ecXmlCompType.h>
#include <xml/ecXmlEnumType.h>
#include <xml/ecXmlVectorType.h>

/// Class to hold a serializable DDS Publish/Subscribe manipulator configuration.
class EC_ROSPLUGINS_ROS1PUBSUBPLUGIN_DECL EcRos1PubSubManipulatorConfig : public EcXmlCompoundType
{
public:
   enum ManipulatorMode
   {
      MODE_IGNORE,           // Do not publish or subscribe to anything
      MODE_PUB_EE_PUB_JOINT, // Publish end effectors and joint angles
      MODE_PUB_EE_SUB_JOINT, // Publish end effectors, subscribe to joint angles
      MODE_SUB_EE_PUB_JOINT, // Subscribe to end effectors, publish joint angles
      MODE_SUB_EE,           // Subscribe to end effectors
      MODE_SUB_JOINT         // Subscribe to joint angles
   };

   ECXMLOBJECT(EcRos1PubSubManipulatorConfig);

   void registerComponents
      (
      );

   EcXmlEnumU32 mode; ///< Manipulator mode
};

typedef EcXmlVectorType<EcRos1PubSubManipulatorConfig> EcRos1PubSubManipulatorConfigVector;

#endif // ecRos1PubSubManipulatorConfig_H_
