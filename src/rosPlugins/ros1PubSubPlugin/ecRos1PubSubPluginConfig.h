#ifndef ecRos1PubSubPluginConfig_H_
#define ecRos1PubSubPluginConfig_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecRos1PubSubPluginConfig.h
/// @brief Class to hold a serializable ROS1 Publish/Subscribe configuration.
//
//------------------------------------------------------------------------------
#include <rosPlugins/rosPlugins_config.h>
#include "ecRos1PubSubManipulatorConfig.h"

/// Class to hold a serializable ROS1 Publish/Subscribe configuration.
class EC_ROSPLUGINS_ROS1PUBSUBPLUGIN_DECL EcRos1PubSubPluginConfig : public EcXmlCompoundType
{
public:
   ECXMLOBJECT(EcRos1PubSubPluginConfig);

   void registerComponents
      (
      );

   EcXmlString                         jointAnglesTopicName;      ///< Name for joint angles topic
   EcXmlString                         desiredPlacementTopicName; ///< Name for desired placement topic
   EcRos1PubSubManipulatorConfigVector manipulatorConfigs;        ///< Manipulator configuration
};

#endif // ecRos1PubSubPluginConfig_H_
