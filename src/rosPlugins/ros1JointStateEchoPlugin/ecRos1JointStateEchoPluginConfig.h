#ifndef ecRos1JointStateEchoPluginConfig_H_
#define ecRos1JointStateEchoPluginConfig_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file EcRos1JointStateEchoPluginConfig.h
/// @brief Class to hold a serializable ROS1 Joint State Echo configuration.
//
//------------------------------------------------------------------------------
#include <rosPlugins/rosPlugins_config.h>
#include <xml/ecXmlBasicType.h>
#include <xml/ecXmlCompType.h>

/// Class to hold a serializable ROS1 Robot State Publisher configuration.
class EC_ROSPLUGINS_ROS1JOINTSTATEECHOPLUGIN_DECL EcRos1JointStateEchoPluginConfig : public EcXmlCompoundType
{
public:
   ECXMLOBJECT(EcRos1JointStateEchoPluginConfig);

   void registerComponents
      (
      );

   EcXmlReal   publish_frequency;            ///< ROS publish_frequency. Defaults to zero or simulation rate (loaded on the parameter server).
   EcXmlU32    queueSize;                    ///< Size of ROS publish/subscribe queue.
   EcXmlString feedbackJointStatesTopicName; ///< ROS topic name for joint states feedback. Default is "joint_states".
   EcXmlString publishJointStatesTopicName;  ///< ROS topic name for publishing joint states. Default is "actin_joint_states".
};

#endif // ecRos1JointStateEchoPluginConfig_H_
