#ifndef ros1TalkerPlugin_H_
#define ros1TalkerPlugin_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1TalkerPlugin.h
/// @class ros1TalkerPlugin
/// @brief ROS1 talker plugin example
/// @details Example talker plugin for ROS1
//
//------------------------------------------------------------------------------
#include <rosPlugins/rosPlugins_config.h>
#include <ros1PluginBase/ecRos1PluginBase.h>
#include <std_msgs/Float64.h>

class EC_ROSPLUGINS_ROS1TALKERPLUGIN_DECL ros1TalkerPlugin : public EcRos1PluginBase
{
public:
   /// @copydoc Ec::Plugin::update
   virtual void update
      (
      const EcReal time
      );

protected:
   /// @copydoc EcRos1PluginBase::initRos
   virtual EcBoolean initRos
      (
      );

   /// Constructor.
   ros1TalkerPlugin
   (
   );

   ros::Publisher               m_Publisher;   ///< ROS1 publisher
   std_msgs::Float64::Ptr       m_pMsg;        ///< A pointer to message data to publish.

};

#endif // ros1TalkerPlugin_H_
