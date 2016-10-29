#ifndef ros1ListenerPlugin_H_
#define ros1ListenerPlugin_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1ListenerPlugin.h
/// @class ros1ListenerPlugin
/// @brief ROS1 listener plugin example
/// @details Example listener plugin for ROS1
//
//------------------------------------------------------------------------------
#include <rosPlugins/rosPlugins_config.h>
#include <ros1PluginBase/ecRos1PluginBase.h>
#include <ros/ros.h>

class EC_ROSPLUGINS_ROS1LISTENERPLUGIN_DECL ros1ListenerPlugin : public EcRos1PluginBase
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
    ros1ListenerPlugin
       (
       );

    ros::Subscriber  m_Subscriber; ///< ROS1 subscriber class.
};

#endif // ros1ListenerPlugin_H_
