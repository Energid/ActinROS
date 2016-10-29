//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1ListenerPlugin.cpp
//
//------------------------------------------------------------------------------
#include "ros1ListenerPlugin.h"
#include <std_msgs/Float64.h>

EC_PLUGIN_STUB_DEFAULT(ros1ListenerPlugin)

//---
// Anonymous namespace
//---
namespace
{
   void callback
     (
     std_msgs::Float64::ConstPtr msg
     )
   {
      EcPrint(None) << "Received publisher time " << msg->data << std::endl;
   }
}

//------------------------------------------------------------------------------
ros1ListenerPlugin::ros1ListenerPlugin
   (
   ) :
EcRos1PluginBase(),
m_Subscriber()
{
}

//------------------------------------------------------------------------------
EcBoolean ros1ListenerPlugin::initRos
   (
   )
{
   try
   {

      // Create a subscriber
      m_Subscriber = m_pNodeHandle->subscribe("chatter", 7, callback);

   }
   catch(ros::Exception error)
   {
      EcPrint(Error) << "Failed to load plugin with exception " << error.what() << std::endl;
      return EcFalse;
   }

   return EcTrue;
}

//------------------------------------------------------------------------------
void ros1ListenerPlugin::update
   (
   const EcReal time
   )
{
   (void)time;
    ros::spinOnce();
}


