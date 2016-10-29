//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1TalkerPlugin.cpp
//
//------------------------------------------------------------------------------
#include "ros1TalkerPlugin.h"
#include <exception>

EC_PLUGIN_STUB_DEFAULT(ros1TalkerPlugin)

//------------------------------------------------------------------------------
ros1TalkerPlugin::ros1TalkerPlugin
   (
   ) :
EcRos1PluginBase(),
m_pMsg(boost::make_shared<std_msgs::Float64>())
{
    m_UpdatePeriodInMs = 1000; // 1 second
}

//------------------------------------------------------------------------------
EcBoolean ros1TalkerPlugin::initRos
   (
   )
{
   try
   {
      // Create a publisher
      m_Publisher = m_pNodeHandle->advertise<std_msgs::Float64>("chatter",7);

   }
   catch(std::runtime_error error)
   {
      EcPrint(Error) << "Failed to load plugin with exception " << error.what() << std::endl;
      return EcFalse;
   }

   return EcTrue;
}

//------------------------------------------------------------------------------
void ros1TalkerPlugin::update
   (
   const EcReal time
   )
{
   if (m_pMsg && ros::ok())
   {
      m_pMsg->data = time;
      m_Publisher.publish(m_pMsg);
      ros::spinOnce();
   }
   else
   {
      EcPrint(Error) << "Cannot publish data" << std::endl;
   }
}
