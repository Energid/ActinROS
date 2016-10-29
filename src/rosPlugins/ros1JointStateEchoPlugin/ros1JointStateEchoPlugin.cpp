//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1JointStateEchoPlugin.cpp
//
//------------------------------------------------------------------------------
#include <plugins/ecIOParams.h>

#include "ros1JointStateEchoPlugin.h"


EC_PLUGIN_STUB_DEFAULT(ros1JointStateEchoPlugin)

//------------------------------------------------------------------------------
ros1JointStateEchoPlugin::ros1JointStateEchoPlugin
   (
   ) :
EcRos1PluginBase(),
m_ConfigurationPtr(new EcRos1JointStateEchoPluginConfig()),
m_ActinJointStatesReader(),
m_JointStatesMap(),
m_JointStatesWriter(),
m_Mutex()
{
   if (m_ConfigurationPtr->publish_frequency.value() != 0.0)
   {
      m_UpdatePeriodInMs = static_cast<EcU32>(1000.0/m_ConfigurationPtr->publish_frequency);
   }
}

//---
// ros1RobotStatePubSubPlugin public methods
//---

//---------------------------------------------------------------------------
EcRos1JointStateEchoPluginConfig ros1JointStateEchoPlugin::configuration
   (
   ) const
{
   EcMutexScopedLock lock(m_Mutex);

   return *m_ConfigurationPtr.get();
}

//---------------------------------------------------------------------------
EcBoolean ros1JointStateEchoPlugin::setConfiguration
   (
   const EcRos1JointStateEchoPluginConfig& configuration
   )
{
   EcMutexScopedLock lock(m_Mutex);

   *m_ConfigurationPtr = configuration;

   return initRos();
}

//---------------------------------------------------------------------------
EcBoolean ros1JointStateEchoPlugin::readConfigurationString
   (
   EcXmlReader& stream
   )
{
   EcRos1JointStateEchoPluginConfig config;
   const EcBoolean success = config.read(stream);
   setConfiguration(config);

   return success;
}

//---------------------------------------------------------------------------
EcBoolean ros1JointStateEchoPlugin::writeConfigurationString
   (
   EcXmlWriter& stream
   ) const
{
   return configuration().write(stream);
}

//------------------------------------------------------------------------------
void ros1JointStateEchoPlugin::update
   (
   const EcReal time
   )
{
   EcMutexScopedLock lock(m_Mutex);

   if (ros::ok())
   {
      for(EcStringROSJointStatesMap::iterator iter = m_JointStatesMap.begin();
          iter != m_JointStatesMap.end(); iter++)
      {
         m_JointStatesWriter.publish(iter->second);
      }
      ros::spinOnce();
   }
   else
   {
      EcPrint(Error) << "Cannot publish data" << std::endl;
   }
}

//------------------------------------------------------------------------------
EcBoolean ros1JointStateEchoPlugin::initState
   (
   )
{
    m_JointStatesMap.clear();
    return EcTrue;
}

//---
// ros1RobotStatePubSubPlugin protected methods
//---

//------------------------------------------------------------------------------
EcBoolean ros1JointStateEchoPlugin::initRos
   (
   )
{
    EcMutexScopedLock lock(m_Mutex);

    if (m_ConfigurationPtr->publish_frequency.value() == 0.0)
    {
       EcReal simUpdateRate = param<Ec::SimulationThreadUpdateRate,EcReal>();
       m_pPrivateNodeHandle->setParam("publish_frequency", simUpdateRate);
    }
    else
    {
       m_pPrivateNodeHandle->setParam("publish_frequency", m_ConfigurationPtr->publish_frequency.value());
       m_UpdatePeriodInMs = static_cast<EcU32>(1000.0/m_ConfigurationPtr->publish_frequency);
    }

    // Subscribe to joint_states.
    try
    {
       // Make queue size twice the number of manipulators.
       m_ActinJointStatesReader = m_pNodeHandle
          ->subscribe(m_ConfigurationPtr->publishJointStatesTopicName,
          m_ConfigurationPtr->queueSize,
          &ros1JointStateEchoPlugin::actinJointStateCallback, this);
    }
    catch(ros::Exception error)
    {
       EcPrint(Error) << "Failed to create ROS subscriber with exception, " << error.what() << std::endl;
       return EcFalse;
    }

    // Publish joint_states.
    try
    {
       m_JointStatesWriter = m_pNodeHandle
          ->advertise<sensor_msgs::JointState>(m_ConfigurationPtr->feedbackJointStatesTopicName,
          m_ConfigurationPtr->queueSize);
    }
    catch(ros::Exception error)
    {
       EcPrint(Error) << "Failed to create ROS publisher with exception, " << error.what() << std::endl;
       return EcFalse;
    }

    return EcTrue;
}

//---------------------------------------------------------------------------
void ros1JointStateEchoPlugin::actinJointStateCallback
   (
   sensor_msgs::JointState::ConstPtr state
   )
{
   if (state->name.size() != state->position.size())
   {
       EcERROR("Robot state publisher received an invalid joint state vector\n");
       return;
   }

   // Copy and set time.
   if (m_JointStatesMap.find(state->header.frame_id) == m_JointStatesMap.end())
   {
       m_JointStatesMap[state->header.frame_id] = boost::make_shared<sensor_msgs::JointState>();
   }
   *m_JointStatesMap[state->header.frame_id] = *state;
   m_JointStatesMap[state->header.frame_id]->header.stamp = ros::Time::now();
}
