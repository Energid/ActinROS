//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1CytonMasterPlugin.cpp
//
//------------------------------------------------------------------------------
#include <plugins/ecIOParams.h>

#include "ros1CytonMasterPlugin.h"

EC_PLUGIN_STUB_DEFAULT(ros1CytonMasterPlugin)

//------------------------------------------------------------------------------
ros1CytonMasterPlugin::ros1CytonMasterPlugin
   (
   ) :
EcRos1PluginBase(),
m_pMasterEe(boost::make_shared<std_msgs::Float64MultiArray>())
{
   m_UpdatePeriodInMs = 40; // 25 Hz
}

//------------------------------------------------------------------------------
EcBoolean ros1CytonMasterPlugin::initRos
   (
   )
{
   try
   {
      // Create a publisher
      m_Publisher = m_pNodeHandle->advertise<std_msgs::Float64MultiArray>("CytonMasterEe",7);

      m_Subscriber = m_pNodeHandle->subscribe("CytonSlaveJoints", 7, &ros1CytonMasterPlugin::subscriberCallback, this);

   }
   catch(ros::Exception error)
   {
      EcPrint(Error) << "Failed to load plugin with exception " << error.what() << std::endl;
      return EcFalse;
   }

   return EcTrue;
}

//------------------------------------------------------------------------------
void ros1CytonMasterPlugin::update
   (
   const EcReal time
   )
{
   if (m_pMasterEe && ros::ok())
   {
      // Clear old joint data
      m_pMasterEe->data.clear();

      // Get joint data from manipulator
      getParam<Ec::DesiredEndEffector, EcCoordinateSystemTransformation>(0, 0, m_EcMasterEe);

      m_pMasterEe->data.push_back(m_EcMasterEe.translation().x());
      m_pMasterEe->data.push_back(m_EcMasterEe.translation().y());
      m_pMasterEe->data.push_back(m_EcMasterEe.translation().z());
      m_pMasterEe->data.push_back(m_EcMasterEe.orientation().w());
      m_pMasterEe->data.push_back(m_EcMasterEe.orientation().x());
      m_pMasterEe->data.push_back(m_EcMasterEe.orientation().y());
      m_pMasterEe->data.push_back(m_EcMasterEe.orientation().z());

      m_Publisher.publish(m_pMasterEe);
      ros::spinOnce();
   }
   else
   {
      EcPrint(Error) << "Cannot publish data" << std::endl;
   }
}

//------------------------------------------------------------------------------
void ros1CytonMasterPlugin::subscriberCallback
   (
   std_msgs::Float64MultiArray::ConstPtr msg
   )
{
      EcPrint(Debug) << "Joint data size " << msg->data.size() << std::endl;
      setParam<Ec::JointAngle, EcRealVector>(0, msg->data);
}
