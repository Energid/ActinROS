//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1CytonSlavePlugin.cpp
//
//------------------------------------------------------------------------------
#include <plugins/ecIOParams.h>

#include "ros1CytonSlavePlugin.h"

EC_PLUGIN_STUB_DEFAULT(ros1CytonSlavePlugin)

//------------------------------------------------------------------------------
ros1CytonSlavePlugin::ros1CytonSlavePlugin
   (
   ) :
EcRos1PluginBase(),
m_pSlaveJointValues(boost::make_shared<std_msgs::Float64MultiArray>()),
m_pMasterEe(boost::make_shared<std_msgs::Float64MultiArray>())
{
   m_UpdatePeriodInMs = 40; // 25 Hz
}

//------------------------------------------------------------------------------
EcBoolean ros1CytonSlavePlugin::initRos
   (
   )
{
   try
   {
      // Create a publisher
      m_Publisher = m_pNodeHandle->advertise<std_msgs::Float64MultiArray>("CytonSlaveJoints",7);

      // Subscribe
      m_Subscriber = m_pNodeHandle->subscribe("CytonMasterEe", 7, &ros1CytonSlavePlugin::subscriberCallback, this);

   }
   catch(ros::Exception error)
   {
      EcPrint(Error) << "Failed to load plugin with exception " << error.what() << std::endl;
      return EcFalse;
   }

   return EcTrue;
}

//------------------------------------------------------------------------------
void ros1CytonSlavePlugin::update
   (
   const EcReal time
   )
{
   if (m_pSlaveJointValues && ros::ok())
   {
      // Clear old joint data
      m_pSlaveJointValues->data.clear();

      // Get joint data from manipulator
      getParam<Ec::JointAngle>(0, m_pSlaveJointValues->data);

      // Publish joint values
      m_Publisher.publish(m_pSlaveJointValues);
      ros::spinOnce();
   }
   else
   {
      EcPrint(Error) << "Cannot publish data" << std::endl;
   }
}

//------------------------------------------------------------------------------
void ros1CytonSlavePlugin::subscriberCallback
   (
   std_msgs::Float64MultiArray::ConstPtr msg
   )
{
      // Set translation and orientation of Ee
      m_EcMasterEe.setTranslation(EcVector(msg->data[0], msg->data[1], msg->data[2]));
      m_EcMasterEe.setOrientation(EcOrientation(msg->data[3], msg->data[4], msg->data[5], msg->data[6]));

      for (EcU32 ii = 0; ii < msg->data.size(); ++ii)
      {
         EcPrint(None) << "End-effector translation: " << m_EcMasterEe.translation() << std::endl;
      }

      // Set end-effector
      setParam<Ec::DesiredEndEffector, EcCoordinateSystemTransformation>(0, 0, m_EcMasterEe);

}

