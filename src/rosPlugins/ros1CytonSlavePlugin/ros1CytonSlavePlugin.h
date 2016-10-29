#ifndef ros1CytonSlavePlugin_H_
#define ros1CytonSlavePlugin_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1CytonSlavePlugin.h
/// @class ros1CytonSlavePlugin
/// @brief ROS1 slave for a Cyton manipulator
/// @details This ROS1 node receives and sets desired end-effector position and orientation
///          published by another ROS1 node, and publishes the current joint values of the
///          manipulator.
//
//------------------------------------------------------------------------------
#include <rosPlugins/rosPlugins_config.h>  // Required to be first header.
#include <foundCommon/ecCoordSysXForm.h>

#include <ros1PluginBase/ecRos1PluginBase.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>


class EC_ROSPLUGINS_ROS1CYTONSLAVEPLUGIN_DECL ros1CytonSlavePlugin : public EcRos1PluginBase
{
public:
   /// @copydoc Ec::Plugin::update
   virtual void update
      (
      const EcReal time
      );

   /// @brief Subscriber callback function
   void subscriberCallback
      (
      std_msgs::Float64MultiArray::ConstPtr msg
      );


protected:
   /// @copydoc EcRos1PluginBase::initRos
   virtual EcBoolean initRos
      (
      );

   /// Constructor.
   ros1CytonSlavePlugin
      (
      );

   ros::Publisher                     m_Publisher;          ///< ROS1 publisher
   ros::Subscriber                    m_Subscriber;         ///< ROS1 subscriber

   std_msgs::Float64MultiArray::Ptr   m_pSlaveJointValues;  ///< Pointer to a float array that stores joint values from slave node
   std_msgs::Float64MultiArray::Ptr   m_pMasterEe;          ///< Pointer to a float array that stores EE position from simulation

   EcCoordinateSystemTransformation   m_EcMasterEe;         ///< Frame end-effector of master manipulator
};

#endif // ros1CytonSlavePlugin_H_
