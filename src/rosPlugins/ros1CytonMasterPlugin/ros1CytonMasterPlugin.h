#ifndef ros1CytonMasterPlugin_H_
#define ros1CytonMasterPlugin_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1CytonMasterPlugin.h
/// @class ros1CytonInterfacePlugin
/// @brief ROS1 publisher to publish Cyton joint positions
/// @details This ROS1 node published the frame end-effector of the manipualtor, and
///          update the simulation state with received joint values from the slave node.
//
//------------------------------------------------------------------------------
#include <rosPlugins/rosPlugins_config.h>  // Required to be first header.
#include <foundCommon/ecCoordSysXForm.h>

#include <ros1PluginBase/ecRos1PluginBase.h>
#include <std_msgs/Float64MultiArray.h>


class EC_ROSPLUGINS_ROS1CYTONMASTERPLUGIN_DECL ros1CytonMasterPlugin : public EcRos1PluginBase
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
   ros1CytonMasterPlugin
      (
      );

   ros::Publisher                     m_Publisher;             ///< ROS1 publisher
   ros::Subscriber                    m_Subscriber;            ///< ROS1 subscriber

   std_msgs::Float64MultiArray::Ptr   m_pMasterEe;             ///< Pointer to a float array that stores EE position from simulation and is published by the ROS node

   EcCoordinateSystemTransformation   m_EcMasterEe;            ///< Frame end-effector of master manipulator
};

#endif // ros1CytonMasterPlugin_H_
