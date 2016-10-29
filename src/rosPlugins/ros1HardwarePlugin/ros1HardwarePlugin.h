#ifndef ros1HardwarePlugin_H_
#define ros1HardwarePlugin_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1HardwarePlugin.h
/// @class ros1HardwarePlugin
/// @brief ROS1 hardware plugin
/// @details Hardware plugin class for ROS1
//
//------------------------------------------------------------------------------
#include <rosPlugins/rosPlugins_config.h>  // Required to be first header.
#include <hardwarePlugin/ecHardwarePlugin.h>
#include <manipulator/ecIndManipulator.h>
#include <manipulator/ecGravitationalTorqueTool.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


class EC_ROSPLUGINS_ROS1HARDWAREPLUGIN_DECL ros1HardwarePlugin : public Ec::HardwarePlugin
{
public:

   /// @copydoc Ec::HardwarePlugin::initImplementation()
   Ec::HardwareStatus initImplementation
      (
      );

   /// Virtual function called in init for
   /// performing ROS specific initialization
   /// tasks.
   /// @return EcBoolean Success or failure
   virtual EcBoolean initRos
      (
      );

   /// @copydoc Ec::HardwarePlugin::updateDisabledImplementation()
   void updateDisabledImplementation
      (
      );

   /// @copydoc Ec::HardwarePlugin::getFromHardwareImplementation()
   Ec::HardwareStatus getFromHardwareImplementation
      (
      );

   /// @copydoc Ec::HardwarePlugin::setToHardwareImplementation()
   Ec::HardwareStatus setToHardwareImplementation
      (
      );

   /// @copydoc Ec::HardwarePlugin::torqueEnableImplementation()
   Ec::HardwareStatus torqueEnableImplementation
      (
      const EcBoolean enable
      );

   /// @copydoc Ec::HardwarePlugin::createConfigFile()
   void createConfigFile
      (
      );

   /// Return the pointer to the ROS node handle.
   virtual ros::NodeHandlePtr nodeHandle
      (
      ) const;

   /// Return the pointer to the ROS private ("~") node handle.
   virtual ros::NodeHandlePtr privateNodeHandle
      (
      ) const;

protected:
    /// Constructor.
    ros1HardwarePlugin
       (
       );

    /// Callback method for setting joint placements for the active manipulator.
    /// ROS uses the "joint_states" topic name for sensor feedback of joint positions.
    /// @param[in] state Const pointer to joint states (frame ID = /tf_prefix/manipulatorName)
    virtual void jointStatesCallback
       (
       sensor_msgs::JointState::ConstPtr state
       );

    ros::NodeHandlePtr                 m_pNodeHandle;             ///< Pointer to ROS1 node handle.
    ros::NodeHandlePtr                 m_pPrivateNodeHandle;      ///< Pointer to ROS1 private node handle ("~").
    sensor_msgs::JointState            m_CurrentJointState;       ///< Keeps track of current joint state for the selected manipulator.
    sensor_msgs::JointState::Ptr       m_ActinJointStates;        ///< Published Actin joint states for active manipulator.
    ros::Subscriber                    m_JointStatesReader;       ///< ROS joint states subscriber.
    ros::Publisher                     m_ActinJointStatesWriter;  ///< ROS joint states publisher.
    EcRealVector                       m_JointPositions;          ///< Helper joint positions array.
    EcRealVector                       m_JointVelocities;         ///< Helper joint velocties array.
    EcRealVector                       m_JointTorques;            ///< Helper joint torques array.
    EcIndividualManipulator            m_ActiveManipulator;       ///< Helper active individual manipulator for calculating torques.
    EcPositionState                    m_PositionState;           ///< Helper position state class for calculating torques.
    mutable EcGravitationalTorqueTool  m_GravitationalTorqueTool; ///< Utility gravitational-torque tool.
};

#endif // ros1HardwarePlugin_H_
