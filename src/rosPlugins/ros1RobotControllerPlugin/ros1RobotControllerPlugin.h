#ifndef ros1RobotControllerPlugin_H_
#define ros1RobotControllerPlugin_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1RobotControllerPlugin.h
/// @class ros1RobotControllerPlugin
/// @brief Configures and manages ROS1 robot controller
/// @details This ROS1 node handles robot control.
//
//------------------------------------------------------------------------------
#include <ros1HardwarePlugin/ros1HardwarePlugin.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include "ecRos1RobotControllerPluginConfig.h"

class EC_ROSPLUGINS_ROS1ROBOTCONTROLLERPLUGIN_DECL ros1RobotControllerPlugin : public ros1HardwarePlugin
{
public:
   /// Get configuration object
   /// @return EcRos1RobotControllerPluginConfig configuration (return by value for thread safety)
   virtual EcRos1RobotControllerPluginConfig configuration
      (
      ) const;

   /// Set configuration object
   /// @param[in] configuration configuration to set
   /// @return EcBoolean Success or failure
   virtual EcBoolean setConfiguration
      (
      const EcRos1RobotControllerPluginConfig& configuration
      );

   /// @copydoc Ec::HardwarePlugin::initImplementation()
   Ec::HardwareStatus initImplementation
      (
      );

   /// @copydoc ros1HardwarePlugin::initRos()
   EcBoolean initRos
     (
     );

   /// @copydoc Ec::HardwarePlugin::setToHardwareImplementation()
   Ec::HardwareStatus setToHardwareImplementation
      (
      );

   /// @copydoc Ec::HardwarePlugin::createConfigFile()
   void createConfigFile
      (
      );

protected:
   typedef std::vector<ros::Subscriber>                  SubscriberVector;
   typedef std::vector<ros::Publisher>                   PublisherVector;
   typedef std::vector<geometry_msgs::PoseStamped::Ptr>  PoseStampedPtrVector;

   /// Constructor.
   ros1RobotControllerPlugin
     (
     );

   /// Method for setting up initial position controller isOn state for each manipulator.
   virtual void updateInitialPositionControllerFlags
      (
      );

   /// Method for setting position controller isOn state for each manipulator
   /// according to configuration.
   virtual void updatePositionControllerFlags
      (
      );

   /// Method to setup ROS publications & subscriptions.
   /// @return EcBoolean Success or failure
   virtual EcBoolean setupRobotStatePublicationAndSubscriptions
      (
      );

   /// Callback method for receiving end-effector placements from external sources.
   /// @param[in] placement Const pointer to pose (frame ID = /tf_prefix/manipulatorName/linkName/endEffectorType)
   virtual void endEffectorCallback
      (
      geometry_msgs::PoseStampedConstPtr placement
      );

   /// @copydoc ros1HardwarePlugin::jointStatesCallback()
   void jointStatesCallback
      (
      sensor_msgs::JointState::ConstPtr state
      );

   /// Callback method for setting base positions.
   /// The transformation lookup expects the source frame ID to be "world".
   /// For active manipulators the target frame is /tf_prefix/manipulatorName/baseLinkName/sensed.
   /// For all other maniupulators the target frame is /tf_prefix/manipulatorName/baseLinkName.
   /// @param[in] manipulatorId Manipulator ID
   /// @param[in] position ROS tf::Transform providing updated position & orientation.
   virtual void basePositionCallback
      (
      EcU32 manipulatorId,
      const tf::StampedTransform& position
      );

   EcStringVector                                         m_ManipulatorNames;                      ///< Manipulator names within model.
   EcStringU32Map                                         m_ActiveEePoseNamesMap;                  ///< Map of end-effector topic names to end-effector index.
   EcCoordinateSystemTransformation                       m_Transformation;                        ///< Helper transformation created on construction.
   EcCoordinateSystemTransformation                       m_EECallbackTransformation;              ///< Helper transformation created on construction.
   EcCoordinateSystemTransformation                       m_BaseTransformation;                    ///< Helper transformation created on construction.
   EcCoordinateSystemTransformation                       m_BaseCallbackTransformation;            ///< Helper transformation created on construction.
   EcOrientation                                          m_EECallbackOrientation;                 ///< Helper orientation created on construction.
   EcOrientation                                          m_BaseCallbackOrientation;               ///< Helper orientation created on construction.
   EcStringU32Map                                         m_BaseLinkNamesMap;                      ///< Map of base link names to manipulator index.
   geometry_msgs::TransformStamped                        m_BaseTransform;                         ///< Time stamped transform for publishing base pose.
   EcBooleanVector                                        m_InitialPositionControllerFlagVector;   ///< Initial position controller flags.
   boost::scoped_ptr<tf::TransformBroadcaster>            m_TfBroadcaster;                         ///< Scoped pointer to ROS transform broadcaster.
   tf::StampedTransform                                   m_LookupTransform;                       ///< Helper ROS transform created on construction.
   boost::scoped_ptr<tf::TransformListener>               m_TfListener;                            ///< Scoped pointer to ROS transform listener.
   SubscriberVector                                       m_EndEffectorReaders;                    ///< Vector of ROS subscribers for reading ROS poses.
   PublisherVector                                        m_EndEffectorWriters;                    ///< Vector of ROS publishers for publishing actual POSE position.
   PoseStampedPtrVector                                   m_ActualEndEffectorPoses;                ///< Vector of ROS stamped pose pointers for publishing actual end-effector position.
};

#endif // ros1RobotControllerPlugin_H_
