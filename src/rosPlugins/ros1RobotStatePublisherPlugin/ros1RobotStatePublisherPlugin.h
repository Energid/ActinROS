#ifndef ros1RobotStatePublisherPlugin_H_
#define ros1RobotStatePublisherPlugin_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1RobotStatePublisherPlugin.h
/// @class ros1RobotStatePublisherPlugin
/// @brief Configures and manages ROS1 robot state publisher
/// @details This ROS1 node handles robot state publication.
//
//------------------------------------------------------------------------------
#include <ros1PluginBase/ecRos1PluginBase.h>
#include <foundCommon/ecCoordSysXForm.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include "ecRos1RobotStatePublisherPluginConfig.h"

class EC_ROSPLUGINS_ROS1ROBOTSTATEPUBLISHERPLUGIN_DECL ros1RobotStatePublisherPlugin : public EcRos1PluginBase
{
public:
   /// Get configuration object
   /// @return EcRos1RobotStatePubSubPluginConfig configuration (return by value for thread safety)
   virtual EcRos1RobotStatePublisherPluginConfig configuration
      (
      ) const;
    
   /// Set configuration object
   /// @param[in] configuration configuration to set
   /// @return EcBoolean Success or failure
   virtual EcBoolean setConfiguration
      (
      const EcRos1RobotStatePublisherPluginConfig& configuration
      );
    
   /// @copydoc Ec::Plugin::readConfigurationString(EcXmlReader&)
   EcBoolean readConfigurationString
      (
      EcXmlReader& stream
      );
    
   /// @copydoc Ec::Plugin::writeConfigurationString(EcXmlWriter&) const
   EcBoolean writeConfigurationString
      (
      EcXmlWriter& stream
      ) const;

   /// @copydoc Ec::Plugin::initState()
   EcBoolean initState
      (
      );
    
   /// @copydoc Ec::Plugin::update
   virtual void update
      (
      const EcReal time
      );

protected:
   typedef std::vector<geometry_msgs::TransformStamped>  StampedTransformVector;
   typedef std::vector<StampedTransformVector>           StampedTransformVectorVector;
   typedef std::map<EcString, ros::Time>                 EcStringROSTimeMap;
   typedef std::vector<EcStringROSTimeMap>               EcStringROSTimeMapVector;

   /// @copydoc EcRos1PluginBase::initRos
   virtual EcBoolean initRos
      (
      );
    
   /// Constructor.
   ros1RobotStatePublisherPlugin
     (
     );

   /// Method to setup ROS publications & subscriptions.
   /// @return EcBoolean Success or failure
   virtual EcBoolean setupRobotStatePublicationAndSubscriptions
      (
      );

   /// Callback method for setting joint placements for non-active manipulators.
   /// Actin will use the "actin_joint_states" topic name for publication/subscription.
   /// @param[in] state Const pointer to joint states (frame ID = /tf_prefix/manipulatorName)
   virtual void jointStateCallback
      (
      sensor_msgs::JointState::ConstPtr state
      );

   /// Callback method for setting base positions.
   /// The transformation lookup expects the source frame ID to be "world".
   /// The target frame is /tf_prefix/manipulatorName/baseLinkName.
   /// @param[in] manipulatorId Manipulator ID
   /// @param[in] position ROS tf::Transform providing updated position & orientation.
   virtual void basePositionCallback
      (
      EcU32 manipulatorId,
      const tf::StampedTransform& position
      );

   /// Method for publishing transforms.
   /// The top frame ID is "world", child link is /tf_prefix/manipulatorName/baseLinkName.
   /// Thereafter the frame ID is /tf_prefix/manipulatorName/baseLinkName,
   /// child link /tf_prefix/manipulatorName/link0, etc.
   /// @param[in] manipulatorId Manipulator ID
   /// @param[in] positions Joint positions
   /// @param[in] time ROS time stamp of join_states message.
   virtual void publishTransforms
      (
      EcU32 manipulatorID,
      const EcRealVector &positions,
      const ros::Time& time
      );

   EcStringVector                                           m_ManipulatorNames;           ///< Manipulator names within model.
   StampedTransformVectorVector                             m_ManipulatorTransforms;      ///< Stores all ROS transforms for the active manipulator.
   EcCoordinateSystemTransformation                         m_Transformation;             ///< Helper transformation created on construction.
   EcCoordinateSystemTransformation                         m_BaseCallbackTransformation; ///< Helper transformation created on construction.
   EcOrientation                                            m_BaseCallbackOrientation;    ///< Helper orientation created on construction.
   EcStringU32Map                                           m_BaseLinkNamesMap;           ///< Map of base link names to manipulator index.
   boost::scoped_ptr<EcRos1RobotStatePublisherPluginConfig> m_ConfigurationPtr;           ///< Scoped pointer to configuration parameters.
   boost::scoped_ptr<tf::TransformBroadcaster>              m_TfBroadcaster;              ///< Scoped pointer to ROS transform broadcaster.
   tf::StampedTransform                                     m_LookupTransform;            ///< Helper ROS transform created on construction.
   boost::scoped_ptr<tf::TransformListener>                 m_TfListener;                 ///< Scoped pointer to ROS transform listener.
   ros::Subscriber                                          m_JointStateReader;           ///< ROS joint state subscriber.
   ros::Duration                                            m_PublishInterval;            ///< Publish interval (inverse of publish_frequency).
   ros::Time                                                m_LastCallbackTime;           ///< Keep track of last callback time.
   EcStringROSTimeMapVector                                 m_LastPublishTime;            ///< Keep track of last publication time.
   mutable EcMutex                                          m_Mutex;                      ///< Handle multi-threaded access to configuration parameters.
};

#endif // ros1RobotStatePublisherPlugin_H_
