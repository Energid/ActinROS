#ifndef ros1JointStateEchoPlugin_H_
#define ros1JointStateEchoPlugin_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1JointStateEchoPlugin.h
/// @class ros1JointStateEchoPlugin
/// @brief Subscribe to Actin joint states and publish to feedback joint states.
/// @details This ROS1 node will echo the Actin joint states messages to the
/// feedback joint states. The topic names are configurable but the defaults
/// are "actin_joint_states" and "joint_states" respectively.
//
//------------------------------------------------------------------------------
#include <rosPlugins/rosPlugins_config.h>
#include <ros1PluginBase/ecRos1PluginBase.h>
#include <sensor_msgs/JointState.h>
#include "ecRos1JointStateEchoPluginConfig.h"

#include <map>

typedef std::map<EcString, sensor_msgs::JointState::Ptr> EcStringROSJointStatesMap;

class EC_ROSPLUGINS_ROS1JOINTSTATEECHOPLUGIN_DECL ros1JointStateEchoPlugin : public EcRos1PluginBase
{
public:
   /// Get configuration object
   /// @return EcRos1RobotStatePubSubPluginConfig configuration (return by value for thread safety)
   virtual EcRos1JointStateEchoPluginConfig configuration
      (
      ) const;
    
   /// Set configuration object
   /// @param[in] configuration configuration to set
   /// @return EcBoolean Success or failure
   virtual EcBoolean setConfiguration
      (
      const EcRos1JointStateEchoPluginConfig& configuration
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
    
   /// @copydoc Ec::Plugin::update
   virtual void update
      (
      const EcReal time
      );

   /// @copydoc Ec::Plugin::initState
   EcBoolean initState
      (
      );

protected:

   /// @copydoc EcRos1PluginBase::initRos
   virtual EcBoolean initRos
      (
      );
    
   /// Constructor.
   ros1JointStateEchoPlugin
     (
     );

   /// Callback method for setting joint placements for non-active manipulators.
   /// Actin will use the "actin_joint_states" topic name for publication/subscription.
   /// @param[in] state Const pointer to joint states (frame ID = /tf_prefix/manipulatorName)
   virtual void actinJointStateCallback
      (
      sensor_msgs::JointState::ConstPtr state
      );

   boost::scoped_ptr<EcRos1JointStateEchoPluginConfig> m_ConfigurationPtr;          ///< Scoped pointer to configuration parameters.
   ros::Subscriber                                     m_ActinJointStatesReader;    ///< ROS joint state subscriber.
   EcStringROSJointStatesMap                           m_JointStatesMap;            ///< Joint states map for publication.
   ros::Publisher                                      m_JointStatesWriter;         ///< ROS joint state publisher.
   mutable EcMutex                                     m_Mutex;                     ///< Handle multi-threaded access to configuration parameters.
};

#endif // ros1JointStateEchoPlugin_H_
