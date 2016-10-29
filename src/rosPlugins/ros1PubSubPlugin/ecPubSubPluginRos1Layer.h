#ifndef ecPubSubPluginRos1Layer_H
#define ecPubSubPluginRos1Layer_H
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecPubSubPluginRos1Layer.h
//
//------------------------------------------------------------------------------
#include <rosPlugins/rosPlugins_config.h>
#include <ros/ros.h>
#include "ManipulatorJointAngles.h"
#include "ManipulatorMessage.h"

#include <control/ecManipEndEffectorPlace.h>

#include <boost/function.hpp>

// Forward declarations
class EcRos1PubSubPluginConfig;
class ros1PubSubPlugin;

//---------------------------------------------------------------------------
class EC_ROSPLUGINS_ROS1PUBSUBPLUGIN_DECL ecPubSubPluginRos1Layer
{
public:
   /// Type of the end effector callback: void (*callback)(EcU32 manipulatorId, const EcManipulatorEndEffectorPlacement& placement)
   typedef boost::function2<void, EcU32, const EcManipulatorEndEffectorPlacement&> EndEffectorCallbackType;

   /// Type of the joint angle callback: void (*callback)(EcU32 manipulatorId, const EcRealVector& jointAngles)
   typedef boost::function2<void, EcU32, const EcRealVector&> JointAngleCallbackType;

   ecPubSubPluginRos1Layer
      (
      ros1PubSubPlugin*       pPlugin,
      EndEffectorCallbackType endEffectorCallback,
      JointAngleCallbackType  jointAngleCallback
      );

   ~ecPubSubPluginRos1Layer
      (
      );

   /// Set configuration object
   /// @param[in] config configuration to set
   /// @return EcBoolean Success or failure
   EcBoolean setConfiguration
     (
     const EcRos1PubSubPluginConfig& config
     );

   EcBoolean hasPublisher
      (
      ) const;

   void publishEndEffectors
      (
      const EcManipulatorEndEffectorPlacementVector& placements
      );

   void publishEndEffectors
      (
      EcU32                                    manipulatorId,
      const EcManipulatorEndEffectorPlacement& placement
      );

   void publishJointAngles
      (
      const EcRealVectorVector& jointAngles
      );

   void publishJointAngles
      (
      EcU32               manipulatorId,
      const EcRealVector& jointAngles
      );

private:
   typedef EcRos1::ManipulatorMessage::Ptr       EndEffectorSamplePtr;
   typedef std::vector<EndEffectorSamplePtr>     EndEffectorSamplePtrVector;

   typedef EcRos1::ManipulatorJointAngles::Ptr   JointAngleSamplePtr;
   typedef std::vector<JointAngleSamplePtr>      JointAngleSamplePtrVector;

   void endEffectorDataCallback
      (
      EcRos1::ManipulatorMessage::ConstPtr sample
      );

   void jointAngleDataCallback
      (
      EcRos1::ManipulatorJointAngles::ConstPtr sample
      );

   EcBoolean isEEPublisherNeeded
      (
      const EcRos1PubSubPluginConfig& config
      ) const;
    
   EcBoolean isJointPublisherNeeded
      (
      const EcRos1PubSubPluginConfig& config
      ) const;

   EcBoolean isEESubscriberNeeded
      (
      const EcRos1PubSubPluginConfig& config
      ) const;
   
   EcBoolean isJointSubscriberNeeded
      (
      const EcRos1PubSubPluginConfig& config
      ) const;
   
   void destroyEndEffectorReader
      (
      );
    
   void destroyEndEffectorWriter
      (
      );
   
   EndEffectorSamplePtr endEffectorSample
      (
      EcU32 manipulatorId
      );
   
   void destroyEndEffectorSample
      (
      EcU32 manipulatorId
      );
    
   void destroyEndEffectorSamples
      (
      );
    
   void destroyJointAngleReader
      (
      );
    
   void destroyJointAngleWriter
      (
      );
   
   JointAngleSamplePtr jointAngleSample
      (
      EcU32 manipulatorId
      );
   
   void destroyJointAngleSample
      (
      EcU32 manipulatorId
      );
    
   void destroyJointAngleSamples
      (
      );

   EndEffectorCallbackType    m_EndEffectorCallback;
   JointAngleCallbackType     m_JointAngleCallback;

   // Subscription-related
   ros::Subscriber            m_EndEffectorReader;
   ros::Subscriber            m_JointAngleReader;

   // Publication-related
   ros::Publisher             m_EndEffectorWriter;
   ros::Publisher             m_JointAngleWriter;
   EndEffectorSamplePtrVector m_EndEffectorSamples;
   JointAngleSamplePtrVector  m_JointAngleSamples;
   
   ros1PubSubPlugin*          m_pPlugin;
   EcU32                      m_QueueSize;
};

#endif // ecPubSubPluginRos1Layer_H
