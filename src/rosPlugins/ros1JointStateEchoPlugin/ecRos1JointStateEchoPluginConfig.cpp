//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecRos1JointStateEchoPluginConfig.cpp
//
//------------------------------------------------------------------------------
#include "ecRos1JointStateEchoPluginConfig.h"

static const EcToken TOKEN("ros1JointStateEchoPluginConfig");

//---------------------------------------------------------------------------
EcRos1JointStateEchoPluginConfig::EcRos1JointStateEchoPluginConfig
   (
   ) :
   EcXmlCompoundType(),
   publish_frequency(0.0),
   queueSize(100),
   feedbackJointStatesTopicName("joint_states"),
   publishJointStatesTopicName("actin_joint_states")
{
}

//---------------------------------------------------------------------------
EcRos1JointStateEchoPluginConfig::~EcRos1JointStateEchoPluginConfig
   (
   )
{
}

//---------------------------------------------------------------------------
EcRos1JointStateEchoPluginConfig::EcRos1JointStateEchoPluginConfig
   (
   const EcRos1JointStateEchoPluginConfig& orig
   ) :
   EcXmlCompoundType(orig),
   publish_frequency(orig.publish_frequency),
   queueSize(orig.queueSize),
   feedbackJointStatesTopicName(orig.feedbackJointStatesTopicName),
   publishJointStatesTopicName(orig.publishJointStatesTopicName)
{
}

//---------------------------------------------------------------------------
EcRos1JointStateEchoPluginConfig& EcRos1JointStateEchoPluginConfig::operator=
   (
   const EcRos1JointStateEchoPluginConfig& orig
   )
{
   // self assignment.
   if (this == &orig)
   {
      return *this;
   }

   // call parent
   EcXmlCompoundType::operator=(orig);

   // copy data
   publish_frequency            = orig.publish_frequency;
   queueSize                    = orig.queueSize;
   feedbackJointStatesTopicName = orig.feedbackJointStatesTopicName;
   publishJointStatesTopicName  = orig.publishJointStatesTopicName;

   return *this;
}

//---------------------------------------------------------------------------
EcBoolean EcRos1JointStateEchoPluginConfig::operator==
   (
   const EcRos1JointStateEchoPluginConfig& orig
   ) const
{
   return
      (
      EcXmlCompoundType::operator==(orig) &&
      (publish_frequency            == orig.publish_frequency) &&
      (queueSize                    == orig.queueSize) &&
      (feedbackJointStatesTopicName == orig.feedbackJointStatesTopicName) &&
      (publishJointStatesTopicName  == orig.publishJointStatesTopicName)
      );
}

//---------------------------------------------------------------------------
void EcRos1JointStateEchoPluginConfig::registerComponents
   (
   )
{
   registerComponent(EcToken("publish_frequency"),              &publish_frequency);
   registerComponent(EcToken("queueSize"),                      &queueSize);
   registerComponent(EcToken("feedbackJointStatesTopicName"),   &feedbackJointStatesTopicName);
   registerComponent(EcToken("publishJointStatesTopicName"),    &publishJointStatesTopicName);
}

ECXML_DEFINE_TOKENS(EcRos1JointStateEchoPluginConfig, TOKEN)
