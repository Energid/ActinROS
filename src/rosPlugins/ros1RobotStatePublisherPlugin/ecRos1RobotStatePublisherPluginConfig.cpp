//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecRos1RobotStatePublisherPluginConfig.cpp
//
//------------------------------------------------------------------------------
#include "ecRos1RobotStatePublisherPluginConfig.h"

static const EcToken TOKEN("ros1RobotStatePublisherPluginConfig");

//---------------------------------------------------------------------------
EcRos1RobotStatePublisherPluginConfig::EcRos1RobotStatePublisherPluginConfig
   (
   ) :
EcXmlCompoundType(),
publish_frequency(0.0),
rosWorldFrameName("world"),
feedbackJointStatesTopicName("joint_states"),
manipulatorConfigs()
{
}

//---------------------------------------------------------------------------
EcRos1RobotStatePublisherPluginConfig::~EcRos1RobotStatePublisherPluginConfig
   (
   )
{
}

//---------------------------------------------------------------------------
EcRos1RobotStatePublisherPluginConfig::EcRos1RobotStatePublisherPluginConfig
   (
   const EcRos1RobotStatePublisherPluginConfig& orig
   ) :
EcXmlCompoundType(orig),
publish_frequency(orig.publish_frequency),
rosWorldFrameName(orig.rosWorldFrameName),
feedbackJointStatesTopicName(orig.feedbackJointStatesTopicName),
manipulatorConfigs(orig.manipulatorConfigs)
{
}

//---------------------------------------------------------------------------
EcRos1RobotStatePublisherPluginConfig& EcRos1RobotStatePublisherPluginConfig::operator=
   (
   const EcRos1RobotStatePublisherPluginConfig& orig
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
   publish_frequency             = orig.publish_frequency;
   rosWorldFrameName             = orig.rosWorldFrameName;
   feedbackJointStatesTopicName  = orig.feedbackJointStatesTopicName;
   manipulatorConfigs            = orig.manipulatorConfigs;

   return *this;
}

//---------------------------------------------------------------------------
EcBoolean EcRos1RobotStatePublisherPluginConfig::operator==
   (
   const EcRos1RobotStatePublisherPluginConfig& orig
   ) const
{
   return
      (
      EcXmlCompoundType::operator==(orig) &&
      (publish_frequency            == orig.publish_frequency) &&
      (rosWorldFrameName            == orig.rosWorldFrameName) &&
      (feedbackJointStatesTopicName == orig.feedbackJointStatesTopicName) &&
      (manipulatorConfigs           == orig.manipulatorConfigs)
      );
}

//---------------------------------------------------------------------------
void EcRos1RobotStatePublisherPluginConfig::registerComponents
   (
   )
{
   registerComponent(EcToken("publish_frequency"),             &publish_frequency);
   registerComponent(EcToken("rosWorldFrameName"),             &rosWorldFrameName);
   registerComponent(EcToken("feedbackJointStatesTopicName"),  &feedbackJointStatesTopicName);
   registerComponent(EcToken("manipulatorConfigs"),            &manipulatorConfigs);
}

ECXML_DEFINE_TOKENS(EcRos1RobotStatePublisherPluginConfig, TOKEN)
