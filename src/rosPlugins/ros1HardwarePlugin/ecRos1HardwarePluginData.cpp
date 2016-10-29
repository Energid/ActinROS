//     Copyright (c) 2016 Energid Technologies. All rights reserved. ////
//
// Filename:    ecRos1HardwarePluginData.cpp
//
/////////////////////////////////////////////////////////////////////////
#include "ecRos1HardwarePluginData.h"
#include "ecRos1HardwareConfig.h"

static const EcString ns("http://www.energid.com/namespace/mn#");
static const EcToken EcRos1HardwarePluginDataToken(ns + "ros1HardwarePluginData");

//------------------------------------------------------------------------------
EcRos1HardwarePluginData::EcRos1HardwarePluginData
   (
   ):
Ec::HardwarePluginData(),
publish_frequency(0.0),
rosManipulatorConfig(),
feedbackJointStatesTopicName("joint_states"),
publishJointStatesTopicName("actin_joint_states")
{
   m_vpHardwareConfig.ECXML_REGISTER_COMPONENT_CREATOR(EcRos1HardwareConfig);
}

//---------------------------------------------------------------------------
EcRos1HardwarePluginData::~EcRos1HardwarePluginData
   (
   )
{
}

//------------------------------------------------------------------------------
EcRos1HardwarePluginData::EcRos1HardwarePluginData
   (
   const EcRos1HardwarePluginData& orig
   ):
Ec::HardwarePluginData(orig),
publish_frequency(orig.publish_frequency),
rosManipulatorConfig(orig.rosManipulatorConfig),
feedbackJointStatesTopicName(orig.feedbackJointStatesTopicName),
publishJointStatesTopicName(orig.publishJointStatesTopicName)
{
}

//---------------------------------------------------------------------------
EcRos1HardwarePluginData& EcRos1HardwarePluginData::operator=
   (
   const EcRos1HardwarePluginData& orig
   )
{
   // self assignment.
   if (this == &orig)
   {
      return *this;
   }

   // call parent
   Ec::HardwarePluginData::operator=(orig);

   // copy data
   publish_frequency             = orig.publish_frequency;
   rosManipulatorConfig          = orig.rosManipulatorConfig;
   feedbackJointStatesTopicName  = orig.feedbackJointStatesTopicName;
   publishJointStatesTopicName   = orig.publishJointStatesTopicName;

   return *this;
}

//---------------------------------------------------------------------------
EcBoolean EcRos1HardwarePluginData::operator==
   (
   const EcRos1HardwarePluginData& orig
   ) const
{
   return
      (
      Ec::HardwarePluginData::operator==(orig) &&
      (publish_frequency              == orig.publish_frequency) &&
      (rosManipulatorConfig           == orig.rosManipulatorConfig) &&
      (feedbackJointStatesTopicName   == orig.feedbackJointStatesTopicName) &&
      (publishJointStatesTopicName    == orig.publishJointStatesTopicName)
      );
}

//------------------------------------------------------------------------------
// Rewrite PluginData version to remove baud rate
void EcRos1HardwarePluginData::registerComponents
   (
   )
{
   HardwarePluginData::registerComponents();
   registerComponent(EcToken(ns + "publish_frequency"),             &publish_frequency);
   registerComponent(EcToken(ns + "rosManipulatorConfig"),          &rosManipulatorConfig);
   registerComponent(EcToken(ns + "feedbackJointStatesTopicName"),  &feedbackJointStatesTopicName);
   registerComponent(EcToken(ns + "publishJointStatesTopicName"),   &publishJointStatesTopicName);
}

ECXML_DEFINE_TOKENS(EcRos1HardwarePluginData, EcRos1HardwarePluginDataToken)

