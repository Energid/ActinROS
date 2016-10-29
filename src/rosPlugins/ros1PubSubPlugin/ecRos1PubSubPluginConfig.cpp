//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecRos1PubSubPluginConfig.cpp
//
//------------------------------------------------------------------------------
#include "ecRos1PubSubPluginConfig.h"

static const EcToken TOKEN("ros1PubSubPluginConfig");

//---------------------------------------------------------------------------
EcRos1PubSubPluginConfig::EcRos1PubSubPluginConfig
   (
   ) :
   EcXmlCompoundType(),
   jointAnglesTopicName("jointAngles"),
   desiredPlacementTopicName("desiredPlacement"),
   manipulatorConfigs()
{
}

//---------------------------------------------------------------------------
EcRos1PubSubPluginConfig::~EcRos1PubSubPluginConfig
   (
   )
{
}

//---------------------------------------------------------------------------
EcRos1PubSubPluginConfig::EcRos1PubSubPluginConfig
   (
   const EcRos1PubSubPluginConfig& orig
   ) :
   EcXmlCompoundType(orig),
   jointAnglesTopicName(orig.jointAnglesTopicName),
   desiredPlacementTopicName(orig.desiredPlacementTopicName),
   manipulatorConfigs(orig.manipulatorConfigs)
{
}

//---------------------------------------------------------------------------
EcRos1PubSubPluginConfig& EcRos1PubSubPluginConfig::operator=
   (
   const EcRos1PubSubPluginConfig& orig
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
   jointAnglesTopicName       = orig.jointAnglesTopicName;
   desiredPlacementTopicName  = orig.desiredPlacementTopicName;
   manipulatorConfigs         = orig.manipulatorConfigs;

   return *this;
}

//---------------------------------------------------------------------------
EcBoolean EcRos1PubSubPluginConfig::operator==
   (
   const EcRos1PubSubPluginConfig& orig
   ) const
{
   return
      (
      EcXmlCompoundType::operator==(orig) &&
      (jointAnglesTopicName      == orig.jointAnglesTopicName) &&
      (desiredPlacementTopicName == orig.desiredPlacementTopicName) &&
      (manipulatorConfigs        == orig.manipulatorConfigs)
      );
}

//---------------------------------------------------------------------------
void EcRos1PubSubPluginConfig::registerComponents
   (
   )
{
   registerComponent(EcToken("jointAnglesTopicName"),       &jointAnglesTopicName);
   registerComponent(EcToken("desiredPlacementTopicName"),  &desiredPlacementTopicName);
   registerComponent(EcToken("manipulatorConfigs"),         &manipulatorConfigs);
}

ECXML_DEFINE_TOKENS(EcRos1PubSubPluginConfig, TOKEN)
