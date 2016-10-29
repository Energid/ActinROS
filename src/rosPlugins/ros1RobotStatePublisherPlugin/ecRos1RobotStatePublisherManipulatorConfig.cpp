//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecRos1RobotStatePubisherManipulatorConfig.cpp
//
//------------------------------------------------------------------------------
#include "ecRos1RobotStatePublisherManipulatorConfig.h"

static const EcToken TOKEN("ros1RobotStatePublisherManipulatorConfig");

//---------------------------------------------------------------------------
EcRos1RobotStatePublisherManipulatorConfig::EcRos1RobotStatePublisherManipulatorConfig
   (
   ) :
   EcXmlCompoundType(),
   manipulatorName(""),
   tf_prefix("")
{
}

//---------------------------------------------------------------------------
EcRos1RobotStatePublisherManipulatorConfig::~EcRos1RobotStatePublisherManipulatorConfig
   (
   )
{
}

//---------------------------------------------------------------------------
EcRos1RobotStatePublisherManipulatorConfig::EcRos1RobotStatePublisherManipulatorConfig
   (
   const EcRos1RobotStatePublisherManipulatorConfig& orig
   ) :
   EcXmlCompoundType(orig),
   manipulatorName(orig.manipulatorName),
   tf_prefix(orig.tf_prefix)
{
}

//---------------------------------------------------------------------------
EcRos1RobotStatePublisherManipulatorConfig& EcRos1RobotStatePublisherManipulatorConfig::operator=
   (
   const EcRos1RobotStatePublisherManipulatorConfig& orig
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
   manipulatorName   = orig.manipulatorName;
   tf_prefix         = orig.tf_prefix;

   return *this;
}

//---------------------------------------------------------------------------
EcBoolean EcRos1RobotStatePublisherManipulatorConfig::operator==
   (
   const EcRos1RobotStatePublisherManipulatorConfig& orig
   ) const
{
   return
      (
      EcXmlCompoundType::operator==(orig) &&
      (manipulatorName == orig.manipulatorName) &&
      (tf_prefix == orig.tf_prefix)
      );
}

//---------------------------------------------------------------------------
void EcRos1RobotStatePublisherManipulatorConfig::registerComponents
   (
   )
{
   // register the enumerations
   registerComponent(EcToken("manipulatorName"), &manipulatorName);
   registerComponent(EcToken("tf_prefix"),       &tf_prefix);
}

ECXML_DEFINE_TOKENS(EcRos1RobotStatePublisherManipulatorConfig, TOKEN)
