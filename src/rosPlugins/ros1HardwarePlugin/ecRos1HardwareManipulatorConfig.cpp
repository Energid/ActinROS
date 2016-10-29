//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecRos1HardwareManipulatorConfig.cpp
//
//------------------------------------------------------------------------------
#include "ecRos1HardwareManipulatorConfig.h"

static const EcString ns("http://www.energid.com/namespace/mn#");
static const EcToken TOKEN(ns + "ros1HardwareManipulatorConfig");

//---------------------------------------------------------------------------
EcRos1HardwareManipulatorConfig::EcRos1HardwareManipulatorConfig
   (
   ) :
EcXmlCompoundType(),
rosManipulatorLabel(""),
tf_prefix("")
{
}

//---------------------------------------------------------------------------
EcRos1HardwareManipulatorConfig::~EcRos1HardwareManipulatorConfig
   (
   )
{
}

//---------------------------------------------------------------------------
EcRos1HardwareManipulatorConfig::EcRos1HardwareManipulatorConfig
   (
   const EcRos1HardwareManipulatorConfig& orig
   ) :
EcXmlCompoundType(orig),
rosManipulatorLabel(orig.rosManipulatorLabel),
tf_prefix(orig.tf_prefix)
{
}

//---------------------------------------------------------------------------
EcRos1HardwareManipulatorConfig& EcRos1HardwareManipulatorConfig::operator=
   (
   const EcRos1HardwareManipulatorConfig& orig
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
   rosManipulatorLabel  = orig.rosManipulatorLabel;
   tf_prefix            = orig.tf_prefix;

   return *this;
}

//---------------------------------------------------------------------------
EcBoolean EcRos1HardwareManipulatorConfig::operator==
   (
   const EcRos1HardwareManipulatorConfig& orig
   ) const
{
   return
      (
      EcXmlCompoundType::operator==(orig) &&
      (rosManipulatorLabel == orig.rosManipulatorLabel) &&
      (tf_prefix == orig.tf_prefix)
      );
}

//---------------------------------------------------------------------------
void EcRos1HardwareManipulatorConfig::registerComponents
   (
   )
{
   // register the enumerations
   registerComponent(EcToken(ns + "rosManipulatorLabel"),   &rosManipulatorLabel);
   registerComponent(EcToken(ns + "tf_prefix"),             &tf_prefix);
}

ECXML_DEFINE_TOKENS(EcRos1HardwareManipulatorConfig, TOKEN)
