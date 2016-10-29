//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecRos1PubSubManipulatorConfig.cpp
//
//------------------------------------------------------------------------------
#include "ecRos1PubSubManipulatorConfig.h"

static const EcToken TOKEN("ros1PubSubManipulatorConfig");

//---------------------------------------------------------------------------
EcRos1PubSubManipulatorConfig::EcRos1PubSubManipulatorConfig
   (
   ) :
   EcXmlCompoundType(),
   mode(MODE_IGNORE)
{
}

//---------------------------------------------------------------------------
EcRos1PubSubManipulatorConfig::~EcRos1PubSubManipulatorConfig
   (
   )
{
}

//---------------------------------------------------------------------------
EcRos1PubSubManipulatorConfig::EcRos1PubSubManipulatorConfig
   (
   const EcRos1PubSubManipulatorConfig& orig
   ) :
   EcXmlCompoundType(orig),
   mode(orig.mode)
{
}

//---------------------------------------------------------------------------
EcRos1PubSubManipulatorConfig& EcRos1PubSubManipulatorConfig::operator=
   (
   const EcRos1PubSubManipulatorConfig& orig
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
   mode = orig.mode;

   return *this;
}

//---------------------------------------------------------------------------
EcBoolean EcRos1PubSubManipulatorConfig::operator==
   (
   const EcRos1PubSubManipulatorConfig& orig
   ) const
{
   return
      (
      EcXmlCompoundType::operator==(orig) &&
      (mode == orig.mode)
      );
}

//---------------------------------------------------------------------------
void EcRos1PubSubManipulatorConfig::registerComponents
   (
   )
{
   // register the enumerations
   mode.setEnumString(MODE_IGNORE,           EcToken("IGNORE"));
   mode.setEnumString(MODE_PUB_EE_PUB_JOINT, EcToken("PUB_EE_PUB_JOINT"));
   mode.setEnumString(MODE_PUB_EE_SUB_JOINT, EcToken("PUB_EE_SUB_JOINT"));
   mode.setEnumString(MODE_SUB_EE_PUB_JOINT, EcToken("SUB_EE_PUB_JOINT"));
   mode.setEnumString(MODE_SUB_EE,           EcToken("SUB_EE"));
   mode.setEnumString(MODE_SUB_JOINT,        EcToken("SUB_JOINT"));

   // now register variables
   registerComponent(EcToken("mode"), &mode);
}

ECXML_DEFINE_TOKENS(EcRos1PubSubManipulatorConfig, TOKEN)
