//     Copyright (c) 2016 Energid Technologies. All rights reserved. ////
//
// Filename:    ecRos1HardwareConfig.cpp
//
/////////////////////////////////////////////////////////////////////////
#include "ecRos1HardwareConfig.h"

static const EcString ns("http://www.energid.com/namespace/mn#");
static const EcToken EcRos1HardwareConfigToken(ns + "EcRos1HardwareConfig");

//------------------------------------------------------------------------------
EcRos1HardwareConfig::EcRos1HardwareConfig
   (
   ):
Ec::HardwareConfigBase(),
rosLinkLabel()
{
}


//------------------------------------------------------------------------------
EcRos1HardwareConfig::~EcRos1HardwareConfig
   (
   )
{
}


//------------------------------------------------------------------------------
EcRos1HardwareConfig::EcRos1HardwareConfig
   (
   const EcRos1HardwareConfig& orig
   ):
Ec::HardwareConfigBase(orig),
rosLinkLabel(orig.rosLinkLabel)
{
}


//------------------------------------------------------------------------------
EcRos1HardwareConfig& EcRos1HardwareConfig::operator=
   (
   const EcRos1HardwareConfig& orig
   )
{
   if(this == &orig)
   {
      return *this;
   }

   Ec::HardwareConfigBase::operator=(orig);

   rosLinkLabel = orig.rosLinkLabel;
   return *this;
}


//------------------------------------------------------------------------------
EcBoolean
EcRos1HardwareConfig::operator==
   (
   const EcRos1HardwareConfig& rhs
   ) const
{
   return (Ec::HardwareConfigBase::operator==(rhs) &&
          (rosLinkLabel == rhs.rosLinkLabel)
          );
}

//------------------------------------------------------------------------------
void
EcRos1HardwareConfig::registerComponents
   (
   )
{
   Ec::HardwareConfigBase::registerComponents(); //Remove unnecessary members in config
   registerComponent(EcToken(ns + "rosLinkLabel"), &rosLinkLabel);
}

ECXML_DEFINE_TOKENS(EcRos1HardwareConfig, EcRos1HardwareConfigToken)
