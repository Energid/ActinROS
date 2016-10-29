//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecRos1RobotControllerPluginConfig.cpp
//
//------------------------------------------------------------------------------
#include "ecRos1RobotControllerPluginConfig.h"

static const EcString ns("http://www.energid.com/namespace/mn#");
static const EcToken TOKEN(ns + "ros1RobotControllerPluginConfig");

//---------------------------------------------------------------------------
EcRos1RobotControllerPluginConfig::EcRos1RobotControllerPluginConfig
   (
   ) :
EcRos1HardwarePluginData(),
rosWorldFrameName("world"),
manipulatorConfigs()
{
}

//---------------------------------------------------------------------------
EcRos1RobotControllerPluginConfig::~EcRos1RobotControllerPluginConfig
   (
   )
{
}

//---------------------------------------------------------------------------
EcRos1RobotControllerPluginConfig::EcRos1RobotControllerPluginConfig
   (
   const EcRos1RobotControllerPluginConfig& orig
   ) :
EcRos1HardwarePluginData(orig),
rosWorldFrameName(orig.rosWorldFrameName),
manipulatorConfigs(orig.manipulatorConfigs)
{
}

//---------------------------------------------------------------------------
EcRos1RobotControllerPluginConfig& EcRos1RobotControllerPluginConfig::operator=
   (
   const EcRos1RobotControllerPluginConfig& orig
   )
{
   // self assignment.
   if (this == &orig)
   {
      return *this;
   }

   // call parent
   EcRos1HardwarePluginData::operator=(orig);

   // copy data
   rosWorldFrameName  = orig.rosWorldFrameName;
   manipulatorConfigs = orig.manipulatorConfigs;

   return *this;
}

//---------------------------------------------------------------------------
EcBoolean EcRos1RobotControllerPluginConfig::operator==
   (
   const EcRos1RobotControllerPluginConfig& orig
   ) const
{
   return
      (
      EcRos1HardwarePluginData::operator==(orig) &&
      (rosWorldFrameName  == orig.rosWorldFrameName) &&
      (manipulatorConfigs == orig.manipulatorConfigs)
      );
}

//---------------------------------------------------------------------------
void EcRos1RobotControllerPluginConfig::registerComponents
   (
   )
{
   EcRos1HardwarePluginData::registerComponents();
   registerComponent(EcToken(ns + "rosWorldFrameName"),  &rosWorldFrameName);
   registerComponent(EcToken(ns + "manipulatorConfigs"), &manipulatorConfigs);
}

ECXML_DEFINE_TOKENS(EcRos1RobotControllerPluginConfig, TOKEN)
