#ifndef ecRos1HardwareConfig_H_
#define ecRos1HardwareConfig_H_
//-----------------------------------------------------------------------------
//     Copyright (c) 2011 Energid Technologies. All rights reserved.
//
/// @file ecRos1HardwareConfig.h
//
/// @class EcRos1HardwareConfig
//
/// @brief Class for individual ROS1 controlled manipulators
//
//-----------------------------------------------------------------------------
#include <rosPlugins/rosPlugins_config.h>  // Required to be first header.
#include <hardwarePlugin/ecHardwareConfigBase.h>


class EC_ROSPLUGINS_ROS1HARDWAREPLUGIN_DECL EcRos1HardwareConfig : public Ec::HardwareConfigBase
{
public:
   ECXMLOBJECT(EcRos1HardwareConfig);
   ECXML_XMLOBJECTCREATOR(EcRos1HardwareConfig);

   /// @copydoc Ec::HardwareConfigBase::registerComponents()
   void registerComponents
      (
      );

   EcXmlString  rosLinkLabel;
};

#endif // ecRos1HardwareConfig_H_
