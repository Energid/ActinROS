#ifndef ecRos1HardwareManipulatorConfig_H_
#define ecRos1HardwareManipulatorConfig_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecRos1HardwareManipulatorConfig.h
/// @brief Class to hold a serializable ROS1 hardware manipulator configuration.
//
//------------------------------------------------------------------------------
#include <rosPlugins/rosPlugins_config.h>
#include <xml/ecXmlBasicType.h>
#include <xml/ecXmlCompType.h>

/// Class to hold a serializable ROS1 hardware manipulator configuration.
class EC_ROSPLUGINS_ROS1HARDWAREPLUGIN_DECL EcRos1HardwareManipulatorConfig : public EcXmlCompoundType
{
public:

   ECXMLOBJECT(EcRos1HardwareManipulatorConfig);

   void registerComponents
      (
      );

   EcXmlString rosManipulatorLabel; ///< Name of manipulator as seen by ROS (usually manipulator name with dashes & spaces replaced with underscores).
   EcXmlString tf_prefix;           ///< ROS tf_prefix (default is empty string).
};

#endif // ecRos1HardwareManipulatorConfig_H_
