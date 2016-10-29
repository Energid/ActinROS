#ifndef ros1RobotControllerGUIPlugin_H_
#define ros1RobotControllerGUIPlugin_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1RobotControllerGUIPlugin.h
/// @class ros1RobotControllerGUIPlugin
/// @brief GUI component to setup ROS1 Controller parameters.
//
//------------------------------------------------------------------------------
#include <rosPlugins/rosPlugins_config.h>  // Required to be first header.
#include <hardwarePluginGUI/ecHardwarePluginGUI.h>

class ros1RobotControllerPlugin;

//---------------------------------------------------------------------------
class EC_ROSPLUGINS_ROS1ROBOTCONTROLLERGUIPLUGIN_DECL ros1RobotControllerGUIPlugin : public Ec::hardwarePluginGUI
{
   Q_OBJECT

public:
   /// @copydoc Ec::Plugin::init()
   EcBoolean init
      (
      );

protected Q_SLOTS:
   void onActionConfigure
      (
      );

protected:
   /// Constructor
   ros1RobotControllerGUIPlugin
      (
      );

   /// Destructor
   ~ros1RobotControllerGUIPlugin
      (
      );

   ros1RobotControllerPlugin* m_pPlugin; ///< Handle to plugin that does the work
};

#endif // ros1RobotControllerGUIPlugin_H_
