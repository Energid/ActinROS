#ifndef ecRos1RobotStatePubSubConfigDialog_H_
#define ecRos1RobotStatePubSubConfigDialog_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecRos1RobotStatePubSubConfigDialog.h
/// @class EcRos1RobotStatePubSubConfigDialog
/// @brief Dialog for setting up ROS1 Publish/Subscribe configuration
//
//------------------------------------------------------------------------------
#include <rosPlugins/rosPlugins_config.h>  // Required to be first header.

#include <QtWidgets/QDialog>
#include <QtCore/QScopedPointer>

// Forward declarations
class ros1RobotControllerPlugin;
namespace Ui { class EcRos1RobotControllerConfigDialog; }

//---------------------------------------------------------------------------
class EC_ROSPLUGINS_ROS1ROBOTCONTROLLERGUIPLUGIN_DECL EcRos1RobotControllerConfigDialog : public QDialog
{
   Q_OBJECT

public:
   /// Constructor
   /// @param[in] pPlugin A pointer to the plugin to issue ROS1 Robot State Publish/Subscribe commands
   /// @param[in] pParent A pointer to the parent widget
   EcRos1RobotControllerConfigDialog
      (
      ros1RobotControllerPlugin* pPlugin,
      QWidget*         pParent
      );

   /// Destructor
   ~EcRos1RobotControllerConfigDialog
      (
      );

protected Q_SLOTS:
   void accept
      (
      );

protected:
   /// set from XML configuration
   void setFromConfiguration
      (
      );

   QScopedPointer<Ui::EcRos1RobotControllerConfigDialog> m_UiPtr;    ///< UI components from Qt Designer
   ros1RobotControllerPlugin*                            m_pPlugin;  ///< Plugin to control ROS1 Publish/Subscribe
};

#endif // ecRos1RobotStatePubSubConfigDialog_H_
