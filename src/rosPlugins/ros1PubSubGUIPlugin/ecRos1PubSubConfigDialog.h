#ifndef ecRos1PubSubConfigDialog_H_
#define ecRos1PubSubConfigDialog_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecRos1PubSubConfigDialog.h
/// @class EcRos1PubSubConfigDialog
/// @brief Dialog for setting up ROS1 Publish/Subscribe configuration
//
//------------------------------------------------------------------------------
#include <rosPlugins/rosPlugins_config.h>
#include <QtWidgets/QDialog>
#include <QtCore/QScopedPointer>

// Forward declarations
class ros1PubSubPlugin;
namespace Ui { class EcRos1PubSubConfigDialog; }

//---------------------------------------------------------------------------
class EC_ROSPLUGINS_ROS1PUBSUBGUIPLUGIN_DECL EcRos1PubSubConfigDialog : public QDialog
{
   Q_OBJECT

public:
   /// Constructor
   /// @param[in] pPlugin A pointer to the plugin to issue ROS1 Publish/Subscribe commands
   /// @param[in] pParent A pointer to the parent widget
   EcRos1PubSubConfigDialog
      (
      ros1PubSubPlugin* pPlugin,
      QWidget*         pParent
      );

   /// Destructor
   ~EcRos1PubSubConfigDialog
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

   QScopedPointer<Ui::EcRos1PubSubConfigDialog> m_UiPtr;    ///< UI components from Qt Designer
   ros1PubSubPlugin*                            m_pPlugin;  ///< Plugin to control ROS1 Publish/Subscribe
};

#endif // ecRos1PubSubConfigDialog_H_
