//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1RobotControllerGUIPlugin.cpp
//
//------------------------------------------------------------------------------
#include "ros1RobotControllerGUIPlugin.h"
#include "ecRos1RobotControllerConfigDialog.h"

#include <ros1RobotControllerPlugin/ros1RobotControllerPlugin.h>

#include <QtWidgets/QAction>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QToolBar>

EC_PLUGIN_STUB_DEFAULT(ros1RobotControllerGUIPlugin);

//---
// ros1RobotStatePubSubGUIPlugin constructor
//---

//---------------------------------------------------------------------------
ros1RobotControllerGUIPlugin::ros1RobotControllerGUIPlugin
   (
   ) :
Ec::hardwarePluginGUI("ros1RobotControllerPlugin"),
m_pPlugin(EcNULL)
{
}

//---
// ros1RobotStatePubSubGUIPlugin destructor
//---

//---------------------------------------------------------------------------
ros1RobotControllerGUIPlugin::~ros1RobotControllerGUIPlugin
   (
   )
{
}

//---
// ros1RobotStatePubSubGUIPlugin public methods
//---

//---------------------------------------------------------------------------
EcBoolean ros1RobotControllerGUIPlugin::init
   (
   )
{
   if (Ec::hardwarePluginGUI::init())
   {
      m_pPlugin = EcPLUGIN_FIND_AND_CAST(ros1RobotControllerPlugin);
      if (!m_pPlugin)
      {
         return EcFalse;
      }

      // Add a button to the miscellaneous toolbar.
      QToolBar* pMiscToolBar = getPtr<QToolBar>("miscellaneousToolBar");
      if (pMiscToolBar)
      {
         QAction* pConfigureAction = new QAction(this);
         pConfigureAction->setCheckable(false);
         pConfigureAction->setIcon(QIcon(QString::fromUtf8(":/images/ros-logo.png")));

         pConfigureAction->setObjectName(QString::fromStdString("actionConfigureRos1RobotController"));
         pConfigureAction->setText(tr("&Configure ROS Robot Controller Settings"));
         pConfigureAction->setToolTip(tr("Configure ROS Robot Controller Settings"));
         pConfigureAction->setStatusTip(tr("Configure ROS Robot Controller Settings"));

         pMiscToolBar->addAction(pConfigureAction);

         connect(pConfigureAction, SIGNAL(triggered()), SLOT(onActionConfigure()));

         enableWithFile(pConfigureAction);
      }

      return EcTrue;
   }
   return EcFalse;
}

//---
// ros1RobotStatePubSubGUIPlugin protected slots
//---

//---------------------------------------------------------------------------
void ros1RobotControllerGUIPlugin::onActionConfigure
   (
   )
{
   QMainWindow* pWin = getPtr<QMainWindow>();
   EcRos1RobotControllerConfigDialog dialog(m_pPlugin, pWin);
   dialog.exec();
}
