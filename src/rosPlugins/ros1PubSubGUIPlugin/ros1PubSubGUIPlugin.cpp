//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1PubSubGUIPlugin.cpp
//
//------------------------------------------------------------------------------
#include "ros1PubSubGUIPlugin.h"
#include "ecRos1PubSubConfigDialog.h"

#include <ros1PubSubPlugin/ros1PubSubPlugin.h>

#include <QtWidgets/QAction>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QToolBar>

EC_PLUGIN_STUB_DEFAULT(ros1PubSubGUIPlugin);

//---
// ros1PubSubGUIPlugin constructor
//---

//---------------------------------------------------------------------------
ros1PubSubGUIPlugin::ros1PubSubGUIPlugin
   (
   ) :
   Ec::PluginGUI(),
   m_pPlugin(EcNULL)
{
   m_RequiredPlugins.push_back("ros1PubSubPlugin");
}

//---
// ros1PubSubGUIPlugin destructor
//---

//---------------------------------------------------------------------------
ros1PubSubGUIPlugin::~ros1PubSubGUIPlugin
   (
   )
{
}

//---
// ros1PubSubGUIPlugin public methods
//---

//---------------------------------------------------------------------------
EcBoolean ros1PubSubGUIPlugin::init
   (
   )
{
   m_pPlugin = EcPLUGIN_FIND_AND_CAST(ros1PubSubPlugin);
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
      pConfigureAction->setIcon(QIcon(QString::fromUtf8(":/images/network.png")));

      pConfigureAction->setObjectName(QString::fromStdString("actionConfigureRos1PubSub"));
      pConfigureAction->setText(tr("&Configure ROS Publish/Subscribe Settings"));
      pConfigureAction->setToolTip(tr("Configure ROS Publish/Subscribe Settings"));
      pConfigureAction->setStatusTip(tr("Configure ROS Publish/Subscribe Settings"));

      pMiscToolBar->addAction(pConfigureAction);

      connect(pConfigureAction, SIGNAL(triggered()), SLOT(onActionConfigure()));

      enableWithFile(pConfigureAction);
   }

   return EcTrue;
}

//---
// ros1PubSubGUIPlugin protected slots
//---

//---------------------------------------------------------------------------
void ros1PubSubGUIPlugin::onActionConfigure
   (
   )
{
   QMainWindow* pWin = getPtr<QMainWindow>();
   EcRos1PubSubConfigDialog dialog(m_pPlugin, pWin);
   dialog.exec();
}
