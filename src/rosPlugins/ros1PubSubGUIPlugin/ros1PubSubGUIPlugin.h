#ifndef ros1PubSubGUIPlugin_H_
#define ros1PubSubGUIPlugin_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1PubSubGUIPlugin.h
/// @class ros1PubSubGUIPlugin
/// @brief GUI component to setup ROS1 Publish/Subscribe parameters.
//
//------------------------------------------------------------------------------
#include <rosPlugins/rosPlugins_config.h>
#include <viewerCore/ecPluginGUI.h>

class ros1PubSubPlugin;

//---------------------------------------------------------------------------
class EC_ROSPLUGINS_ROS1PUBSUBGUIPLUGIN_DECL ros1PubSubGUIPlugin : public Ec::PluginGUI
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
   ros1PubSubGUIPlugin
      (
      );

   /// Destructor
   ~ros1PubSubGUIPlugin
      (
      );

   ros1PubSubPlugin* m_pPlugin; ///< Handle to plugin that does the work
};

#endif // ros1PubSubGUIPlugin_H_
