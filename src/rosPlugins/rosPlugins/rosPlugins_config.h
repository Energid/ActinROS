#ifndef rosPlugins_config_H_
#define rosPlugins_config_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file rosPlugins_config.h
/// @brief Configuration header for the rosPlugins hierarchy.
//
//------------------------------------------------------------------------------
#include <foundCore/ecConfig.h>

/// ros1CytonMasterPlugin directory
#if defined(rosPlugins_ros1CytonMasterPlugin_EXPORTS)
#  define EC_ROSPLUGINS_ROS1CYTONMASTERPLUGIN_DECL EC_DECL_EXPORTS
#else
#  define EC_ROSPLUGINS_ROS1CYTONMASTERPLUGIN_DECL EC_DECL_IMPORTS
#endif

/// ros1CytonSlavePlugin directory
#if defined(rosPlugins_ros1CytonSlavePlugin_EXPORTS)
#  define EC_ROSPLUGINS_ROS1CYTONSLAVEPLUGIN_DECL EC_DECL_EXPORTS
#else
#  define EC_ROSPLUGINS_ROS1CYTONSLAVEPLUGIN_DECL EC_DECL_IMPORTS
#endif

/// ros1HardwarePlugin directory
#if defined(rosPlugins_ros1HardwarePlugin_EXPORTS)
#  define EC_ROSPLUGINS_ROS1HARDWAREPLUGIN_DECL EC_DECL_EXPORTS
#else
#  define EC_ROSPLUGINS_ROS1HARDWAREPLUGIN_DECL EC_DECL_IMPORTS
#endif

/// ros1JointStateEchoPlugin directory
#if defined(rosPlugins_ros1JointStateEchoPlugin_EXPORTS)
#  define EC_ROSPLUGINS_ROS1JOINTSTATEECHOPLUGIN_DECL EC_DECL_EXPORTS
#else
#  define EC_ROSPLUGINS_ROS1JOINTSTATEECHOPLUGIN_DECL EC_DECL_IMPORTS
#endif

/// ros1ListenerPlugin directory
#if defined(rosPlugins_ros1ListenerPlugin_EXPORTS)
#  define EC_ROSPLUGINS_ROS1LISTENERPLUGIN_DECL EC_DECL_EXPORTS
#else
#  define EC_ROSPLUGINS_ROS1LISTENERPLUGIN_DECL EC_DECL_IMPORTS
#endif

/// ros1PubSubGUIPlugin directory
#if defined(rosPlugins_ros1PubSubGUIPlugin_EXPORTS)
#  define EC_ROSPLUGINS_ROS1PUBSUBGUIPLUGIN_DECL EC_DECL_EXPORTS
#else
#  define EC_ROSPLUGINS_ROS1PUBSUBGUIPLUGIN_DECL EC_DECL_IMPORTS
#endif

/// ros1PubSubPlugin directory
#if defined(rosPlugins_ros1PubSubPlugin_EXPORTS)
#  define EC_ROSPLUGINS_ROS1PUBSUBPLUGIN_DECL EC_DECL_EXPORTS
#else
#  define EC_ROSPLUGINS_ROS1PUBSUBPLUGIN_DECL EC_DECL_IMPORTS
#endif

/// ros1RobotControllerGUIPlugin directory
#if defined(rosPlugins_ros1RobotControllerGUIPlugin_EXPORTS)
#  define EC_ROSPLUGINS_ROS1ROBOTCONTROLLERGUIPLUGIN_DECL EC_DECL_EXPORTS
#else
#  define EC_ROSPLUGINS_ROS1ROBOTCONTROLLERGUIPLUGIN_DECL EC_DECL_IMPORTS
#endif

/// ros1RobotControllerPlugin directory
#if defined(rosPlugins_ros1RobotControllerPlugin_EXPORTS)
#  define EC_ROSPLUGINS_ROS1ROBOTCONTROLLERPLUGIN_DECL EC_DECL_EXPORTS
#else
#  define EC_ROSPLUGINS_ROS1ROBOTCONTROLLERPLUGIN_DECL EC_DECL_IMPORTS
#endif

/// ros1RobotStatePublisherPlugin directory
#if defined(rosPlugins_ros1RobotStatePublisherPlugin_EXPORTS)
#  define EC_ROSPLUGINS_ROS1ROBOTSTATEPUBLISHERPLUGIN_DECL EC_DECL_EXPORTS
#else
#  define EC_ROSPLUGINS_ROS1ROBOTSTATEPUBLISHERPLUGIN_DECL EC_DECL_IMPORTS
#endif

/// ros1TalkerPlugin directory
#if defined(rosPlugins_ros1TalkerPlugin_EXPORTS)
#  define EC_ROSPLUGINS_ROS1TALKERPLUGIN_DECL EC_DECL_EXPORTS
#else
#  define EC_ROSPLUGINS_ROS1TALKERPLUGIN_DECL EC_DECL_IMPORTS
#endif

#endif // rosPlugins_config_H_
