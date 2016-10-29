ecProject(ros1RobotStatePublisherPlugin)

ecCompileFlags("-DEC_HAVE_ACTIN")

ecIncludeSolutions(
    stableFoundation 
    actinCore
    foundation 
    actin
    ros1
    rosPlugins
)

ecExternIncludeDirs(
    Boost
    ros1
)

ecProjectLinkLibraries(
   ros1-ros1PluginBase
   actin-plugins
)

ecExternLinkLibraries(
   roscpp
   roscpp_serialization
   rosconsole
   rostime
   tf
   tf2_ros
   tf2
   console_bridge
)

ecSourceFiles(
    ecRos1RobotStatePublisherManipulatorConfig.cpp
    ecRos1RobotStatePublisherPluginConfig.cpp
    ecTemplateInstRos1RobotStatePublisherPlugin.cpp
    ros1RobotStatePublisherPlugin.cpp
)

ecPlugin(0)
