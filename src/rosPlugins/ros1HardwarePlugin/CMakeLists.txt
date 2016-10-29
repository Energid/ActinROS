ecProject(ros1HardwarePlugin)

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
   actin-hardwarePlugin
   ros1-ros1Tools
)

ecExternLinkLibraries(
   roscpp_serialization
   rosconsole
   rostime
   tf
   tf2_ros
   tf2
   console_bridge
)

ecCompileFlags("-DEC_HAVE_ACTIN")

ecSourceFiles(
    ecRos1HardwareConfig.cpp
    ecRos1HardwareManipulatorConfig.cpp
    ecRos1HardwarePluginData.cpp
    ros1HardwarePlugin.cpp
)

ecPlugin(0)
