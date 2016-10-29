ecProject(ros1RobotControllerPlugin)

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
   rosPlugins-ros1HardwarePlugin
)

ecSourceFiles(
    ecRos1RobotControllerPluginConfig.cpp
    ecTemplateInstRos1RobotControllerPlugin.cpp
    ros1RobotControllerPlugin.cpp
)

ecPlugin(0)
