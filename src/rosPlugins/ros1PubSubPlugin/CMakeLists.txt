ecProject(ros1PubSubPlugin)

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
)

ecRos1Idl(EcRos1
    ManipulatorJointAngles.msg
    ManipulatorMessage.msg
)

ecSourceFiles(
    ecRos1PubSubManipulatorConfig.cpp
    ecRos1PubSubPluginConfig.cpp
    ecPubSubPluginRos1Layer.cpp
    ecTemplateInstRos1PubSubPlugin.cpp
    ros1PubSubPlugin.cpp
)

ecPlugin(0)
