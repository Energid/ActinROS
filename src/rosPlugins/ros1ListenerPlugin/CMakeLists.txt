ecProject(ros1ListenerPlugin)

ecCompileFlags("-DEC_HAVE_ACTIN")

ecIncludeSolutions(
    stableFoundation 
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

ecSourceFiles(
    ros1ListenerPlugin.cpp
)

ecPlugin(0)
