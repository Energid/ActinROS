ecProject(ros1PubSubGUIPlugin)

ecCompileFlags("-DEC_HAVE_ACTIN")

ecIncludeSolutions(
    stableFoundation 
    actinCore
    foundation 
    actin
    guiCommon
    ros1
    rosPlugins
)

ecExternIncludeDirs(
    Boost
    QT
    ros1
)

ecProjectLinkLibraries(
   guiCommon-viewerCore
   rosPlugins-ros1PubSubPlugin
)

ecExternLinkLibraries(
   Qt5Gui
)

ecSourceFiles(
    ecRos1PubSubConfigDialog.cpp
    ros1PubSubGUIPlugin.cpp
)

ecQtObjectHeaderFiles(
    ecRos1PubSubConfigDialog.h
    ros1PubSubGUIPlugin.h
)

ecQtUiFiles(
    ecRos1PubSubConfigDialog.ui
)

ecPlugin(0)
