ecProject(ros1RobotControllerGUIPlugin)

ecCompileFlags("-DEC_HAVE_ACTIN")

ecIncludeSolutions(
    stableFoundation 
    actinCore
    foundation 
    actin
    guiCommon
    ros1
    plugins
    rosPlugins
)

ecExternIncludeDirs(
    Boost
    QT
    ros1
)

ecProjectLinkLibraries(
   plugins-hardwarePluginGUI
   rosPlugins-ros1RobotControllerPlugin
)

ecExternLinkLibraries(
   Qt5Gui
)

ecSourceFiles(
    ecRos1RobotControllerConfigDialog.cpp
    ros1RobotControllerGUIPlugin.cpp
)

ecQtObjectHeaderFiles(
    ecRos1RobotControllerConfigDialog.h
    ros1RobotControllerGUIPlugin.h
)

ecQtUiFiles(
    ecRos1RobotControllerConfigDialog.ui
)

ecPlugin(0)
