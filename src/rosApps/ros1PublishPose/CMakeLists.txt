ecProject(ros1PublishPose)

ecIncludeSolutions(
    stableFoundation
    foundation
    actin
    actinCore
)

ecExternIncludeDirs(Boost ros1)

ecProjectLinkLibraries(
    actin-systemSimulation
)

ecExternLinkLibraries(
   roscpp
   roscpp_serialization
   rosconsole
   rostime
)

ecSourceFiles(ecRos1PublishPoseMain.cpp)
ecExecutable()
