ecSolution(rosPlugins)

if (EC_BUNDLE_ros1_OSS_INITIALIZED)
   ecAddSubdirs(
       ros1CytonMasterPlugin
       ros1CytonSlavePlugin
       ros1HardwarePlugin
       ros1JointStateEchoPlugin
       ros1ListenerPlugin
       ros1PubSubPlugin
       ros1RobotControllerPlugin
       ros1RobotStatePublisherPlugin
       ros1TalkerPlugin
   )
   if (NOT EC_EMBEDDED)
      ecAddSubdirs(
          ros1PubSubGUIPlugin
          ros1RobotControllerGUIPlugin
      )
   endif (NOT EC_EMBEDDED)
endif (EC_BUNDLE_ros1_OSS_INITIALIZED)

ecSolutionHeaders(rosPlugins/rosPlugins_config.h)