ecAddSubdirs(rosApps)

if (NOT EC_ENABLE_STATIC_BUILD)
    ecAddSubdirs(rosPlugins)
endif ()