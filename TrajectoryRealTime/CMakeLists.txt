# Create library for TrajectoryRealTime
add_library(TrajectoryRealTime
    src/move.cpp
    src/plan.cpp
    src/sdf.cpp
    src/core.cpp
)

# Link against other project libraries and external dependencies
target_link_libraries(TrajectoryRealTime
    TrajectoryGeneration
    TrajectoryExecution
    ViconDataStream
    ${GPMP2_LIBRARIES}        
    ${GTSAM_LIBRARIES}        
    ${EZC3D_LIBRARIES}
    ${PINOCCHIO_LIBRARIES}
    ${VICON_LIBRARIES}
    ${YAMLCPP_LIBRARIES}
    console_bridge
    pthread
    dl
)

# Include directories
target_include_directories(TrajectoryRealTime PUBLIC
    include
    ../TrajectoryGeneration/include
    ../TrajectoryExecution/include
    ../ViconDataStream/include
)