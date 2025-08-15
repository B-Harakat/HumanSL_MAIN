# TrajectoryExecution Library
add_library(TrajectoryExecution
    src/Controller.cpp
    src/Dynamics.cpp
    src/Filter.cpp
    src/Fwd_kinematics.cpp
    src/Jacobian.cpp
    src/KinovaTrajectory.cpp
)

# Make headers available to other targets
target_include_directories(TrajectoryExecution PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../ViconDataStream/include
)

target_link_libraries(TrajectoryExecution 
    ViconDataStream
)