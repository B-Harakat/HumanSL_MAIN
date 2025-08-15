# TrajectoryGeneration Library
add_library(TrajectoryGeneration
    src/utils.cpp
    src/Obstacles.cpp
    src/GenerateArmModel.cpp
    src/GenerateLogs.cpp
    src/TrajectoryInitiation.cpp
    src/TrajectoryOptimization.cpp
    #src/Planner.cpp
    #src/Tube.cpp
    #src/RRTStarPlanner.cpp
    #src/RRTStarSmoothing.cpp
    #src/TrajectoryComparison.cpp
)

# Make headers available to other targets
target_include_directories(TrajectoryGeneration PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../thirdParty
    ${CMAKE_CURRENT_SOURCE_DIR}/../ViconDataStream/include
)

target_link_libraries(TrajectoryGeneration 
    ViconDataStream
)