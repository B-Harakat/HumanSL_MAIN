# ViconDataStream library CMakeLists.txt
cmake_minimum_required(VERSION 3.12)

# Create the ViconDataStream library
add_library(ViconDataStream STATIC
    src/ViconInterface.cpp
    src/ViconInfo.cpp
    src/ViconVisualize.cpp
    src/ViconTrigger.cpp
)

# Include directories for this library
target_include_directories(ViconDataStream
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${VICON_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/TrajectoryGeneration/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Link Vicon libraries to this library
target_link_libraries(ViconDataStream
    ${VICON_LIBRARIES}
)

# Set properties
set_target_properties(ViconDataStream PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Compiler flags
target_compile_options(ViconDataStream PRIVATE -Wall -Wextra -O3)