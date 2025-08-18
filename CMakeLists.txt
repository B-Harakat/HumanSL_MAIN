cmake_minimum_required(VERSION 3.12)
project(HumanSL_Masters_Project_2025)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-w)

add_compile_definitions(GTSAM_EIGEN_SERIALIZATION_CONFLICT)

# Set paths to bundled libraries
set(THIRDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
set(THIRDPARTY_LIB_DIR "${THIRDPARTY_DIR}/lib")
set(THIRDPARTY_INCLUDE_DIR "${THIRDPARTY_DIR}/include")

# Find required packages (only for system packages)
find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)

# Use bundled Eigen3
set(EIGEN3_INCLUDE_DIR "${THIRDPARTY_INCLUDE_DIR}/eigen3")

# Use bundled GTSAM
set(GTSAM_INCLUDE_DIRS ${THIRDPARTY_INCLUDE_DIR})
set(GTSAM_LIBRARIES 
    ${THIRDPARTY_LIB_DIR}/libgtsam.so
    ${THIRDPARTY_LIB_DIR}/libgtsam_unstable.so
    ${THIRDPARTY_LIB_DIR}/libmetis-gtsam.so
    ${THIRDPARTY_LIB_DIR}/libcephes-gtsam.so
)
message(STATUS "Using bundled GTSAM libraries: ${GTSAM_LIBRARIES}")

# Use bundled GPMP2
set(GPMP2_INCLUDE_DIRS ${THIRDPARTY_INCLUDE_DIR})
set(GPMP2_LIBRARIES ${THIRDPARTY_LIB_DIR}/libgpmp2.so)
message(STATUS "Using bundled GPMP2 library: ${GPMP2_LIBRARIES}")

# Use bundled ezc3d
set(EZC3D_INCLUDE_DIRS ${THIRDPARTY_INCLUDE_DIR})
set(EZC3D_LIBRARIES ${THIRDPARTY_LIB_DIR}/libezc3d.so)
message(STATUS "Using bundled ezc3d library: ${EZC3D_LIBRARIES}")

# Use bundled Pinocchio
set(PINOCCHIO_INCLUDE_DIRS ${THIRDPARTY_INCLUDE_DIR})
set(PINOCCHIO_LIBRARIES 
    ${THIRDPARTY_LIB_DIR}/libpinocchio_default.so
    ${THIRDPARTY_LIB_DIR}/libpinocchio_parsers.so
  # ${THIRDPARTY_LIB_DIR}/libpinocchio_casadi.so
    ${THIRDPARTY_LIB_DIR}/libpinocchio_collision.so
)
message(STATUS "Using bundled Pinocchio libraries: ${PINOCCHIO_LIBRARIES}")

# Use bundled Open3D
set(OPEN3D_INCLUDE_DIRS ${THIRDPARTY_INCLUDE_DIR})
set(OPEN3D_LIBRARIES ${THIRDPARTY_LIB_DIR}/libOpen3D.so)
message(STATUS "Using bundled Open3D library: ${OPEN3D_LIBRARIES}")


# Find yaml-cpp (system package)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

# Use bundled Kinova API
set(KORTEX_DIR "${THIRDPARTY_DIR}/kortex_api/")

# Use Kinova's library selection logic
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(KORTEX_LIB_SUBDIR "debug")
else()
    set(KORTEX_LIB_SUBDIR "release")
endif()

# Link to bundled Kinova's pre-built static library
link_libraries(${KORTEX_DIR}lib/${KORTEX_LIB_SUBDIR}/libKortexApiCpp.a)

# Use bundled include directories
include_directories(${KORTEX_DIR}include)
include_directories(${KORTEX_DIR}include/client)
include_directories(${KORTEX_DIR}include/common)
include_directories(${KORTEX_DIR}include/messages)
include_directories(${KORTEX_DIR}include/client_stubs)

# Use bundled vicon include directory
set(VICON_DIR "${THIRDPARTY_DIR}/vicon_api")
set(VICON_INCLUDE_DIRS ${VICON_DIR}/include)
set(VICON_LIBRARIES 
    ${VICON_DIR}/lib/libViconDataStreamSDK_CPP.so
    ${VICON_DIR}/lib/libboost_system-mt-x64.so
    ${VICON_DIR}/lib/libboost_thread-mt-x64.so
    ${VICON_DIR}/lib/libboost_chrono-mt-x64.so
    ${VICON_DIR}/lib/libboost_filesystem-mt-x64.so
)
message(STATUS "Using bundled Vicon DataStream SDK: ${VICON_LIBRARIES}")

# Global include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
    ${GTSAM_INCLUDE_DIRS}
    ${GPMP2_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${YAMLCPP_INCLUDE_DIRS}
    ${EZC3D_INCLUDE_DIRS}
    ${PINOCCHIO_INCLUDE_DIRS}
    ${OPEN3D_INCLUDE_DIRS}
    ${VICON_INCLUDE_DIRS}
    ${THIRDPARTY_INCLUDE_DIR}
)

# Add the thirdparty lib directory to the library search path
link_directories(${THIRDPARTY_LIB_DIR} ${VICON_DIR}/lib)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra -D_OS_UNIX")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")

# Add linker flags for better symbol resolution
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-as-needed")

# Add subdirectories
add_subdirectory(TrajectoryGeneration)
add_subdirectory(TrajectoryExecution)
add_subdirectory(ViconDataStream)
add_subdirectory(TrajectoryRealTime)


function(add_project_executable target_name source_file)
    add_executable(${target_name} ${source_file})
    
    # Link against the libraries from subdirectories and bundled libraries
    target_link_libraries(${target_name} 
        TrajectoryGeneration
        TrajectoryExecution
        TrajectoryRealTime
        ViconDataStream
        ${GPMP2_LIBRARIES}        
        ${GTSAM_LIBRARIES}        
        ${EZC3D_LIBRARIES}
        ${PINOCCHIO_LIBRARIES}
        ${OPEN3D_LIBRARIES}
        ${VICON_LIBRARIES}
        ${YAMLCPP_LIBRARIES}
        console_bridge
        pthread
        dl
    )

    # Include directories
    target_include_directories(${target_name} PRIVATE
        TrajectoryGeneration/include
        TrajectoryExecution/include
        TrajectoryRealTime/include
        ViconDataStream/include
    )

    # Set RPATH so the executable can find the bundled libraries
    set_target_properties(${target_name} PROPERTIES
        INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/lib:${CMAKE_CURRENT_SOURCE_DIR}/third_party/vicon_api/lib"
        BUILD_WITH_INSTALL_RPATH TRUE
        INSTALL_RPATH_USE_LINK_PATH TRUE
    )
endfunction()

# Copy config files to build directory
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config/joint_limits.yaml
    ${CMAKE_CURRENT_BINARY_DIR}/config/joint_limits.yaml
    COPYONLY
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config/dh_params.yaml
    ${CMAKE_CURRENT_BINARY_DIR}/config/dh_params.yaml
    COPYONLY
)

# add executables using function macro
add_project_executable(main main.cpp)

# add_project_executable(money_file money_file.cpp)
# add_project_executable(test_vicon test_vicon.cpp)
# add_project_executable(test_gpmp2_vs_rrt_star test_gpmp2_vs_rrt_star.cpp)

add_project_executable(test_vicon_subject test_vicon_subject.cpp)
# add_project_executable(test_kinova test_kinova.cpp)
# add_project_executable(test_task_impedance test_task_impedance.cpp)
add_project_executable(test_joint_impedance test_joint_impedance.cpp)
# add_project_executable(test_joint_position test_joint_position.cpp)
add_project_executable(test_generation_and_joint_position test_generation_and_joint_position.cpp)
# add_project_executable(test_pose_1 test_pose_1.cpp)
# add_project_executable(workspace_analysis workspace_analysis.cpp)
# add_project_executable(test_sleep test_sleep.cpp)
# Installation
#install(TARGETS money_file test_vicon main
#    RUNTIME DESTINATION bin
#)