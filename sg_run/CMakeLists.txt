cmake_minimum_required( VERSION 3.10 )

project( simulation_setup )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#Optimization parameters
set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_CXX_FLAGS "-O2")

# Add SGConnect
message(STATUS "Attempting to add the SGCoreCpp project at ${CMAKE_CURRENT_SOURCE_DIR}../../")
include(${CMAKE_CURRENT_SOURCE_DIR}/libs/SGConnect/LinkSGConnect.cmake) # Runs the module that include SGCoreCpp into this project
message(STATUS "Done!")

# Add SGCore
message(STATUS "Attempting to add the SGCoreCpp project at ${CMAKE_CURRENT_SOURCE_DIR}../../")
include(${CMAKE_CURRENT_SOURCE_DIR}/libs/SGCoreCpp/LinkSGCoreCpp.cmake) # Runs the module that include SGCoreCpp into this project
message(STATUS "Done!")

# Include directories
include_directories(
    ./include
    /opt/ros/noetic/include
    /opt/ros/noetic/lib
    )

set(ALL_LIBS
    #${SGCONNECT_BIN_DEBUG}
    ${SGCONNECT_BIN_RELEASE}
    #${SGCORE_BIN_DEBUG}
    ${SGCORE_BIN_RELEASE}
)

set(SOURCES)


# target
add_executable(main main.cpp ${SOURCES})

target_include_directories(main PRIVATE )

    
target_link_libraries(main ${ALL_LIBS}
        -I/opt/ros/noetic/include
        -L/opt/ros/noetic/lib
        -lroscpp
        -lrostime
        -lrosconsole
        -lroscpp_serialization
)