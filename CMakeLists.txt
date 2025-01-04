cmake_minimum_required(VERSION 3.10)
project(depth_img_normal_estimation)

set(CATKIN_PACKAGE_DEPENDENCIES
    roscpp
    roslib
    cv_bridge
    image_transport
    )

find_package(catkin REQUIRED COMPONENTS
          ${CATKIN_PACKAGE_DEPENDENCIES}
        )

###################################
## catkin specific configuration ##
###################################

catkin_package(
   INCLUDE_DIRS 
        include
   LIBRARIES
        ${PROJECT_NAME}
   CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# Main library
add_library(${PROJECT_NAME}
    src/NormalEstimator.cpp
)

add_dependencies(${PROJECT_NAME} 
              ${${PROJECT_NAME}_EXPORTED_TARGETS} 
              ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

## Declare a C++ executable

# Normal estimation node
add_executable(normal_estimation_node
        src/NormalEstimationNode.cpp
        )
add_dependencies(normal_estimation_node
        ${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(normal_estimation_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

#############
## Testing ##
#############
