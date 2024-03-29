cmake_minimum_required(VERSION 2.8.3)
project(scriptctrl_example)

find_package(catkin REQUIRED COMPONENTS
  any_node
  rocoma_ros
  roco
  message_logger
  rocomaex_model
)

find_package(Lua 5.3 EXACT REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS any_node rocoma_ros roco message_logger rocomaex_model
  DEPENDS
)

###########
## Build ##
###########
add_definitions(-std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LUA_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/ScriptCtrl.cpp
  src/lcm.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${LUA_LIBRARIES}
)

# Declare a C++ executable
add_executable(${PROJECT_NAME}_exec src/${PROJECT_NAME}_exec.cpp)

add_dependencies(${PROJECT_NAME}_exec ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_exec
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(DIRECTORY
#   param
#   launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
