cmake_minimum_required(VERSION 2.8.3)
project(srdfdom_advr)

find_package(Boost REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS rosconsole_bridge urdf cmake_modules)

find_package(TinyXML REQUIRED)

include_directories(include ${TinyXML_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include ${TinyXML_INCLUDE_DIRS}
  DEPENDS rosconsole_bridge urdf
)

add_library(${PROJECT_NAME} src/model.cpp)
target_link_libraries(${PROJECT_NAME} ${TinyXML_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#######################
# Add Testing target  #
#######################
if(SRDFDOM_COMPILE_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()
