cmake_minimum_required(VERSION 2.8.3)
project(test)

find_package(urdfdom REQUIRED)

add_definitions(-DTEST_RESOURCE_LOCATION="${CMAKE_CURRENT_SOURCE_DIR}/res")

add_executable(${PROJECT_NAME}_parser test_parser.cpp)
target_link_libraries(${PROJECT_NAME}_parser 
                      srdfdom ${urdfdom_LIBRARIES})

