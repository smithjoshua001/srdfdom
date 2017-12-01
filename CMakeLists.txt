cmake_minimum_required(VERSION 2.8.3)
project(srdfdom_advr)

function(ExtractLibraries outputVar libraryHints)
	foreach(LIB ${ARGN})
		message(STATUS "LIB: ${LIB}")
		find_library(TMP ${LIB} HINTS ${libraryHints})
		set( ${outputVar} ${${outputVar}} ${TMP})
		unset(TMP CACHE)
	endforeach(LIB)
  set(${outputVar} ${${outputVar}} PARENT_SCOPE)
endfunction()

if (NOT MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

find_package(urdfdom_headers REQUIRED)
include_directories(BEFORE SYSTEM ${urdfdom_headers_INCLUDE_DIRS})


find_package(PkgConfig REQUIRED)
pkg_search_module(TINYXML2 REQUIRED TinyXML2 tinyxml2)
ExtractLibraries(TINYXML_LIBRARIES ${TINYXML2_LIBRARY_DIRS} ${TINYXML2_LIBRARIES})
message(STATUS "TINYXML2 LIBS: ${TINYXML_LIBRARIES}")



message(STATUS "TINYXML_VERSION: ${TINYXML2_VERSION}")
include_directories(BEFORE SYSTEM ${TINYXML2_INCLUDE_DIRS})
link_directories(${TINYXML2_LIBRARY_DIRS})
message(STATUS "TINYXML_LIBRARIES: ${TINYXML_LIBRARIES}")
message(STATUS "TINYXML_LIBRARY_DIR: ${TINYXML2_LIBRARY_DIR}")
message(STATUS "TINYXML_INCLUDE_DIR: ${TINYXML2_INCLUDE_DIRS}")
message(STATUS "TINYXML_LIBRARY: ${TINYXML2_LIBRARY}")

include_directories(include ${TINYXML_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED src/model.cpp)
target_link_libraries(${PROJECT_NAME} ${TINYXML_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  DESTINATION "lib"
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION "include/${PROJECT_NAME}"
  FILES_MATCHING PATTERN "*.h"
)

set(LIBRARY_NAME "${CMAKE_PROJECT_NAME}")
# Configuration installation

configure_file("${CMAKE_PROJECT_NAME}-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config.cmake"
    @ONLY)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config.cmake"
    DESTINATION "share/${CMAKE_PROJECT_NAME}")


#######################
# Add Testing target  #
#######################
if(SRDFDOM_COMPILE_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()
