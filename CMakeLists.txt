cmake_minimum_required(VERSION 2.6.0)
project(paradoxPath)

IF(NOT MSVC)
	message(FATAL_ERROR "Other platforms and compilers not supported")
ENDIF()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGTEST_HAS_TR1_TUPLE=0 -DGTEST_USE_OWN_TR1_TUPLE=1 ")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT ")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd ")

ADD_SUBDIRECTORY(pathfinder)
ADD_SUBDIRECTORY(main)
ADD_SUBDIRECTORY("external/gtest-1.7.0")
enable_testing()

list(APPEND paradoxPath_INCLUDE_DIRS
	${gtest_SOURCE_DIR}/include 
	${gtest_SOURCE_DIR}
	${PROJECT_SOURCE_DIR}
)
include_directories(
	${paradoxPath_INCLUDE_DIRS}
)

FILE(GLOB paradoxPath_TESTS "${PROJECT_SOURCE_DIR}/test/*.cpp")

add_executable(runUnitTests
    ${paradoxPath_TESTS}
)

target_link_libraries(runUnitTests gtest gtest_main pathfinder)

add_test(
  NAME test
  COMMAND runUnitTests
)