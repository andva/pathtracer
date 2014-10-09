cmake_minimum_required(VERSION 2.6.0)
project(paradoxPath)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

list(APPEND paradoxPath_INCLUDE_DIRS
	${CMAKE_SOURCE_DIR}/external/gtest-1.7.0
)

include_directories(
	${paradoxPath_INCLUDE_DIRS}
)

IF(UNIX)
	add_definitions(-std=c++11 -Wshadow -Wpointer-arith -Wcast-qual --coverage -Werror -Wall -Werror=sign-compare)
ELSE()
	add_definitions(
		-DGTEST_HAS_TR1_TUPLE=0
		-DGTEST_USE_OWN_TR1_TUPLE=1
		)
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
ENDIF(UNIX)

SET (CMAKE_SHARED_LINKER_FLAGS ${CMAKE_SHARED_LINKER_FLAGS_INIT} $ENV{LDFLAGS} CACHE STRING "-lgcov")
SET (CMAKE_STATIC_LINKER_FLAGS ${CMAKE_STATIC_LINKER_FLAGS_INIT} $ENV{LDFLAGS} CACHE STRING "-gcov")

ADD_SUBDIRECTORY(pathfinder)

ADD_SUBDIRECTORY("external/gtest-1.7.0")
enable_testing()

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR} ${PROJECT_SOURCE_DIR})

FILE(GLOB paradoxPath_TESTS "${PROJECT_SOURCE_DIR}/test/*.cpp")

add_executable(runUnitTests
    ${paradoxPath_TESTS}
)

target_link_libraries(runUnitTests gtest gtest_main pathfinder)

add_test(
  NAME test
  COMMAND runUnitTests
)