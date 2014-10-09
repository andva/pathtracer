cmake_minimum_required(VERSION 2.6.0)
project(paradoxPath)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(LIBRARY_OUTPUT_PATH
    ${PROJECT_BINARY_DIR}/lib
    CACHE PATH
    "Directory where all libraries will be stored"
)

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
		/MT
		/Wall /WX- /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS
		)
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

#set_target_properties(runUnitTests PROPERTIES COMPILE_FLAGS "--coverage")
target_link_libraries(runUnitTests gtest gtest_main pathfinder)
#target_link_libraries(runUnitTests ${GTEST_LIBRARY} ${GTEST_MAIN_LIBRARY} pathfinder)

add_test(
  NAME test
  COMMAND runUnitTests
)