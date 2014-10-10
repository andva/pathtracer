set(PATHFINDER_SRC
	pathfinder.cpp
	)

set(PATHFINDER_HEAD 
	pathfinder.h
	)

set(PLATFORM_COMPILE_FLAGS
	#/Wall
	)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PLATFORM_COMPILE_FLAGS} ")

add_library(pathfinder
	${PATHFINDER_HEAD}
	${PATHFINDER_SRC})