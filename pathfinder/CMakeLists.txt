set(PATHFINDER_SRC
	pathfinder.cpp
	searchspace.cpp
	node.cpp
	vec2.cpp
    objectpool.cpp
	)

set(PATHFINDER_HEAD 
	pathfinder.h
	searchspace.h
	node.h
	vec2.h
    objectpool.h
	)

set(PLATFORM_COMPILE_FLAGS
	#/Wall
	)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PLATFORM_COMPILE_FLAGS} ")

add_library(pathfinder
	${PATHFINDER_HEAD}
	${PATHFINDER_SRC})