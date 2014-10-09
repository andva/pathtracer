set(PATHFINDER_SRC
	pathfinder.cpp
	)

set(PATHFINDER_HEAD 
	pathfinder.h
	)

set(PLATFORM_COMPILE_FLAGS
	/DNT_PLUGIN
    /DREQUIRE_IOSTREAM
	)

add_definitions(${PLATFORM_COMPILE_FLAGS})

add_library(pathfinder
	${PATHFINDER_HEAD}
	${PATHFINDER_SRC})