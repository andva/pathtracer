set(MAIN_SRC
	main.cpp
	)

set(PLATFORM_COMPILE_FLAGS
	#/Wall
	)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PLATFORM_COMPILE_FLAGS} ")

include_directories(
	${PROJECT_SOURCE_DIR}
)

add_executable(main
    ${MAIN_SRC}
)

target_link_libraries(main pathfinder)