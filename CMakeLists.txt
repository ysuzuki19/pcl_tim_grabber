CMAKE_MINIMUM_REQUIRED(VERSION 3.7...3.18)

PROJECT(visualize_tim VERSION 1.0 LANGUAGES CXX)
ADD_EXECUTABLE(visualize_tim main.cpp)
SET_PROPERTY(TARGET visualize_tim PROPERTY CXX_STANDARD 17)

SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_MULTITHREADED ON)
SET(Boost_USE_STATIC_RUNTIME OFF)

FIND_PACKAGE(Boost 1.66.0 REQUIRED thread system)
IF(Boost_FOUND)
	INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
	LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
ENDIF(Boost_FOUND)
SET(USERD_LIBS ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

FIND_PACKAGE( PCL 1.12 REQUIRED )
IF(PCL_FOUND)
	INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
	ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
	LINK_DIRECTORIES( ${PCL_LIBRARY_DIRS} )
	TARGET_LINK_LIBRARIES( visualize_tim ${PCL_LIBRARIES} )
ENDIF()


TARGET_LINK_LIBRARIES(visualize_tim pthread)
TARGET_LINK_LIBRARIES(visualize_tim ${USED_LIBS})
