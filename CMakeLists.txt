### CMake file for Flying_Sebs
###
###   -  2010: Initial version <druon@lirmm.fr>
###
###
###############################################################################

cmake_minimum_required (VERSION 2.6)

project (Flying_Sebs)

set (Flying_Sebs_VERSION_MAJOR 0)
set (Flying_Sebs_VERSION_MINOR 1)

set( CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /lib /usr/lib /usr/local/lib )
set( CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} /usr/include /usr/local/include )
set( CMAKE_MODULE_PATH  ${CMAKE_MODULE_PATH}  "${PROJECT_SOURCE_DIR}/cmake" ) 

set ( INSTALL_BIN_PATH bin )
set ( INSTALL_LIB_PATH lib )
set ( INSTALL_AR_PATH  lib )
set ( INSTALL_HEADERS_PATH include )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__STDC_CONSTANT_MACROS ")

set( CMAKE_MODULE_PATH  "/usr/local/lib/cmake/"
			"${PROJECT_SOURCE_DIR}/cmake" )


# Make relative paths absolute (needed later on)

foreach(p LIB BIN HEADERS AR)
    set(var INSTALL_${p}_PATH)
    if(NOT IS_ABSOLUTE "${${var}}")
        set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
    endif()
endforeach()


### Search for Lib/Headers

find_package( RT_ARDrone REQUIRED )
find_package(aruco REQUIRED)


### Set include directories 
        
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/include"
	             "${CMAKE_CURRENT_SOURCE_DIR}/src"
        	     "${CMAKE_CURRENT_BINARY_DIR}"
                     "${CMAKE_INSTALL_PREFIX}/include" 
		     "${RT_ARDRONE_INCLUDE_DIRS}"
			"/usr/local/include/aruco")

link_directories ( ${aruco_LIB_DIR})



### File Lists

file( GLOB hdr "src/*.h*" )
file( GLOB src "src/*.cpp" "src/*.c" )

### Main Target and linking

include_directories("${RT_ARDRONE_INCLUDE_DIRS}/../tools/drone_videostream/src")

add_executable( "Flying_Sebs" 	
			"src/test.cpp" 
			${hdr} )

target_link_libraries ( "Flying_Sebs" ${CMAKE_THREAD_LIBS_INIT}
					${RT_ARDRONE_LIBRARIES} 
					${SDL_LIBRARY} 
					"Xxf86vm" 
					${aruco_LIBS})



add_executable( 	"Calibration" 	
			"src/camera_calibration.cpp" 
			${hdr} )

target_link_libraries ( "Calibration" ${CMAKE_THREAD_LIBS_INIT}
					${RT_ARDRONE_LIBRARIES} 
					${SDL_LIBRARY} 
					"Xxf86vm" 
					${aruco_LIBS})

add_executable( 	"Calibration_below" 	
			"src/camera_calibration_below.cpp" 
			${hdr} )

target_link_libraries ( "Calibration_below" ${CMAKE_THREAD_LIBS_INIT}
					${RT_ARDRONE_LIBRARIES} 
					${SDL_LIBRARY} 
					"Xxf86vm" 
					${aruco_LIBS})



add_executable( 	"Drone_tracking" 	
			"src/drone_tracking.cpp" 
			${hdr} )
				

target_link_libraries ( "Drone_tracking" ${CMAKE_THREAD_LIBS_INIT}
					${RT_ARDRONE_LIBRARIES} 
					${SDL_LIBRARY} 
					"Xxf86vm" 
					${aruco_LIBS})

add_executable( 	"Drone_tracking_below" 	
			"src/drone_tracking_below.cpp" 
			${hdr} )
				

target_link_libraries ( "Drone_tracking_below" ${CMAKE_THREAD_LIBS_INIT}
					${RT_ARDRONE_LIBRARIES} 
					${SDL_LIBRARY} 
					"Xxf86vm" 
					${aruco_LIBS})


find_package(MoGS_Joystick)

if (MoGS_Joystick_FOUND)

include_directories(${MoGS_Joystick_INCLUDE_DIRS})
link_directories(${MoGS_Joystick_LIBRARY_DIRS})

add_executable( 	"Drone_pad" 	
			"src/drone_pad.cpp" 
			${hdr} )
				

target_link_libraries ( "Drone_pad" ${CMAKE_THREAD_LIBS_INIT}
					${RT_ARDRONE_LIBRARIES} 
					${SDL_LIBRARY} 
					"Xxf86vm" 
					${aruco_LIBS}
					${MoGS_Joystick_LIBRARIES})


endif (MoGS_Joystick_FOUND)

### Install targets

install( TARGETS "Flying_Sebs"
        RUNTIME DESTINATION ${INSTALL_BIN_PATH}  COMPONENT main
        LIBRARY DESTINATION ${INSTALL_LIB_PATH}  COMPONENT main
        ARCHIVE DESTINATION ${INSTALL_AR_PATH}   COMPONENT main )