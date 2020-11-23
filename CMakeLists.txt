cmake_minimum_required (VERSION 2.6)   #版本要求
project (websocket_client)                     #工程名称

#option (ENABLE_CPP11 "Build websocketpp with CPP11 features enabled." TRUE)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread " )
option(ASIO_STANDALONE "Set to use boost" ON)
option(dronestate_sim_send "Set to send mavlink msg from server" ON)
option(mavlink_v2  "Set to use mavlink_v2"  ON)  #use mavlink_v2
option(mavlink_v1  "Set to use mavlink_v1"  OFF) #use mavlink_v1
message ("mavlink_v2: " ${mavlink_v2})
message ("mavlink_v1: " ${mavlink_v1})
if (mavlink_v2)
	ADD_DEFINITIONS(-Dmavlink_v2)
	message ("mavlink_v2 is used")
elseif(mavlink_v1)
	ADD_DEFINITIONS(-Dmavlink_v1)
	message ("mavlink_v1 is used")
endif ()
find_package (Boost 1.39.0 COMPONENTS system thread)


if (Boost_FOUND)
# Boost is a project wide global dependency.
	include_directories (${Boost_INCLUDE_DIRS})
	link_directories (${Boost_LIBRARY_DIRS})

# Pretty print status
	message (STATUS "-- Include Directories")
	foreach (include_dir ${Boost_INCLUDE_DIRS})
    	message (STATUS "       " ${include_dir})
	endforeach ()
		message (STATUS "-- Library Directories")
	foreach (library_dir ${Boost_LIBRARY_DIRS})
    	message (STATUS "       " ${library_dir})
	endforeach ()
		message (STATUS "-- Libraries")
	foreach (boost_lib ${Boost_LIBRARIES})
    	message (STATUS "       " ${boost_lib})
	endforeach ()
message ("")
else ()
message (FATAL_ERROR "Failed to find required dependency: boost")
endif ()

if (mavlink_v2)
	include_directories( ./ ./asio-1.18.0/include ./mavlink_headers/c_library_v2/ ./websocketpp) #添加头文件
	FILE(GLOB_RECURSE CURRENT_INCLUDE ./asio-1.18.0/include/*.hpp ./mavlink_headers/c_library_v2/*.h ./websocketpp/*.hpp ./*.hpp)
	message ("mavlink_v2 is used")
	#message (${CURRENT_INCLUDE})
else ()
	include_directories( ./ ./asio-1.18.0/include ./mavlink_headers/c_library_v1 ./websocketpp) #添加头文件
	FILE(GLOB_RECURSE CURRENT_INCLUDE ./asio-1.18.0/include/*.hpp ./mavlink_headers/c_library_v1/*.h ./websocketpp/*.hpp ./*.hpp)
	message ("mavlink_v1 is used")
endif ()
#message (${CURRENT_INCLUDE})
add_executable(websocket_client echo_client.cpp ${CURRENT_INCLUDE} ${Boost_LIBRARIES})  
target_link_libraries(websocket_client ${Boost_LIBRARIES})
#add_executable(websocket_server echo_server.cpp ${CURRENT_INCLUDE} ${Boost_LIBRARIES})  
#target_link_libraries(websocket_server ${Boost_LIBRARIES})
add_executable(websocket_server broadcast_server.cpp ${CURRENT_INCLUDE} ${Boost_LIBRARIES})  
target_link_libraries(websocket_server ${Boost_LIBRARIES})
