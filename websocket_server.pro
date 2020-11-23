TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
#DEFINES += BOOST_USE_LIB

#INCLUDEPATH += /usr/include/boost
#HEADERS += $$PWD/websocketpp/*.hpp
#INCLUDEPATH += $$PWD/websocketpp/*

SOURCE_DIR = $$PWD/asio-1.18.0
INCLUDEPATH += \
$$SOURCE_DIR/include 

HEADERS += \
$$SOURCE_DIR/include/*.hpp \
    echo_handler.hpp

## mavlink
## select mavlink version
DEFINES += mavlink_v1
contains(DEFINES, mavlink_v1){
	INCLUDEPATH += \
	$$PWD/mavlink_headers/c_library_v1
	HEADERS += \
		$$PWD/mavlink_headers/c_library_v1/*.hpp
	message("mavlink_v1 version used")
}else{
    
	INCLUDEPATH += \
	$$PWD/mavlink_headers/c_library_v2
	HEADERS += \
		$$PWD/mavlink_headers/c_library_v2/*.hpp
	message("mavlink_v2 version used")
}
DEFINES += ASIO_STANDALONE
## send drone state sim data
DEFINES += dronestate_sim_send
#LIBS +=/usr/lib/libboost_thread.a
#LIBS +=/usr/lib/libboost_system.a
LIBS += -lpthread
SOURCES += \
	broadcast_server.cpp
    #client.cpp
