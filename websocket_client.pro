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
    echo_handler.hpp \
    strconvert.hpp \
    mavlink_public.hpp

DEFINES += ASIO_STANDALONE

#LIBS +=/usr/lib/libboost_thread.a
#LIBS +=/usr/lib/libboost_system.a
LIBS += -lpthread

## mavlink
INCLUDEPATH += \
$$PWD/mavlink_headers
HEADERS += \
	$$PWD/mavlink-headers/*.hpp

SOURCES += \
    echo_client.cpp
	
    #client.cpp
