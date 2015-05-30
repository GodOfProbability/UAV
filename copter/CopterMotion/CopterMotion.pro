QT += serialport
QT += widgets
QMAKE_CC = gcc
QMAKE_CXX = g++
CONFIG += release c++11

LIBS += `pkg-config opencv --libs`

HEADERS         = MAVLinkConnection.h \
                  MainWindow.h \
    CaptureThread.h \
    Chameleon.h \
    ConfigFile.h \
    ControlThread.h \
    ImageTracker.h \
    WebcamTracker.h

SOURCES         = main.cpp \
                  MAVLinkConnection.cpp \
                  MainWindow.cpp \
    CaptureThread.cpp \
    Chameleon.cpp \
    ConfigFile.cpp \
    ControlThread.cpp \
    ImageTracker.cpp \
    WebcamTracker.cpp


target.path = CopterMotion
INSTALLS += target
INCLUDEPATH += /home/odroid/GCS_MAVLink/include/mavlink/150319_c_library-master/common/
INCLUDEPATH += /usr/include/
INCLUDEPATH += /usr/include/arm-linux-gnueabihf/

