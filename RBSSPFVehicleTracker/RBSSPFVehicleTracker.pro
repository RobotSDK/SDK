#-------------------------------------------------
#
# Project created by QtCreator 2015-08-03T11:43:46
#
#-------------------------------------------------

QT       -= gui

CONFIG += c++11

TARGET = RBSSPFVehicleTracker
TEMPLATE = lib
CONFIG += staticlib

SOURCES += rbsspfvehicletracker.cpp

HEADERS += rbsspfvehicletracker.h \
    rbsspfvehicletracker.cuh

DISTFILES += \
    rbsspfvehicletracker.cu

include($$(ROBOTSDKCUDA))

MOC_DIR = $$(HOME)/SDK/RBSSPFVehicleTracker/Build/MOC
UI_DIR = $$(HOME)/SDK/RBSSPFVehicleTracker/Build/UI

CONFIG(debug, debug|release){
        OBJECTS_DIR = $$(HOME)/SDK/RBSSPFVehicleTracker/Build/OBJ/Debug
        DESTDIR = $$(HOME)/SDK/RBSSPFVehicleTracker/Build/lib
        TARGET = RBSSPFVehicleTracker_Debug
        target.path = $$(HOME)/SDK/RBSSPFVehicleTracker/lib
}
else {
        OBJECTS_DIR = $$(HOME)/SDK/RBSSPFVehicleTracker/Build/OBJ/Release
        DESTDIR = $$(HOME)/SDK/RBSSPFVehicleTracker/Build/lib
        TARGET = RBSSPFVehicleTracker_Release
        target.path = $$(HOME)/SDK/RBSSPFVehicleTracker/lib
}

INSTALLS += target

headertarget.files = $$HEADERS
headertarget.path = $$(HOME)/SDK/RBSSPFVehicleTracker/include

INSTALLS += headertarget
