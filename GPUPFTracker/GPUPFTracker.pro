#-------------------------------------------------
#
# Project created by QtCreator 2015-07-08T16:48:00
#
#-------------------------------------------------

QT       -= gui

TARGET = GPUPFTracker
TEMPLATE = lib
CONFIG += staticlib

SOURCES +=

HEADERS += \
    egotransform.h \
    particlefilterbase.h \
    particlefilterdef.h \
    randomgenerator.h

DISTFILES += \
    egotransform.cu

MOC_DIR = $$(HOME)/SDK/$$TARGET/Build/MOC
UI_DIR = $$(HOME)/SDK/$$TARGET/Build/UI

CONFIG(debug, debug|release){
    OBJECTS_DIR = $$(HOME)/SDK/$$TARGET/Build/OBJ/Debug
    DESTDIR = $$(HOME)/SDK/$$TARGET/Build/lib/Debug
    target.path = $$(HOME)/SDK/$$TARGET/lib/Debug
}
else{
    OBJECTS_DIR = $$(HOME)/SDK/$$TARGET/Build/OBJ/Release
    DESTDIR = $$(HOME)/SDK/$$TARGET/Build/lib/Release
    target.path = $$(HOME)/SDK/$$TARGET/lib/Release
}
INSTALLS += target

headers.path=$$(HOME)/SDK/$$TARGET/include
headers.files=$$HEADERS
INSTALLS += headers

include($$(ROBOTSDKCUDA))
