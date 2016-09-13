#-------------------------------------------------
#
# Project created by QtCreator 2014-05-01T14:24:33
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET += pcl_visualizer
TEMPLATE = app

INCLUDEPATH += -I/usr/local/include/opencv
INCLUDEPATH += -I/usr/local/include
INCLUDEPATH += ../src/include_files
LIBS += -L/usr/local/lib
LIBS += -lopencv_core
LIBS += -lopencv_imgproc
LIBS += -lopencv_highgui
LIBS += -lopencv_ml
LIBS += -lopencv_video
LIBS += -lopencv_features2d
LIBS += -lopencv_calib3d
LIBS += -lopencv_objdetect
LIBS += -lopencv_contrib
LIBS += -lopencv_legacy
LIBS += -lopencv_flann
LIBS += -lopencv_nonfree


SOURCES += main.cpp pclviewer.cpp include_files/LmdFuncs.cpp

HEADERS  += pclviewer.h include_files/LmdFuncs.h

FORMS    += pclviewer.ui

