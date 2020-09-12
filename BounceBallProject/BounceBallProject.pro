#-------------------------------------------------
#
# Project created by QtCreator 2020-09-12T13:28:38
#
#-------------------------------------------------

QT       += core gui multimedia \
            multimediawidgets \
            network \
            serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = BounceBallProject
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        mainwindow.cpp \
    imageprocessing.cpp \
    sender.cpp

HEADERS += \
        mainwindow.h \
    imageprocessing.h \
    sender.h

FORMS += \
        mainwindow.ui

INCLUDEPATH += /usr/local/include/opencv4/

LIBS += -L/usr/local/lib -lopencv_core \
         -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lopencv_calib3d \
        -lopencv_features2d -lopencv_videoio -lopencv_video

