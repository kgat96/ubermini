
QT       += core gui printsupport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = spectrum
TEMPLATE = app

CONFIG   += console

LIBS += -lusb-1.0

INCLUDEPATH += ../qcustomplot
INCLUDEPATH += /usr/include/libusb-1.0

SOURCES += main.cpp\
          ../qcustomplot/qcustomplot.cpp \
          uberusb.c \
          mainwindow.cpp

HEADERS  += mainwindow.h \
            uberusb.h \
            ../qcustomplot/qcustomplot.h

FORMS    += mainwindow.ui

#RESOURCES += mainwindow.qrc
#RC_ICONS += ./images/voice.ico

