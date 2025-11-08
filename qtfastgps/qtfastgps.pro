# fastgps-qt/qtfastgps/qtfastgps.pro
TEMPLATE = app
CONFIG += debug_and_release c++11
QT += widgets concurrent

TARGET = qtfastgps

# Link against the libfastgps library from the common build lib folder
# This path points to the 'lib' folder at the root of the build directory
LIBS += -L$$top_builddir/lib -lfastgps

# Include the libfastgps headers
INCLUDEPATH += ../libfastgps

SOURCES += \
    configeditordialog.cpp \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    configeditordialog.h \
    mainwindow.h

FORMS +=
