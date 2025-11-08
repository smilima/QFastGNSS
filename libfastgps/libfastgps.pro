# fastgps-qt/libfastgps/libfastgps.pro
TEMPLATE = lib
CONFIG += staticlib debug_and_release c++17

# Add Qt dependencies for concurrent processing
QT += core concurrent

TARGET = fastgps

# Put the final library in a common build lib folder
DESTDIR = $$top_builddir/lib

# Add all C and C++ source files from the library
SOURCES += \
    acquire.cpp \
    code_table.cpp \
    correlator.cpp \
    datetime.cpp \
    dopp_pos.cpp \
    ephemerides.cpp \
    fastgps.cpp \
    gnss_utils.cpp \
    intrpsp3c.cpp \
    kiss_fft.c \
    kiss_fftr.c \
    lin_alg.cpp \
    nav.cpp \
    snap_shot.cpp \
    tracking.cpp

# Add all header files
HEADERS += \
    _kiss_fft_guts.h \
    acquisition.h \
    datetime.h \
    dopp_pos.h \
    fastgps.h \
    funcs.h \
    globals.h \
    intrpsp3c.h \
    kiss_fft.h \
    kiss_fftr.h \
    parameters.h \
    snap_shot.h \
    structs.h \
    types.h

# Specify the include path
INCLUDEPATH += .

# Suppress warnings from the original code
QMAKE_CFLAGS += -Wno-unused-parameter -Wno-implicit-function-declaration
QMAKE_CXXFLAGS += -Wno-unused-parameter -Wno-misleading-indentation -Wno-unused-but-set-variable -Wno-array-parameter
