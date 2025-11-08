# fastgps-qt/fastgps-qt.pro
TEMPLATE = subdirs

SUBDIRS = \
    libfastgps \
    qtfastgps

qtfastgps.depends = libfastgps
