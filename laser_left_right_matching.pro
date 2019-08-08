QT += core
QT -= gui

CONFIG += c++11

TARGET = laser_left_right_matching
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

DEFINES+= \
    IMG_DISPLAY \
#    CAMERA_HORIZONTAL \
    PCL_PROCESS \
#    OUTPUT_POINTS_DEBUG \
#    对比时再打开
#    MATCH_COMPARE \
#    统计激光点的个数
    POINTS_STATISTICES \

HEADERS += \
    Zhangsuen.h \
    StereoMatchLeftAndRight.h \
    Steger.h \
    EpipolarMatching.h \
    Utils.h \
    ReferenceLine.h \
    SortAlgorithm.h \
    DisparityAndDepth.h \
    RetrievePointCloud.h \
    DisplayImg.h \
    BindAlgorithm.h

SOURCES += \
    Zhangsuen.cpp \
    main.cpp \
    StereoMatchLeftAndRight.cpp \
    Steger.cpp \
    EpipolarMatching.cpp \
    ReferenceLine.cpp \
    SortAlgorithm.cpp \
    DisparityAndDepth.cpp \
    RetrievePointCloud.cpp \
    DisplayImg.cpp \
    BindAlgorithm.cpp

INCLUDEPATH += \
#opencv2
#/usr/local/opencv2.4.13/include \
#/usr/local/opencv2.4.13/include/opencv \
#opencv3
/usr/local/opencv320/include \
/usr/local/opencv320/include/opencv \
/usr/local/opencv320/include/opencv2 \
/usr/local/opencv320/include/opencv2/flann \
#pcl1.8
/usr/local/pcl1.8.0/include/pcl-1.8 \
/usr/include/vtk-6.3 \

LIBS +=  \
#opencv2
#/usr/local/opencv2.4.13/lib/libopencv_*.so \
#opencv3
/usr/local/opencv320/lib/libopencv_*.so \
#pcl
/usr/local/pcl1.8.0/lib/libpcl_*.so \
/usr/lib/libvtk*.so \
/usr/lib/x86_64-linux-gnu/libboost_*.so \
