TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++11
QMAKE_CXXFLAGS_RELEASE += -O3

#QMAKE_CXXFLAGS +=  -mfpu=neon # z-march=armv7-a -marm -mthumb-interwork -mfloat-abi=hard
#QMAKE_CFLAGS += -mfloat-abi=softfp #-mfpu=neon -march=armv7-a -marm -mthumb-interwork


INCLUDEPATH += /usr/local/include \
               /usr/local/include/opencv \
               /usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_core.so    \
        /usr/local/lib/libopencv_imgproc.so \
        /usr/local/lib/libopencv_imgcodecs.so \
        -pthread
      # -ldl
LIBS += -L/usr/local/lib -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_objdetect -lopencv_highgui -lopencv_videoio -lopencv_photo -lopencv_imgcodecs -lopencv_video -lopencv_ml -lopencv_imgproc -lopencv_flann -lopencv_core

SOURCES += main.cpp \
    serial.cpp \
    RuneResFilter.cpp \
    ArmorDetector.cpp \
    Predictor.cpp \
    ImageConsProd.cpp \
    RemoteController.cpp \
    AngleSolver.cpp \
    RMVideoCapture.cpp \
    Runedetector.cpp

HEADERS += \
    serial.h \
    cmdline.h \
    RuneDetector.hpp \
    RuneResFilter.hpp \
    Settings.hpp \
    ArmorDetector.hpp \
    Predictor.hpp \
    ImageConsProd.hpp \
    RemoteController.hpp \
    AngleSolver.hpp \
    RMVideoCapture.hpp \
    LedController.hpp \
    kalman.hpp \
    pid.hpp \
    Runedetector.hpp

