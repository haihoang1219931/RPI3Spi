CONFIG += console c++11

INCLUDEPATH += /home/hai/raspi/sysroot/usr/include
DEPENDPATH += /home/hai/raspi/sysroot/usr/include

LIBS += -L/home/hai/raspi/sysroot/usr/lib -lwiringPi -lwiringPiDev

SOURCES += \
    main_spi_dev.cpp \
    main_touch_screen.cpp

