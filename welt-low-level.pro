QT += core network gui
QT += serialport # раскомментировать!!!

CONFIG += c++17 console
CONFIG -= app_bundle

SOURCES += \
        PS_MS5837/ms5837.cpp \
        hydroacoustic_peleng/hydroPeleng.cpp \
        vectorNav/vectorNav.cpp \
        VMA/vma_controller.cpp \
        cs_rov.cpp \
        kx_pult/configdata.cpp \
        kx_pult/kx_protocol.cpp \
        kx_pult/qkx_coeffs.cpp \
        kx_pult/qpiconfig.cpp \
        uWaveSingapore/hydroacoustics.cpp \
        uWaveSingapore/statemachine.cpp \
        uWaveSingapore/json_parser.cpp \
        main.cpp

INCLUDEPATH += /home/isla/rpi/sysroot/usr/include
INCLUDEPATH += /home/isla/rpi/sysroot/usr/local/include
LIBS += -L/home/isla/rpi/sysroot/usr/lib -lwiringPi
LIBS += -L/home/isla/rpi/qt5.15/lib -lQt5Gui -lQt5Network -lQt5SerialPort -lQt5Core
LIBS += -lGLESv2 -lpthread
LIBS += -L"/home/isla/rpi/sysroot/usr/lib/arm-linux-gnueabihf"
LIBS += -li2c


target.path = /home/projects/welt-low-level-SAUVC25/build-rpi$

INSTALLS += target

HEADERS += \
    PS_MS5837/ms5837.h \
    hydroacoustic_peleng/hydroPeleng.h \
    vectorNav/vectorNav.h \
    VMA/vma_controller.h \
    cs_rov.h \
    kx_pult/configdata.h \
    kx_pult/kx_protocol.h \
    kx_pult/qkx_coeffs.h \
    kx_pult/qpiconfig.h \
    protocol_bort_AUV/pc_protocol.h \
    protocol_bort_AUV/protocol.h \
    protocol_bort_AUV/udp_protocol.h \
    uWaveSingapore/hydroacoustics.h \
    uWaveSingapore/statemachine.h \
    uWaveSingapore/json_parser.h

