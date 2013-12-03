#-------------------------------------------------
#
# Project created by QtCreator 2012-05-16T18:06:02
#
#-------------------------------------------------

QT       += core gui

TARGET = EpuckMonitor
TEMPLATE = app


SOURCES += main.cpp\
        monitor.cpp \
     /home/bachelor/git/tobiasjahn-lpzrobots-fork/real_robots/robots/epuck/SerialComm.cpp \
     /home/bachelor/git/tobiasjahn-lpzrobots-fork/real_robots/robots/epuck/epuckbluetooth.cpp \
        plotwidget.cpp

HEADERS  += monitor.h \
     /home/bachelor/git/tobiasjahn-lpzrobots-fork/real_robots/robots/epuck/SerialComm.h \
     /home/bachelor/git/tobiasjahn-lpzrobots-fork/real_robots/robots/epuck/epuckbluetooth.h \
        plotwidget.h

FORMS    += \
    monitor.ui



INCLUDEPATH +=  /home/bachelor/git/lpzrobots/selforg \
                /home/bachelor/LPZ/include/ \
                /home/bachelor/LPZ/lib/ \
                /home/bachelor/git/lpzrobots/selforg/utils \
                /home/bachelor/git/lpzrobots/ode_robots/osg \
                /home/bachelor/git/lpzrobots/ode_robots/utils \
                /home/bachelor/git/lpzrobots/opende/include \
                /home/bachelor/git/lpzrobots/ode_robots \
                /home/bachelor/git/tobiasjahn-lpzrobots-fork/real_robots/robots/epuck/



LIBS       += -lreadline \
         -lncurses \
          ~/LPZ/lib/libselforg.a \
          ~/LPZ/lib/libselforg_dbg.a \
          ~/LPZ/lib/libselforg_opt.a
