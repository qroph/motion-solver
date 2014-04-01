CONFIG += console
CONFIG -= app_bundle

QMAKE_CXXFLAGS += -Wall
QMAKE_CXXFLAGS += -std=c++11

HEADERS += \
    Solver.h \
    Roadmap.h \
    Planner.h \
    Object.h \
    NearestNeighbor.h \
    LocalPlanner.h \
    Generator.h \
    Distance.h \
    DisjointSets.h \
    Configuration.h \
    CollisionDetection.h \
    Common.h

SOURCES += \
    main.cpp \
    Solver.cpp \
    Roadmap.cpp \
    Planner.cpp \
    NearestNeighbor.cpp \
    Common.cpp \
    LocalPlanner.cpp \
    Generator.cpp \
    Distance.cpp \
    CollisionDetection.cpp

LIBS += -L$$PWD/coldet/src/ -lcoldet
INCLUDEPATH += $$PWD/coldet/src
