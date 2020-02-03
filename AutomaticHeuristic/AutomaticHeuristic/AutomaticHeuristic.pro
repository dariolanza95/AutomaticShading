QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle
# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += *.cpp \
    defaultdataloader.cpp \
    screeclassifier.cpp \
    simulationdata.cpp \
    riverclassifier.cpp \
    riverclassifiertester.cpp \
    ribwriter.cpp \
    ribnode.cpp \
    Graphics/openglvisualizer.cpp \
    Graphics/mesh.cpp \
    Graphics/IndexBuffer.cpp \
    Graphics/Shader.cpp \
    Graphics/Grid2D.cpp \
    Graphics/VertexBuffer.cpp \
    Graphics/GLWrapper.cpp \
    Graphics/Exception.cpp \
    Graphics/Camera.cpp \
    Graphics/Camera.cpp \
    Graphics/Exception.cpp \
    Graphics/GLWrapper.cpp \
    Graphics/Grid2D.cpp \
    Graphics/IndexBuffer.cpp \
    Graphics/mesh.cpp \
    Graphics/openglvisualizer.cpp \
    Graphics/Shader.cpp \
    Graphics/VertexBuffer.cpp \
    Resources/defaultdataloader.cpp \
    defaultdataloader.cpp \
    main.cpp \
    ribnode.cpp \
    ribwriter.cpp \
    riverclassifier.cpp \
    riverclassifiertester.cpp \
    screeclassifier.cpp \
    simulationdata.cpp \
    Graphics/PerlinNoise.cpp \
    ShaderParameters.cpp \
    aclassifier.cpp



INCLUDEPATH +="/home/pandora/OpenMesh-8.0/src/"

win32:CONFIG(release, debug|release): LIBS += -L/home/pandora/OpenMesh-8.0/build/Build/lib/release/ -lOpenMeshCore
else:win32:CONFIG(debug, debug|release): LIBS += -L/home/pandora/OpenMesh-8.0/build/Build/lib/debug/ -lOpenMeshCore
else:unix: LIBS += -L/home/pandora/OpenMesh-8.0/build/Build/lib/ -lOpenMeshCore

INCLUDEPATH += /home/pandora/OpenMesh-8.0/build/Build
DEPENDPATH += /home/pandora/OpenMesh-8.0/build/Build

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += /home/pandora/OpenMesh-8.0/build/Build/lib/release/libOpenMeshCore.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += /home/pandora/OpenMesh-8.0/build/Build/lib/debug/libOpenMeshCore.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += /home/pandora/OpenMesh-8.0/build/Build/lib/release/OpenMeshCore.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += /home/pandora/OpenMesh-8.0/build/Build/lib/debug/OpenMeshCore.lib
else:unix: PRE_TARGETDEPS += /home/pandora/OpenMesh-8.0/build/Build/lib/libOpenMeshCore.a

HEADERS += *.h\
    defaultdataloader.h \
    dataloader.h \
    aclassifier.h \
    screeclassifier.h \
    simulationdata.h \
    riverclassifier.h \
    riverclassifiertester.h \
    ribwriter.h \
    ribnode.h \
    Graphics/openglvisualizer.h \
    Graphics/IndexBuffer.h \
    Graphics/Shader.h \
    Graphics/VertexBuffer.h \
    Graphics/Mesh.h \
    Graphics/GLWrapper.h \
    Graphics/Grid2D.h \
    Graphics/Exception.h \
    Graphics/Camera.h \
    Graphics/Camera.h \
    Graphics/Exception.h \
    Graphics/GLWrapper.h \
    Graphics/Grid2D.h \
    Graphics/IndexBuffer.h \
    Graphics/Mesh.h \
    Graphics/openglvisualizer.h \
    Graphics/Shader.h \
    Graphics/VertexBuffer.h \
    Resources/defaultdataloader.h \
    aclassifier.h \
    dataloader.h \
    defaultdataloader.h \
    ribnode.h \
    ribwriter.h \
    riverclassifier.h \
    riverclassifiertester.h \
    screeclassifier.h \
    simulationdata.h \
    Graphics/PerlinNoise.h \
    Graphics/platform_includes.h \
    ShaderParameters.h

unix|win32: LIBS += -lglfw
    LIBS+=-lboost_system
    LIBS+=-lboost_filesystem
unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += glfw3

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../usr/lib/x86_64-linux-gnu/release/ -lglfw
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../usr/lib/x86_64-linux-gnu/debug/ -lglfw
else:unix: LIBS += -L$$PWD/../../../../../../usr/lib/x86_64-linux-gnu/ -lglfw

INCLUDEPATH += $$PWD/../../../../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../../../../usr/lib/x86_64-linux-gnu

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../usr/lib/x86_64-linux-gnu/release/ -lGLEW
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../usr/lib/x86_64-linux-gnu/debug/ -lGLEW
else:unix: LIBS += -L$$PWD/../../../../../../usr/lib/x86_64-linux-gnu/ -lGLEW

INCLUDEPATH += $$PWD/../../../../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../../../../usr/lib/x86_64-linux-gnu

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../usr/lib/x86_64-linux-gnu/release/ -lGL
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../usr/lib/x86_64-linux-gnu/debug/ -lGL
else:unix: LIBS += -L$$PWD/../../../../../../usr/lib/x86_64-linux-gnu/ -lGL

INCLUDEPATH += $$PWD/../../../../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../../../../usr/lib/x86_64-linux-gnu

DISTFILES += \
    Resources/lambert_f.glsl \
    Resources/lambert_v.glsl
