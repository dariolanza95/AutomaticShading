QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle
CONFIG += object_with_source
# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += Graphics/openglvisualizer.cpp \
    Graphics/mesh.cpp \
    Graphics/IndexBuffer.cpp \
    Graphics/Shader.cpp \
    Graphics/Grid2D.cpp \
    Graphics/VertexBuffer.cpp \
    Graphics/GLWrapper.cpp \
    Graphics/Exception.cpp \
    Graphics/Camera.cpp \
    Graphics/PerlinNoise.cpp \
    Resources/aclassifier.cpp \
    Resources/featuresfinder.cpp \
    Resources/flowclassifier.cpp \
    Resources/main.cpp \
    Resources/simulationdata.cpp \
    Resources/riverclassifiertester.cpp \
    Resources/screeclassifier.cpp \
    Resources/riverclassifier.cpp \
    Resources/ribwriter.cpp \
    Resources/VertexEditTag.cpp \
    Resources/FieldThreeDWriter.cpp \
    Resources/pointcloudwritertester.cpp \
    Resources/materialclassifier.cpp \
    Resources/LICMap.cpp \
    External/FastNoise/FastNoise.cpp \
    Graphics/exceptionclass.cpp \
    Resources/shaderparameter.cpp \
    Resources/ShaderWrapper.cpp \
    Resources/flowshader.cpp \
    Resources/materialshader.cpp \
    Resources/utils.cpp \
    Resources/ribconstant.cpp \
    Resources/RibShaderNode.cpp \
    Resources/ribmasknode.cpp \
    Resources/ribmixnode.cpp \
    Resources/ribaddnode.cpp \
    Resources/bxdfnode.cpp \
    Resources/riblight.cpp \
    Resources/displnode.cpp \
    Resources/sedimentationclassifier.cpp \
    Resources/sedimentationshader.cpp \
    Resources/RibNode.cpp \
    Resources/AShader.cpp \
    Resources/airpressureclassifier.cpp \
    Resources/airpressureshader.cpp \
    Resources/sedimentationdata.cpp \
    Resources/outputwriter.cpp \
    Resources/apointcloudwriter.cpp \
    Resources/RIBPointCloudWriter.cpp \
    Resources/pointcloudwriter.cpp \
    Resources/inputfilereader.cpp



OpenMeshDirectory = $$(OPENMESHTREE)

INCLUDEPATH +=$${OpenMeshDirectory}/src

win32:CONFIG(release, debug|release):    LIBS += -L$${OpenMeshDirectory}/build/Build/lib/ -lOpenMeshCore.so
else:win32:CONFIG(debug, debug|release): LIBS += -L$${OpenMeshDirectory}/build/Build/lib/ -lOpenMeshCore.so
else:unix: LIBS += -L$${OpenMeshDirectory}/build/Build/lib/ -lOpenMeshCore

INCLUDEPATH += $$OPENMESHTREE/build/Build
DEPENDPATH +=  $$OPENMESHTREE/build/Build

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS +=     $${OpenMeshDirectory}/build/Build/lib/libOpenMeshCore.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS +=  $${OpenMeshDirectory}/build/Build/lib/libOpenMeshCore.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $${OpenMeshDirectory}/build/Build/lib/OpenMeshCore.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS +=   $${OpenMeshDirectory}/build/Build/lib/OpenMeshCore.lib
else:unix: PRE_TARGETDEPS += $${OpenMeshDirectory}/build/Build/lib/libOpenMeshCore.a

HEADERS += Graphics/openglvisualizer.h \
    Graphics/IndexBuffer.h \
    Graphics/Shader.h \
    Graphics/VertexBuffer.h \
    Graphics/Mesh.h \
    Graphics/GLWrapper.h \
    Graphics/Grid2D.h \
    Graphics/Exception.h \
    Graphics/Camera.h \
    Graphics/PerlinNoise.h \
    Graphics/platform_includes.h \
    Resources/aclassifier.h \
    Resources/featuresfinder.h \
    Resources/flowclassifier.h \
    Resources/dataloader.h \
    Resources/ribwriter.h \
    Resources/riverclassifier.h \
    Resources/riverclassifiertester.h \
    Resources/screeclassifier.h \
    Resources/simulationdata.h \
    Resources/VertexEditTag.h \
    Resources/FieldThreeDWriter.h \
    Resources/PointCloudWriter.h \
    Resources/pointcloudwritertester.h \
    Resources/materialclassifier.h \
    Resources/LICMap.h \
    External/FastNoise/FastNoise.h \
    Resources/subdividerandinterpolator.h \
    Resources/subdividerandinterpolator_impl.h \
    Graphics/exceptionclass.h \
    Resources/shaderparameter.h \
    Resources/ShaderWrapper.h \
    Resources/flowshader.h \
    Resources/materialshader.h \
    Resources/utils.h \
    Resources/ribconstant.h \
    Resources/ribshadernode.h \
    Resources/ribmasknode.h \
    Resources/ribmixnode.h \
    Resources/ribaddnode.h \
    Resources/bxdfnode.h \
    Resources/riblight.h \
    Resources/displnode.h \
    Resources/mydefwrapper.h \
    Resources/myrules.h \
    Resources/sedimentationclassifier.h \
    Resources/sedimentationshader.h \
    Resources/RibNode.h \
    Resources/AShader.h \
    Resources/airpressureclassifier.h \
    Resources/airpressureshader.h \
    Resources/sedimentationdata.h \
    External/tps/gauss-elim.h \
    External/tps/linalg3d.h \
    External/tps/ludecomposition.h \
    Resources/outputwriter.h \
    Resources/apointcloudwriter.h \
    Resources/pointcloudwriter.h \
    Resources/inputfilereader.h

unix|win32: LIBS += -lglfw
    LIBS+=-lboost_system
    LIBS+=-lboost_filesystem
unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += glfw3

win32:CONFIG(release, debug|release): LIBS +=    -L/usr/lib/x86_64-linux-gnu/release/ -lglfw
else:win32:CONFIG(debug, debug|release): LIBS += -L/usr/lib/x86_64-linux-gnu/debug/ -lglfw
else:unix: LIBS += -L/usr/lib/x86_64-linux-gnu/ -lglfw

INCLUDEPATH += /usr/lib/x86_64-linux-gnu
DEPENDPATH  += /usr/lib/x86_64-linux-gnu

win32:CONFIG(release, debug|release): LIBS +=    -L/usr/lib/x86_64-linux-gnu/release/ -lGLEW
else:win32:CONFIG(debug, debug|release): LIBS += -L/usr/lib/x86_64-linux-gnu/debug/ -lGLEW
else:unix: LIBS += -L/usr/lib/x86_64-linux-gnu/ -lGLEW

INCLUDEPATH += /usr/lib/x86_64-linux-gnu
DEPENDPATH +=  /usr/lib/x86_64-linux-gnu

win32:CONFIG(release, debug|release):    LIBS += -L/usr/lib/x86_64-linux-gnu/release/ -lGL
else:win32:CONFIG(debug, debug|release): LIBS += -L/usr/lib/x86_64-linux-gnu/debug/ -lGL
else:unix: LIBS += -L/usr/lib/x86_64-linux-gnu/ -lGL

INCLUDEPATH += /usr/lib/x86_64-linux-gnu
INCLUDEPATH += /opt/pixar/RenderManProServer-23.1/include/
INCLUDEPATH += /opt/pixar/RenderManProServer-23.1/lib/
DEPENDPATH +=  /usr/lib/x86_64-linux-gnu

DISTFILES += \
    Resources/lambert_f.glsl \
    Resources/lambert_v.glsl

unix|win32: LIBS += -lField3D


INCLUDEPATH += "/usr/lib/x86_64-linux-gnu"
LIBS += -L"/usr/include/hdf5/serial"


unix|win32: LIBS += -lhdf5_hl_cpp

unix: PKGCONFIG += hdf5

unix|win32: LIBS += -lprman

win32:CONFIG(release, debug|release): LIBS +=    -L/opt/pixar/RenderManProServer-23.1/lib/release/ -lprman
else:win32:CONFIG(debug, debug|release): LIBS += -L/opt/pixar/RenderManProServer-23.1/lib/debug/ -lprman
else:unix: LIBS += -L/opt/pixar/RenderManProServer-23.1/lib/ -lprman

INCLUDEPATH += /opt/pixar/RenderManProServer-23.1/include
DEPENDPATH +=  /opt/pixar/RenderManProServer-23.1/include

QMAKE_CFLAGS_RELEASE += -fopenmp
QMAKE_CFLAGS_DEBUG += -fopenmp
    QMAKE_CXXFLAGS += -fopenmp
    QMAKE_LFLAGS += -fopenmp



unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += pcl_kdtree-1.8


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../mathtoolbox/mathtoolbox/build/release/ -lmathtoolbox
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../mathtoolbox/mathtoolbox/build/debug/ -lmathtoolbox
#else:unix: LIBS += -L$$PWD/../../../../mathtoolbox/mathtoolbox/build/ -lmathtoolbox
else:unix: LIBS += -L$$PWD/External/MathtoolBox/ -lmathtoolbox


INCLUDEPATH += $$PWD/External/MathtoolBox/
DEPENDPATH +=  $$PWD/External/MathtoolBox/


#INCLUDEPATH += $$PWD/../../../../mathtoolbox/mathtoolbox/build
#DEPENDPATH += $$PWD/../../../../mathtoolbox/mathtoolbox/build

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../../mathtoolbox/mathtoolbox/build/release/libmathtoolbox.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../../mathtoolbox/mathtoolbox/build/debug/libmathtoolbox.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../../mathtoolbox/mathtoolbox/build/release/mathtoolbox.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../../mathtoolbox/mathtoolbox/build/debug/mathtoolbox.lib
#else:unix: PRE_TARGETDEPS += $$PWD/../../../../mathtoolbox/mathtoolbox/build/libmathtoolbox.a
else:unix: PRE_TARGETDEPS += $$PWD/External/MathtoolBox/libmathtoolbox.a

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../OpenMesh-8.0/build/Build/lib/release/ -lOpenMeshCore
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../OpenMesh-8.0/build/Build/lib/debug/ -lOpenMeshCore
else:unix: LIBS += -L$$PWD/../../../../OpenMesh-8.0/build/Build/lib/ -lOpenMeshCore

INCLUDEPATH += $$PWD/../../../../OpenMesh-8.0/build/Build
DEPENDPATH += $$PWD/../../../../OpenMesh-8.0/build/Build

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../../OpenMesh-8.0/build/Build/lib/release/libOpenMeshCore.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../../OpenMesh-8.0/build/Build/lib/debug/libOpenMeshCore.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../../OpenMesh-8.0/build/Build/lib/release/OpenMeshCore.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../../OpenMesh-8.0/build/Build/lib/debug/OpenMeshCore.lib
else:unix: PRE_TARGETDEPS += $$PWD/../../../../OpenMesh-8.0/build/Build/lib/libOpenMeshCore.a
