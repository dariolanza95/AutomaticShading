#############################################################################
# Makefile for building: TerrainFluid
# Generated by qmake (2.01a) (Qt 4.8.7) on: ven feb 14 12:53:57 2020
# Project:  ../terrain-erosion/TerrainFluid.pro
# Template: app
# Command: /usr/lib/x86_64-linux-gnu/qt4/bin/qmake -spec /usr/share/qt4/mkspecs/linux-g++-64 -o Makefile ../terrain-erosion/TerrainFluid.pro
#############################################################################

####### Compiler, tools and options

CC            = gcc
CXX           = g++
DEFINES       = 
CFLAGS        = -m64 -pipe -O2 -Wall -W $(DEFINES)
CXXFLAGS      = -m64 -pipe -std=c++11 -fopenmp -O2 -Wall -W $(DEFINES)
INCPATH       = -I/usr/share/qt4/mkspecs/linux-g++-64 -I../terrain-erosion -I../terrain-erosion/external -I/lib/x86_64-linux-gnu -I/usr/local/include -I../terrain-erosion -I.
LINK          = g++
LFLAGS        = -m64 -Wl,--start-group -fopenmp -Wl,-O1
LIBS          = $(SUBLIBS)   -lboost_system -lboost_filesystem -lGLEW -lGL -L/lib/x86_64-linux-gnu -ldl -lX11 -L/usr/local/lib -lglfw3 
AR            = ar cqs
RANLIB        = 
QMAKE         = /usr/lib/x86_64-linux-gnu/qt4/bin/qmake
TAR           = tar -cf
COMPRESS      = gzip -9f
COPY          = cp -f
SED           = sed
COPY_FILE     = $(COPY)
COPY_DIR      = $(COPY) -r
STRIP         = strip
INSTALL_FILE  = install -m 644 -p
INSTALL_DIR   = $(COPY_DIR)
INSTALL_PROGRAM = install -m 755 -p
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p

####### Output directory

OBJECTS_DIR   = ./

####### Files

SOURCES       = ../terrain-erosion/Camera.cpp \
		../terrain-erosion/main.cpp \
		../terrain-erosion/TerrainFluidSimulation.cpp \
		../terrain-erosion/Graphics/Shader.cpp \
		../terrain-erosion/Graphics/GLWrapper.cpp \
		../terrain-erosion/Simulation/FluidSimulation.cpp \
		../terrain-erosion/Graphics/IndexBuffer.cpp \
		../terrain-erosion/Math/PerlinNoise.cpp 
OBJECTS       = Camera.o \
		main.o \
		TerrainFluidSimulation.o \
		Shader.o \
		GLWrapper.o \
		FluidSimulation.o \
		IndexBuffer.o \
		PerlinNoise.o
DIST          = ../terrain-erosion/Resources/lambert_f_arrow.glsl \
		../terrain-erosion/Resources/lambert_v_arrow.glsl \
		/usr/share/qt4/mkspecs/common/unix.conf \
		/usr/share/qt4/mkspecs/common/linux.conf \
		/usr/share/qt4/mkspecs/common/gcc-base.conf \
		/usr/share/qt4/mkspecs/common/gcc-base-unix.conf \
		/usr/share/qt4/mkspecs/common/g++-base.conf \
		/usr/share/qt4/mkspecs/common/g++-unix.conf \
		/usr/share/qt4/mkspecs/qconfig.pri \
		/usr/share/qt4/mkspecs/features/qt_functions.prf \
		/usr/share/qt4/mkspecs/features/qt_config.prf \
		/usr/share/qt4/mkspecs/features/exclusive_builds.prf \
		/usr/share/qt4/mkspecs/features/default_pre.prf \
		/usr/share/qt4/mkspecs/features/release.prf \
		/usr/share/qt4/mkspecs/features/default_post.prf \
		/usr/share/qt4/mkspecs/features/link_pkgconfig.prf \
		/usr/share/qt4/mkspecs/features/shared.prf \
		/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf \
		/usr/share/qt4/mkspecs/features/warn_on.prf \
		/usr/share/qt4/mkspecs/features/resources.prf \
		/usr/share/qt4/mkspecs/features/uic.prf \
		/usr/share/qt4/mkspecs/features/yacc.prf \
		/usr/share/qt4/mkspecs/features/lex.prf \
		/usr/share/qt4/mkspecs/features/include_source_dir.prf \
		../terrain-erosion/TerrainFluid.pro
QMAKE_TARGET  = TerrainFluid
DESTDIR       = 
TARGET        = TerrainFluid

first: all
####### Implicit rules

.SUFFIXES: .o .c .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o "$@" "$<"

####### Build rules

all: Makefile $(TARGET)

$(TARGET):  $(OBJECTS)  
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)

Makefile: ../terrain-erosion/TerrainFluid.pro  /usr/share/qt4/mkspecs/linux-g++-64/qmake.conf /usr/share/qt4/mkspecs/common/unix.conf \
		/usr/share/qt4/mkspecs/common/linux.conf \
		/usr/share/qt4/mkspecs/common/gcc-base.conf \
		/usr/share/qt4/mkspecs/common/gcc-base-unix.conf \
		/usr/share/qt4/mkspecs/common/g++-base.conf \
		/usr/share/qt4/mkspecs/common/g++-unix.conf \
		/usr/share/qt4/mkspecs/qconfig.pri \
		/usr/share/qt4/mkspecs/features/qt_functions.prf \
		/usr/share/qt4/mkspecs/features/qt_config.prf \
		/usr/share/qt4/mkspecs/features/exclusive_builds.prf \
		/usr/share/qt4/mkspecs/features/default_pre.prf \
		/usr/share/qt4/mkspecs/features/release.prf \
		/usr/share/qt4/mkspecs/features/default_post.prf \
		/usr/share/qt4/mkspecs/features/link_pkgconfig.prf \
		/usr/share/qt4/mkspecs/features/shared.prf \
		/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf \
		/usr/share/qt4/mkspecs/features/warn_on.prf \
		/usr/share/qt4/mkspecs/features/resources.prf \
		/usr/share/qt4/mkspecs/features/uic.prf \
		/usr/share/qt4/mkspecs/features/yacc.prf \
		/usr/share/qt4/mkspecs/features/lex.prf \
		/usr/share/qt4/mkspecs/features/include_source_dir.prf
	$(QMAKE) -spec /usr/share/qt4/mkspecs/linux-g++-64 -o Makefile ../terrain-erosion/TerrainFluid.pro
/usr/share/qt4/mkspecs/common/unix.conf:
/usr/share/qt4/mkspecs/common/linux.conf:
/usr/share/qt4/mkspecs/common/gcc-base.conf:
/usr/share/qt4/mkspecs/common/gcc-base-unix.conf:
/usr/share/qt4/mkspecs/common/g++-base.conf:
/usr/share/qt4/mkspecs/common/g++-unix.conf:
/usr/share/qt4/mkspecs/qconfig.pri:
/usr/share/qt4/mkspecs/features/qt_functions.prf:
/usr/share/qt4/mkspecs/features/qt_config.prf:
/usr/share/qt4/mkspecs/features/exclusive_builds.prf:
/usr/share/qt4/mkspecs/features/default_pre.prf:
/usr/share/qt4/mkspecs/features/release.prf:
/usr/share/qt4/mkspecs/features/default_post.prf:
/usr/share/qt4/mkspecs/features/link_pkgconfig.prf:
/usr/share/qt4/mkspecs/features/shared.prf:
/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf:
/usr/share/qt4/mkspecs/features/warn_on.prf:
/usr/share/qt4/mkspecs/features/resources.prf:
/usr/share/qt4/mkspecs/features/uic.prf:
/usr/share/qt4/mkspecs/features/yacc.prf:
/usr/share/qt4/mkspecs/features/lex.prf:
/usr/share/qt4/mkspecs/features/include_source_dir.prf:
qmake:  FORCE
	@$(QMAKE) -spec /usr/share/qt4/mkspecs/linux-g++-64 -o Makefile ../terrain-erosion/TerrainFluid.pro

dist: 
	@$(CHK_DIR_EXISTS) .tmp/TerrainFluid1.0.0 || $(MKDIR) .tmp/TerrainFluid1.0.0 
	$(COPY_FILE) --parents $(SOURCES) $(DIST) .tmp/TerrainFluid1.0.0/ && (cd `dirname .tmp/TerrainFluid1.0.0` && $(TAR) TerrainFluid1.0.0.tar TerrainFluid1.0.0 && $(COMPRESS) TerrainFluid1.0.0.tar) && $(MOVE) `dirname .tmp/TerrainFluid1.0.0`/TerrainFluid1.0.0.tar.gz . && $(DEL_FILE) -r .tmp/TerrainFluid1.0.0


clean:compiler_clean 
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core


####### Sub-libraries

distclean: clean
	-$(DEL_FILE) $(TARGET) 
	-$(DEL_FILE) Makefile




check: first

compiler_rcc_make_all:
compiler_rcc_clean:
compiler_uic_make_all:
compiler_uic_clean:
compiler_image_collection_make_all: qmake_image_collection.cpp
compiler_image_collection_clean:
	-$(DEL_FILE) qmake_image_collection.cpp
compiler_yacc_decl_make_all:
compiler_yacc_decl_clean:
compiler_yacc_impl_make_all:
compiler_yacc_impl_clean:
compiler_lex_make_all:
compiler_lex_clean:
compiler_clean: 

####### Compile

Camera.o: ../terrain-erosion/Camera.cpp ../terrain-erosion/Camera.h \
		../terrain-erosion/platform_includes.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Camera.o ../terrain-erosion/Camera.cpp

main.o: ../terrain-erosion/main.cpp ../terrain-erosion/platform_includes.h \
		../terrain-erosion/TerrainFluidSimulation.h \
		../terrain-erosion/Simulation/FluidSimulation.h \
		../terrain-erosion/Grid2D.h \
		../terrain-erosion/Math/PerlinNoise.h \
		../terrain-erosion/SimulationState.h \
		../terrain-erosion/Grid3D.h \
		../terrain-erosion/Graphics/Shader.h \
		../terrain-erosion/Graphics/GLWrapper.h \
		../terrain-erosion/Exception.h \
		../terrain-erosion/Graphics/Texture2D.h \
		../terrain-erosion/Graphics/VertexBuffer.h \
		../terrain-erosion/Graphics/IndexBuffer.h \
		../terrain-erosion/Camera.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o main.o ../terrain-erosion/main.cpp

TerrainFluidSimulation.o: ../terrain-erosion/TerrainFluidSimulation.cpp ../terrain-erosion/TerrainFluidSimulation.h \
		../terrain-erosion/Simulation/FluidSimulation.h \
		../terrain-erosion/platform_includes.h \
		../terrain-erosion/Grid2D.h \
		../terrain-erosion/Math/PerlinNoise.h \
		../terrain-erosion/SimulationState.h \
		../terrain-erosion/Grid3D.h \
		../terrain-erosion/Graphics/Shader.h \
		../terrain-erosion/Graphics/GLWrapper.h \
		../terrain-erosion/Exception.h \
		../terrain-erosion/Graphics/Texture2D.h \
		../terrain-erosion/Graphics/VertexBuffer.h \
		../terrain-erosion/Graphics/IndexBuffer.h \
		../terrain-erosion/Camera.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o TerrainFluidSimulation.o ../terrain-erosion/TerrainFluidSimulation.cpp

Shader.o: ../terrain-erosion/Graphics/Shader.cpp ../terrain-erosion/Graphics/Shader.h \
		../terrain-erosion/platform_includes.h \
		../terrain-erosion/Graphics/GLWrapper.h \
		../terrain-erosion/Exception.h \
		../terrain-erosion/Graphics/Texture2D.h \
		../terrain-erosion/Grid2D.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Shader.o ../terrain-erosion/Graphics/Shader.cpp

GLWrapper.o: ../terrain-erosion/Graphics/GLWrapper.cpp ../terrain-erosion/Graphics/GLWrapper.h \
		../terrain-erosion/platform_includes.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o GLWrapper.o ../terrain-erosion/Graphics/GLWrapper.cpp

FluidSimulation.o: ../terrain-erosion/Simulation/FluidSimulation.cpp ../terrain-erosion/Simulation/FluidSimulation.h \
		../terrain-erosion/platform_includes.h \
		../terrain-erosion/Grid2D.h \
		../terrain-erosion/Math/PerlinNoise.h \
		../terrain-erosion/SimulationState.h \
		../terrain-erosion/Grid3D.h \
		../terrain-erosion/Math/MathUtil.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o FluidSimulation.o ../terrain-erosion/Simulation/FluidSimulation.cpp

IndexBuffer.o: ../terrain-erosion/Graphics/IndexBuffer.cpp ../terrain-erosion/Graphics/IndexBuffer.h \
		../terrain-erosion/platform_includes.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o IndexBuffer.o ../terrain-erosion/Graphics/IndexBuffer.cpp

PerlinNoise.o: ../terrain-erosion/Math/PerlinNoise.cpp ../terrain-erosion/Math/PerlinNoise.h \
		../terrain-erosion/platform_includes.h \
		../terrain-erosion/Math/MathUtil.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o PerlinNoise.o ../terrain-erosion/Math/PerlinNoise.cpp

####### Install

install:   FORCE

uninstall:   FORCE

FORCE:

