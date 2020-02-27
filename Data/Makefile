CC = g++
CFLAGS = -std=c++14 -Wno-deprecated -fPIC -O3
INCLUDES = -I"$(RMANTREE)/include" -I"../include"
LD = $(CC) -shared

LIBS          = $(SUBLIBS)  -L/usr/lib/x86_64-linux-gnu -L/home/pandora/OpenMesh-8.0/build/Build/lib/ -lOpenMeshCore -lboost_system -lboost_filesystem -L/home/pandora/thesis/AutomaticShading/AutomaticHeuristic/AutomaticHeuristic/../../../../../../usr/lib/x86_64-linux-gnu/ -lglfw -lGLEW -lGL -lField3D -L/usr/include/hdf5/serial -lhdf5_hl_cpp -L/usr/local/lib -lglfw3 -L/usr/lib/x86_64-linux-gnu/hdf5/serial -lhdf5 -lQtCore -lpthread 

SOURCES=\
	PxrWorleyD.cpp \
	

OBJECTS = $(SOURCES:.cpp=.o)
DSO = $(SOURCES:.cpp=.so)
.SUFFIXES: .cpp .o .so

all:  $(OBJECTS) $(DSO) $(LIBS)

.o.so: $(OBJECTS)
	$(LD) -o $@ $< 

.cpp.o:
	$(CC) $(INCLUDES) $(CFLAGS) -o $@ -c $<

.PHONY: clean

clean:
	find . -name "*.so" -delete -o -name "*.o" -delete

