CC = g++
CFLAGS = -std=c++14 -Wno-deprecated -fPIC -O3
INCLUDES = -I"$(RMANTREE)/include" -I"../include"
LD = $(CC) -shared
LINKS = -L/usr/lib/x86_64-linux-gnu 

SOURCES=\
	MaskReader.cpp \
	FlowShader.cpp \
	PxrAdd.cpp\
	MaterialShader.cpp
	

OBJECTS = $(SOURCES:.cpp=.o)
DSO = $(SOURCES:.cpp=.so)
.SUFFIXES: .cpp .o .so

all:  $(OBJECTS) $(DSO)

.o.so: $(OBJECTS)
	$(LD) -o $@ $< $(LINKS)

.cpp.o:
	$(CC) $(INCLUDES) $(CFLAGS) -o $@ -c $<

.PHONY: clean

clean:
	find . -name "*.so" -delete -o -name "*.o" -delete

