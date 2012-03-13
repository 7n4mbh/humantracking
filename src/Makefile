#CC = gcc
#CXX = g++
CC = icc
CXX = icpc

EXENAME = ../bin/cameraunit
SRC = cameraunit/main.cpp
OBJ = $(SRC:%.cpp=%.o)
INCLUDEDIR = -I/usr/include/i386-linux-gnu `pkg-config opencv --cflags-only-I` -I/usr/include/flycapture -I/usr/include/triclops
LIBDIR = `pkg-config opencv --libs-only-L` -L/usr/lib
LIB = `pkg-config opencv --libs-only-l` -lflycapture -ltriclops -lpnmutils
CFLAGS = $(INCLUDEDIR) -O2
LDFLAGS = $(LIBDIR) $(LIB)

$(EXENAME): $(OBJ)
	$(CC) -o $@ $(OBJ) $(CFLAGS) $(LDFLAGS)

.cpp.o:
	$(CXX) -o $@ $< -c $(CFLAGS)

clean:
	@rm -rf $(EXENAME) $(OBJ) cameraunit/*~ cameraunit/*.bak cameraunit/*.BAK

