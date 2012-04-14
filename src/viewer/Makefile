CC = gcc
CXX = g++
#CC = icc
#CXX = icpc

EXENAME = ../../bin/viewer
SRC = main.cpp
OBJ = $(SRC:%.cpp=%.o)
INCLUDEDIR = -I/usr/include/i386-linux-gnu `pkg-config opencv --cflags-only-I` `pkg-config gtk+-2.0 --cflags-only-I` -I/usr/include/flycapture -I/usr/include/triclops
LIBDIR = `pkg-config opencv --libs-only-L` `pkg-config gtk+-2.0 --libs-only-L` -L/usr/lib
LIB = `pkg-config opencv --libs-only-l` `pkg-config gtk+-2.0 --libs-only-l`
CFLAGS = $(INCLUDEDIR) -O2
LDFLAGS = $(LIBDIR) $(LIB)

$(EXENAME): $(OBJ)
	$(CC) -o $@ $(OBJ) $(CFLAGS) $(LDFLAGS)

.cpp.o:
	$(CXX) -o $@ $< -c $(CFLAGS)

clean:
	@rm -rf $(EXENAME) $(OBJ) *~ *.bak *.BAK

