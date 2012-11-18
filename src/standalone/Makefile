#CC = gcc
#CXX = g++
CC = icc
CXX = icpc

EXENAME = ../../bin/standalone
SRC = main.cpp StereoVideo.cpp track.cpp ClusterTrajectories.cpp ExtendTrajectories.cpp ExtractLUM.cpp GestureRecognition.cpp MakeLUMSlice.cpp PlotCalculationProcess.cpp RenovateTrajectory.cpp ResultRenderer2.cpp Trajectory.cpp
OBJ = $(SRC:%.cpp=%.o)
INCLUDEDIR = -I/usr/include/i386-linux-gnu `pkg-config opencv --cflags-only-I` -I/usr/include/triclops -I/usr/include/flycapture
ifeq ($(LBITS),32)
    LIBDIR = `pkg-config opencv --libs-only-L` -L/usr/lib -L/opt/intel/Compiler/11.1/064/ipp/ia32/sharedlib
    LIB = `pkg-config opencv --libs-only-l` -lz -lippcore -lippi -lipps -lippm -ltriclops -lpnmutils
else
    LIBDIR = `pkg-config opencv --libs-only-L` -L/usr/lib -L/opt/intel/Compiler/11.0/081/ipp/em64t/sharedlib
    LIB = `pkg-config opencv --libs-only-l` -lz -lippcoreem64t -lippiem64t -lippsem64t -lippmem64t -ltriclops -lpnmutils
endif
CFLAGS = $(INCLUDEDIR) -O2
LDFLAGS = $(LIBDIR) $(LIB)

$(EXENAME): $(OBJ)
	$(CC) -o $@ $(OBJ) $(CFLAGS) $(LDFLAGS)

.cpp.o:
	$(CXX) -o $@ $< -c $(CFLAGS)

clean:
	@rm -rf $(EXENAME) $(OBJ) *~ *.bak *.BAK

depend:
	makedepend -- $(CFLAGS) -- $(SRC)# DO NOT DELETE