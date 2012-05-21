#CC = gcc
#CXX = g++
CC = icc
CXX = icpc

EXENAME = ../../bin/cameraunit
SRC = main.cpp CalculationProcessLogger.cpp
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
	@rm -rf $(EXENAME) $(OBJ) *~ *.bak *.BAK

depend:
	makedepend -- $(CFLAGS) -- $(SRC)
# DO NOT DELETE

main.o: /usr/include/flycapture/FlyCapture2.h
main.o: /usr/include/flycapture/FlyCapture2Platform.h
main.o: /usr/include/flycapture/FlyCapture2Defs.h /usr/include/memory.h
main.o: /usr/include/features.h /usr/include/i386-linux-gnu/bits/predefs.h
main.o: /usr/include/i386-linux-gnu/sys/cdefs.h
main.o: /usr/include/i386-linux-gnu/bits/wordsize.h
main.o: /usr/include/i386-linux-gnu/gnu/stubs.h
main.o: /usr/include/i386-linux-gnu/gnu/stubs-32.h /usr/include/string.h
main.o: /usr/include/xlocale.h /usr/include/flycapture/Error.h
main.o: /usr/include/stdio.h /usr/include/i386-linux-gnu/bits/types.h
main.o: /usr/include/i386-linux-gnu/bits/typesizes.h /usr/include/libio.h
main.o: /usr/include/_G_config.h /usr/include/wchar.h
main.o: /usr/include/i386-linux-gnu/bits/stdio_lim.h
main.o: /usr/include/i386-linux-gnu/bits/sys_errlist.h
main.o: /usr/include/flycapture/BusManager.h /usr/include/flycapture/Camera.h
main.o: /usr/include/flycapture/CameraBase.h
main.o: /usr/include/flycapture/GigECamera.h /usr/include/flycapture/Image.h
main.o: /usr/include/flycapture/Utilities.h
main.o: /usr/include/flycapture/AVIRecorder.h
main.o: /usr/include/flycapture/TopologyNode.h
main.o: /usr/include/flycapture/ImageStatistics.h
main.o: /usr/include/triclops/triclops.h /usr/include/triclops/triclops3d.h
main.o: /usr/include/triclops/triclopsbuffer.h
main.o: /usr/include/triclops/triclopscontext.h
main.o: /usr/include/triclops/triclopsvalidation.h
main.o: /usr/include/triclops/triclopsrectify.h
main.o: /usr/include/triclops/triclopsstereo.h
main.o: /usr/include/triclops/triclopsimageio.h
main.o: /usr/local/include/opencv/cv.h
main.o: /usr/local/include/opencv2/core/core_c.h
main.o: /usr/local/include/opencv2/core/types_c.h /usr/include/assert.h
main.o: /usr/include/stdlib.h /usr/include/i386-linux-gnu/bits/waitflags.h
main.o: /usr/include/i386-linux-gnu/bits/waitstatus.h /usr/include/endian.h
main.o: /usr/include/i386-linux-gnu/bits/endian.h
main.o: /usr/include/i386-linux-gnu/bits/byteswap.h
main.o: /usr/include/i386-linux-gnu/sys/types.h /usr/include/time.h
main.o: /usr/include/i386-linux-gnu/sys/select.h
main.o: /usr/include/i386-linux-gnu/bits/select.h
main.o: /usr/include/i386-linux-gnu/bits/sigset.h
main.o: /usr/include/i386-linux-gnu/bits/time.h
main.o: /usr/include/i386-linux-gnu/sys/sysmacros.h
main.o: /usr/include/i386-linux-gnu/bits/pthreadtypes.h /usr/include/alloca.h
main.o: /usr/include/stdint.h /usr/include/i386-linux-gnu/bits/wchar.h
main.o: /usr/include/math.h /usr/include/i386-linux-gnu/bits/huge_val.h
main.o: /usr/include/i386-linux-gnu/bits/huge_valf.h
main.o: /usr/include/i386-linux-gnu/bits/huge_vall.h
main.o: /usr/include/i386-linux-gnu/bits/inf.h
main.o: /usr/include/i386-linux-gnu/bits/nan.h
main.o: /usr/include/i386-linux-gnu/bits/mathdef.h
main.o: /usr/include/i386-linux-gnu/bits/mathcalls.h
main.o: /usr/local/include/opencv2/core/core.hpp
main.o: /usr/local/include/opencv2/core/version.hpp
main.o: /usr/local/include/opencv2/core/operations.hpp /usr/include/limits.h
main.o: /usr/include/i386-linux-gnu/bits/posix1_lim.h
main.o: /usr/include/i386-linux-gnu/bits/local_lim.h
main.o: /usr/include/linux/limits.h
main.o: /usr/include/i386-linux-gnu/bits/posix2_lim.h
main.o: /usr/local/include/opencv2/core/mat.hpp
main.o: /usr/local/include/opencv2/imgproc/imgproc_c.h
main.o: /usr/local/include/opencv2/imgproc/types_c.h
main.o: /usr/local/include/opencv2/imgproc/imgproc.hpp
main.o: /usr/local/include/opencv2/video/tracking.hpp
main.o: /usr/local/include/opencv2/features2d/features2d.hpp
main.o: /usr/local/include/opencv2/flann/miniflann.hpp
main.o: /usr/local/include/opencv2/flann/flann.hpp
main.o: /usr/local/include/opencv2/calib3d/calib3d.hpp
main.o: /usr/local/include/opencv2/objdetect/objdetect.hpp
main.o: /usr/local/include/opencv2/legacy/compat.hpp
main.o: /usr/local/include/opencv/highgui.h
main.o: /usr/local/include/opencv2/highgui/highgui_c.h
main.o: /usr/local/include/opencv2/highgui/highgui.hpp /usr/include/zlib.h
main.o: /usr/include/zconf.h /usr/include/zlibdefs.h /usr/include/unistd.h
main.o: /usr/include/i386-linux-gnu/bits/posix_opt.h
main.o: /usr/include/i386-linux-gnu/bits/environments.h
main.o: /usr/include/i386-linux-gnu/bits/confname.h /usr/include/getopt.h
main.o: ../humantracking.h CalculationProcessLogger.h
CalculationProcessLogger.o: ../humantracking.h /usr/local/include/opencv/cv.h
CalculationProcessLogger.o: /usr/local/include/opencv2/core/core_c.h
CalculationProcessLogger.o: /usr/local/include/opencv2/core/types_c.h
CalculationProcessLogger.o: /usr/include/assert.h /usr/include/features.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/predefs.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/sys/cdefs.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/wordsize.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/gnu/stubs.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/gnu/stubs-32.h
CalculationProcessLogger.o: /usr/include/stdlib.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/waitflags.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/waitstatus.h
CalculationProcessLogger.o: /usr/include/endian.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/endian.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/byteswap.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/sys/types.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/types.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/typesizes.h
CalculationProcessLogger.o: /usr/include/time.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/sys/select.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/select.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/sigset.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/time.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/sys/sysmacros.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/pthreadtypes.h
CalculationProcessLogger.o: /usr/include/alloca.h /usr/include/string.h
CalculationProcessLogger.o: /usr/include/xlocale.h /usr/include/stdint.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/wchar.h
CalculationProcessLogger.o: /usr/include/math.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/huge_val.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/huge_valf.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/huge_vall.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/inf.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/nan.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/mathdef.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/mathcalls.h
CalculationProcessLogger.o: /usr/local/include/opencv2/core/core.hpp
CalculationProcessLogger.o: /usr/local/include/opencv2/core/version.hpp
CalculationProcessLogger.o: /usr/local/include/opencv2/core/operations.hpp
CalculationProcessLogger.o: /usr/include/limits.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/posix1_lim.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/local_lim.h
CalculationProcessLogger.o: /usr/include/linux/limits.h
CalculationProcessLogger.o: /usr/include/i386-linux-gnu/bits/posix2_lim.h
CalculationProcessLogger.o: /usr/local/include/opencv2/core/mat.hpp
CalculationProcessLogger.o: /usr/local/include/opencv2/imgproc/imgproc_c.h
CalculationProcessLogger.o: /usr/local/include/opencv2/imgproc/types_c.h
CalculationProcessLogger.o: /usr/local/include/opencv2/imgproc/imgproc.hpp
CalculationProcessLogger.o: /usr/local/include/opencv2/video/tracking.hpp
CalculationProcessLogger.o: /usr/local/include/opencv2/features2d/features2d.hpp
CalculationProcessLogger.o: /usr/local/include/opencv2/flann/miniflann.hpp
CalculationProcessLogger.o: /usr/local/include/opencv2/flann/flann.hpp
CalculationProcessLogger.o: /usr/local/include/opencv2/calib3d/calib3d.hpp
CalculationProcessLogger.o: /usr/local/include/opencv2/objdetect/objdetect.hpp
CalculationProcessLogger.o: /usr/local/include/opencv2/legacy/compat.hpp
CalculationProcessLogger.o: CalculationProcessLogger.h
