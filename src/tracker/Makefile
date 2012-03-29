#CC = gcc
#CXX = g++
CC = icc
CXX = icpc

EXENAME = ../../bin/tracker
SRC = main.cpp track.cpp CameraUnit.cpp ExtractLUM.cpp MakeLUMSlice.cpp Trajectory.cpp ExtendTrajectories.cpp ClusterTrajectories.cpp RenovateTrajectory.cpp PlotCalculationProcess.cpp
OBJ = $(SRC:%.cpp=%.o)
INCLUDEDIR = -I/usr/include/i386-linux-gnu `pkg-config opencv --cflags-only-I`
LIBDIR = `pkg-config opencv --libs-only-L` -L/usr/lib -L/opt/intel/Compiler/11.1/064/ipp/ia32/sharedlib
LIB = `pkg-config opencv --libs-only-l` -lz -lippcore -lippi -lipps -lippm
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

main.o: /usr/local/include/opencv/cv.h
main.o: /usr/local/include/opencv2/core/core_c.h
main.o: /usr/local/include/opencv2/core/types_c.h /usr/include/assert.h
main.o: /usr/include/features.h /usr/include/i386-linux-gnu/bits/predefs.h
main.o: /usr/include/i386-linux-gnu/sys/cdefs.h
main.o: /usr/include/i386-linux-gnu/bits/wordsize.h
main.o: /usr/include/i386-linux-gnu/gnu/stubs.h
main.o: /usr/include/i386-linux-gnu/gnu/stubs-32.h /usr/include/stdlib.h
main.o: /usr/include/i386-linux-gnu/bits/waitflags.h
main.o: /usr/include/i386-linux-gnu/bits/waitstatus.h /usr/include/endian.h
main.o: /usr/include/i386-linux-gnu/bits/endian.h
main.o: /usr/include/i386-linux-gnu/bits/byteswap.h
main.o: /usr/include/i386-linux-gnu/sys/types.h
main.o: /usr/include/i386-linux-gnu/bits/types.h
main.o: /usr/include/i386-linux-gnu/bits/typesizes.h /usr/include/time.h
main.o: /usr/include/i386-linux-gnu/sys/select.h
main.o: /usr/include/i386-linux-gnu/bits/select.h
main.o: /usr/include/i386-linux-gnu/bits/sigset.h
main.o: /usr/include/i386-linux-gnu/bits/time.h
main.o: /usr/include/i386-linux-gnu/sys/sysmacros.h
main.o: /usr/include/i386-linux-gnu/bits/pthreadtypes.h /usr/include/alloca.h
main.o: /usr/include/string.h /usr/include/xlocale.h /usr/include/stdint.h
main.o: /usr/include/i386-linux-gnu/bits/wchar.h /usr/include/math.h
main.o: /usr/include/i386-linux-gnu/bits/huge_val.h
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
main.o: CameraUnit.h track.h PersonTrackingCommon.h pttype.h PosXYT.h
main.o: LinearUniformMotion.h ProbSampleReplace.h Trajectory.h
main.o: RenovateTrajectory.h
track.o: /usr/local/include/opencv/cv.h
track.o: /usr/local/include/opencv2/core/core_c.h
track.o: /usr/local/include/opencv2/core/types_c.h /usr/include/assert.h
track.o: /usr/include/features.h /usr/include/i386-linux-gnu/bits/predefs.h
track.o: /usr/include/i386-linux-gnu/sys/cdefs.h
track.o: /usr/include/i386-linux-gnu/bits/wordsize.h
track.o: /usr/include/i386-linux-gnu/gnu/stubs.h
track.o: /usr/include/i386-linux-gnu/gnu/stubs-32.h /usr/include/stdlib.h
track.o: /usr/include/i386-linux-gnu/bits/waitflags.h
track.o: /usr/include/i386-linux-gnu/bits/waitstatus.h /usr/include/endian.h
track.o: /usr/include/i386-linux-gnu/bits/endian.h
track.o: /usr/include/i386-linux-gnu/bits/byteswap.h
track.o: /usr/include/i386-linux-gnu/sys/types.h
track.o: /usr/include/i386-linux-gnu/bits/types.h
track.o: /usr/include/i386-linux-gnu/bits/typesizes.h /usr/include/time.h
track.o: /usr/include/i386-linux-gnu/sys/select.h
track.o: /usr/include/i386-linux-gnu/bits/select.h
track.o: /usr/include/i386-linux-gnu/bits/sigset.h
track.o: /usr/include/i386-linux-gnu/bits/time.h
track.o: /usr/include/i386-linux-gnu/sys/sysmacros.h
track.o: /usr/include/i386-linux-gnu/bits/pthreadtypes.h
track.o: /usr/include/alloca.h /usr/include/string.h /usr/include/xlocale.h
track.o: /usr/include/stdint.h /usr/include/i386-linux-gnu/bits/wchar.h
track.o: /usr/include/math.h /usr/include/i386-linux-gnu/bits/huge_val.h
track.o: /usr/include/i386-linux-gnu/bits/huge_valf.h
track.o: /usr/include/i386-linux-gnu/bits/huge_vall.h
track.o: /usr/include/i386-linux-gnu/bits/inf.h
track.o: /usr/include/i386-linux-gnu/bits/nan.h
track.o: /usr/include/i386-linux-gnu/bits/mathdef.h
track.o: /usr/include/i386-linux-gnu/bits/mathcalls.h
track.o: /usr/local/include/opencv2/core/core.hpp
track.o: /usr/local/include/opencv2/core/version.hpp
track.o: /usr/local/include/opencv2/core/operations.hpp /usr/include/limits.h
track.o: /usr/include/i386-linux-gnu/bits/posix1_lim.h
track.o: /usr/include/i386-linux-gnu/bits/local_lim.h
track.o: /usr/include/linux/limits.h
track.o: /usr/include/i386-linux-gnu/bits/posix2_lim.h
track.o: /usr/local/include/opencv2/core/mat.hpp
track.o: /usr/local/include/opencv2/imgproc/imgproc_c.h
track.o: /usr/local/include/opencv2/imgproc/types_c.h
track.o: /usr/local/include/opencv2/imgproc/imgproc.hpp
track.o: /usr/local/include/opencv2/video/tracking.hpp
track.o: /usr/local/include/opencv2/features2d/features2d.hpp
track.o: /usr/local/include/opencv2/flann/miniflann.hpp
track.o: /usr/local/include/opencv2/flann/flann.hpp
track.o: /usr/local/include/opencv2/calib3d/calib3d.hpp
track.o: /usr/local/include/opencv2/objdetect/objdetect.hpp
track.o: /usr/local/include/opencv2/legacy/compat.hpp
track.o: /usr/local/include/opencv/highgui.h
track.o: /usr/local/include/opencv2/highgui/highgui_c.h
track.o: /usr/local/include/opencv2/highgui/highgui.hpp track.h
track.o: PersonTrackingCommon.h pttype.h PosXYT.h LinearUniformMotion.h
track.o: ProbSampleReplace.h Trajectory.h RenovateTrajectory.h
CameraUnit.o: CameraUnit.h
ExtractLUM.o: /usr/local/include/opencv/cv.h
ExtractLUM.o: /usr/local/include/opencv2/core/core_c.h
ExtractLUM.o: /usr/local/include/opencv2/core/types_c.h /usr/include/assert.h
ExtractLUM.o: /usr/include/features.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/predefs.h
ExtractLUM.o: /usr/include/i386-linux-gnu/sys/cdefs.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/wordsize.h
ExtractLUM.o: /usr/include/i386-linux-gnu/gnu/stubs.h
ExtractLUM.o: /usr/include/i386-linux-gnu/gnu/stubs-32.h
ExtractLUM.o: /usr/include/stdlib.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/waitflags.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/waitstatus.h
ExtractLUM.o: /usr/include/endian.h /usr/include/i386-linux-gnu/bits/endian.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/byteswap.h
ExtractLUM.o: /usr/include/i386-linux-gnu/sys/types.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/types.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/typesizes.h
ExtractLUM.o: /usr/include/time.h /usr/include/i386-linux-gnu/sys/select.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/select.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/sigset.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/time.h
ExtractLUM.o: /usr/include/i386-linux-gnu/sys/sysmacros.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/pthreadtypes.h
ExtractLUM.o: /usr/include/alloca.h /usr/include/string.h
ExtractLUM.o: /usr/include/xlocale.h /usr/include/stdint.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/wchar.h /usr/include/math.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/huge_val.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/huge_valf.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/huge_vall.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/inf.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/nan.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/mathdef.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/mathcalls.h
ExtractLUM.o: /usr/local/include/opencv2/core/core.hpp
ExtractLUM.o: /usr/local/include/opencv2/core/version.hpp
ExtractLUM.o: /usr/local/include/opencv2/core/operations.hpp
ExtractLUM.o: /usr/include/limits.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/posix1_lim.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/local_lim.h
ExtractLUM.o: /usr/include/linux/limits.h
ExtractLUM.o: /usr/include/i386-linux-gnu/bits/posix2_lim.h
ExtractLUM.o: /usr/local/include/opencv2/core/mat.hpp
ExtractLUM.o: /usr/local/include/opencv2/imgproc/imgproc_c.h
ExtractLUM.o: /usr/local/include/opencv2/imgproc/types_c.h
ExtractLUM.o: /usr/local/include/opencv2/imgproc/imgproc.hpp
ExtractLUM.o: /usr/local/include/opencv2/video/tracking.hpp
ExtractLUM.o: /usr/local/include/opencv2/features2d/features2d.hpp
ExtractLUM.o: /usr/local/include/opencv2/flann/miniflann.hpp
ExtractLUM.o: /usr/local/include/opencv2/flann/flann.hpp
ExtractLUM.o: /usr/local/include/opencv2/calib3d/calib3d.hpp
ExtractLUM.o: /usr/local/include/opencv2/objdetect/objdetect.hpp
ExtractLUM.o: /usr/local/include/opencv2/legacy/compat.hpp
ExtractLUM.o: /usr/local/include/opencv/highgui.h
ExtractLUM.o: /usr/local/include/opencv2/highgui/highgui_c.h
ExtractLUM.o: /usr/local/include/opencv2/highgui/highgui.hpp track.h
ExtractLUM.o: PersonTrackingCommon.h pttype.h PosXYT.h LinearUniformMotion.h
ExtractLUM.o: ProbSampleReplace.h Trajectory.h RenovateTrajectory.h
MakeLUMSlice.o: track.h /usr/local/include/opencv/cv.h
MakeLUMSlice.o: /usr/local/include/opencv2/core/core_c.h
MakeLUMSlice.o: /usr/local/include/opencv2/core/types_c.h
MakeLUMSlice.o: /usr/include/assert.h /usr/include/features.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/predefs.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/sys/cdefs.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/wordsize.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/gnu/stubs.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/gnu/stubs-32.h
MakeLUMSlice.o: /usr/include/stdlib.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/waitflags.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/waitstatus.h
MakeLUMSlice.o: /usr/include/endian.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/endian.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/byteswap.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/sys/types.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/types.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/typesizes.h
MakeLUMSlice.o: /usr/include/time.h /usr/include/i386-linux-gnu/sys/select.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/select.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/sigset.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/time.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/sys/sysmacros.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/pthreadtypes.h
MakeLUMSlice.o: /usr/include/alloca.h /usr/include/string.h
MakeLUMSlice.o: /usr/include/xlocale.h /usr/include/stdint.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/wchar.h /usr/include/math.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/huge_val.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/huge_valf.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/huge_vall.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/inf.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/nan.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/mathdef.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/mathcalls.h
MakeLUMSlice.o: /usr/local/include/opencv2/core/core.hpp
MakeLUMSlice.o: /usr/local/include/opencv2/core/version.hpp
MakeLUMSlice.o: /usr/local/include/opencv2/core/operations.hpp
MakeLUMSlice.o: /usr/include/limits.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/posix1_lim.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/local_lim.h
MakeLUMSlice.o: /usr/include/linux/limits.h
MakeLUMSlice.o: /usr/include/i386-linux-gnu/bits/posix2_lim.h
MakeLUMSlice.o: /usr/local/include/opencv2/core/mat.hpp
MakeLUMSlice.o: /usr/local/include/opencv2/imgproc/imgproc_c.h
MakeLUMSlice.o: /usr/local/include/opencv2/imgproc/types_c.h
MakeLUMSlice.o: /usr/local/include/opencv2/imgproc/imgproc.hpp
MakeLUMSlice.o: /usr/local/include/opencv2/video/tracking.hpp
MakeLUMSlice.o: /usr/local/include/opencv2/features2d/features2d.hpp
MakeLUMSlice.o: /usr/local/include/opencv2/flann/miniflann.hpp
MakeLUMSlice.o: /usr/local/include/opencv2/flann/flann.hpp
MakeLUMSlice.o: /usr/local/include/opencv2/calib3d/calib3d.hpp
MakeLUMSlice.o: /usr/local/include/opencv2/objdetect/objdetect.hpp
MakeLUMSlice.o: /usr/local/include/opencv2/legacy/compat.hpp
MakeLUMSlice.o: /usr/local/include/opencv/highgui.h
MakeLUMSlice.o: /usr/local/include/opencv2/highgui/highgui_c.h
MakeLUMSlice.o: /usr/local/include/opencv2/highgui/highgui.hpp
MakeLUMSlice.o: PersonTrackingCommon.h pttype.h PosXYT.h
MakeLUMSlice.o: LinearUniformMotion.h ProbSampleReplace.h Trajectory.h
MakeLUMSlice.o: RenovateTrajectory.h
Trajectory.o: Trajectory.h PersonTrackingCommon.h pttype.h
Trajectory.o: /usr/include/stdint.h /usr/include/features.h
Trajectory.o: /usr/include/i386-linux-gnu/bits/predefs.h
Trajectory.o: /usr/include/i386-linux-gnu/sys/cdefs.h
Trajectory.o: /usr/include/i386-linux-gnu/bits/wordsize.h
Trajectory.o: /usr/include/i386-linux-gnu/gnu/stubs.h
Trajectory.o: /usr/include/i386-linux-gnu/gnu/stubs-32.h
Trajectory.o: /usr/include/i386-linux-gnu/bits/wchar.h PosXYT.h
Trajectory.o: /usr/include/math.h /usr/include/i386-linux-gnu/bits/huge_val.h
Trajectory.o: /usr/include/i386-linux-gnu/bits/huge_valf.h
Trajectory.o: /usr/include/i386-linux-gnu/bits/huge_vall.h
Trajectory.o: /usr/include/i386-linux-gnu/bits/inf.h
Trajectory.o: /usr/include/i386-linux-gnu/bits/nan.h
Trajectory.o: /usr/include/i386-linux-gnu/bits/mathdef.h
Trajectory.o: /usr/include/i386-linux-gnu/bits/mathcalls.h
Trajectory.o: LinearUniformMotion.h
ExtendTrajectories.o: /usr/include/stdlib.h /usr/include/features.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/predefs.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/sys/cdefs.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/wordsize.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/gnu/stubs.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/gnu/stubs-32.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/waitflags.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/waitstatus.h
ExtendTrajectories.o: /usr/include/endian.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/endian.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/byteswap.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/sys/types.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/types.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/typesizes.h
ExtendTrajectories.o: /usr/include/time.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/sys/select.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/select.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/sigset.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/time.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/sys/sysmacros.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/pthreadtypes.h
ExtendTrajectories.o: /usr/include/alloca.h track.h
ExtendTrajectories.o: /usr/local/include/opencv/cv.h
ExtendTrajectories.o: /usr/local/include/opencv2/core/core_c.h
ExtendTrajectories.o: /usr/local/include/opencv2/core/types_c.h
ExtendTrajectories.o: /usr/include/assert.h /usr/include/string.h
ExtendTrajectories.o: /usr/include/xlocale.h /usr/include/stdint.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/wchar.h
ExtendTrajectories.o: /usr/include/math.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/huge_val.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/huge_valf.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/huge_vall.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/inf.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/nan.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/mathdef.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/mathcalls.h
ExtendTrajectories.o: /usr/local/include/opencv2/core/core.hpp
ExtendTrajectories.o: /usr/local/include/opencv2/core/version.hpp
ExtendTrajectories.o: /usr/local/include/opencv2/core/operations.hpp
ExtendTrajectories.o: /usr/include/limits.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/posix1_lim.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/local_lim.h
ExtendTrajectories.o: /usr/include/linux/limits.h
ExtendTrajectories.o: /usr/include/i386-linux-gnu/bits/posix2_lim.h
ExtendTrajectories.o: /usr/local/include/opencv2/core/mat.hpp
ExtendTrajectories.o: /usr/local/include/opencv2/imgproc/imgproc_c.h
ExtendTrajectories.o: /usr/local/include/opencv2/imgproc/types_c.h
ExtendTrajectories.o: /usr/local/include/opencv2/imgproc/imgproc.hpp
ExtendTrajectories.o: /usr/local/include/opencv2/video/tracking.hpp
ExtendTrajectories.o: /usr/local/include/opencv2/features2d/features2d.hpp
ExtendTrajectories.o: /usr/local/include/opencv2/flann/miniflann.hpp
ExtendTrajectories.o: /usr/local/include/opencv2/flann/flann.hpp
ExtendTrajectories.o: /usr/local/include/opencv2/calib3d/calib3d.hpp
ExtendTrajectories.o: /usr/local/include/opencv2/objdetect/objdetect.hpp
ExtendTrajectories.o: /usr/local/include/opencv2/legacy/compat.hpp
ExtendTrajectories.o: /usr/local/include/opencv/highgui.h
ExtendTrajectories.o: /usr/local/include/opencv2/highgui/highgui_c.h
ExtendTrajectories.o: /usr/local/include/opencv2/highgui/highgui.hpp
ExtendTrajectories.o: PersonTrackingCommon.h pttype.h PosXYT.h
ExtendTrajectories.o: LinearUniformMotion.h ProbSampleReplace.h Trajectory.h
ExtendTrajectories.o: RenovateTrajectory.h
ClusterTrajectories.o: /usr/include/stdlib.h /usr/include/features.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/predefs.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/sys/cdefs.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/wordsize.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/gnu/stubs.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/gnu/stubs-32.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/waitflags.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/waitstatus.h
ClusterTrajectories.o: /usr/include/endian.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/endian.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/byteswap.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/sys/types.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/types.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/typesizes.h
ClusterTrajectories.o: /usr/include/time.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/sys/select.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/select.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/sigset.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/time.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/sys/sysmacros.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/pthreadtypes.h
ClusterTrajectories.o: /usr/include/alloca.h /usr/include/math.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/huge_val.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/huge_valf.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/huge_vall.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/inf.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/nan.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/mathdef.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/mathcalls.h track.h
ClusterTrajectories.o: /usr/local/include/opencv/cv.h
ClusterTrajectories.o: /usr/local/include/opencv2/core/core_c.h
ClusterTrajectories.o: /usr/local/include/opencv2/core/types_c.h
ClusterTrajectories.o: /usr/include/assert.h /usr/include/string.h
ClusterTrajectories.o: /usr/include/xlocale.h /usr/include/stdint.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/wchar.h
ClusterTrajectories.o: /usr/local/include/opencv2/core/core.hpp
ClusterTrajectories.o: /usr/local/include/opencv2/core/version.hpp
ClusterTrajectories.o: /usr/local/include/opencv2/core/operations.hpp
ClusterTrajectories.o: /usr/include/limits.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/posix1_lim.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/local_lim.h
ClusterTrajectories.o: /usr/include/linux/limits.h
ClusterTrajectories.o: /usr/include/i386-linux-gnu/bits/posix2_lim.h
ClusterTrajectories.o: /usr/local/include/opencv2/core/mat.hpp
ClusterTrajectories.o: /usr/local/include/opencv2/imgproc/imgproc_c.h
ClusterTrajectories.o: /usr/local/include/opencv2/imgproc/types_c.h
ClusterTrajectories.o: /usr/local/include/opencv2/imgproc/imgproc.hpp
ClusterTrajectories.o: /usr/local/include/opencv2/video/tracking.hpp
ClusterTrajectories.o: /usr/local/include/opencv2/features2d/features2d.hpp
ClusterTrajectories.o: /usr/local/include/opencv2/flann/miniflann.hpp
ClusterTrajectories.o: /usr/local/include/opencv2/flann/flann.hpp
ClusterTrajectories.o: /usr/local/include/opencv2/calib3d/calib3d.hpp
ClusterTrajectories.o: /usr/local/include/opencv2/objdetect/objdetect.hpp
ClusterTrajectories.o: /usr/local/include/opencv2/legacy/compat.hpp
ClusterTrajectories.o: /usr/local/include/opencv/highgui.h
ClusterTrajectories.o: /usr/local/include/opencv2/highgui/highgui_c.h
ClusterTrajectories.o: /usr/local/include/opencv2/highgui/highgui.hpp
ClusterTrajectories.o: PersonTrackingCommon.h pttype.h PosXYT.h
ClusterTrajectories.o: LinearUniformMotion.h ProbSampleReplace.h Trajectory.h
ClusterTrajectories.o: RenovateTrajectory.h
RenovateTrajectory.o: track.h /usr/local/include/opencv/cv.h
RenovateTrajectory.o: /usr/local/include/opencv2/core/core_c.h
RenovateTrajectory.o: /usr/local/include/opencv2/core/types_c.h
RenovateTrajectory.o: /usr/include/assert.h /usr/include/features.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/predefs.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/sys/cdefs.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/wordsize.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/gnu/stubs.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/gnu/stubs-32.h
RenovateTrajectory.o: /usr/include/stdlib.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/waitflags.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/waitstatus.h
RenovateTrajectory.o: /usr/include/endian.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/endian.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/byteswap.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/sys/types.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/types.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/typesizes.h
RenovateTrajectory.o: /usr/include/time.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/sys/select.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/select.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/sigset.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/time.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/sys/sysmacros.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/pthreadtypes.h
RenovateTrajectory.o: /usr/include/alloca.h /usr/include/string.h
RenovateTrajectory.o: /usr/include/xlocale.h /usr/include/stdint.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/wchar.h
RenovateTrajectory.o: /usr/include/math.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/huge_val.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/huge_valf.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/huge_vall.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/inf.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/nan.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/mathdef.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/mathcalls.h
RenovateTrajectory.o: /usr/local/include/opencv2/core/core.hpp
RenovateTrajectory.o: /usr/local/include/opencv2/core/version.hpp
RenovateTrajectory.o: /usr/local/include/opencv2/core/operations.hpp
RenovateTrajectory.o: /usr/include/limits.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/posix1_lim.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/local_lim.h
RenovateTrajectory.o: /usr/include/linux/limits.h
RenovateTrajectory.o: /usr/include/i386-linux-gnu/bits/posix2_lim.h
RenovateTrajectory.o: /usr/local/include/opencv2/core/mat.hpp
RenovateTrajectory.o: /usr/local/include/opencv2/imgproc/imgproc_c.h
RenovateTrajectory.o: /usr/local/include/opencv2/imgproc/types_c.h
RenovateTrajectory.o: /usr/local/include/opencv2/imgproc/imgproc.hpp
RenovateTrajectory.o: /usr/local/include/opencv2/video/tracking.hpp
RenovateTrajectory.o: /usr/local/include/opencv2/features2d/features2d.hpp
RenovateTrajectory.o: /usr/local/include/opencv2/flann/miniflann.hpp
RenovateTrajectory.o: /usr/local/include/opencv2/flann/flann.hpp
RenovateTrajectory.o: /usr/local/include/opencv2/calib3d/calib3d.hpp
RenovateTrajectory.o: /usr/local/include/opencv2/objdetect/objdetect.hpp
RenovateTrajectory.o: /usr/local/include/opencv2/legacy/compat.hpp
RenovateTrajectory.o: /usr/local/include/opencv/highgui.h
RenovateTrajectory.o: /usr/local/include/opencv2/highgui/highgui_c.h
RenovateTrajectory.o: /usr/local/include/opencv2/highgui/highgui.hpp
RenovateTrajectory.o: PersonTrackingCommon.h pttype.h PosXYT.h
RenovateTrajectory.o: LinearUniformMotion.h ProbSampleReplace.h Trajectory.h
RenovateTrajectory.o: RenovateTrajectory.h
PlotCalculationProcess.o: track.h /usr/local/include/opencv/cv.h
PlotCalculationProcess.o: /usr/local/include/opencv2/core/core_c.h
PlotCalculationProcess.o: /usr/local/include/opencv2/core/types_c.h
PlotCalculationProcess.o: /usr/include/assert.h /usr/include/features.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/predefs.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/sys/cdefs.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/wordsize.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/gnu/stubs.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/gnu/stubs-32.h
PlotCalculationProcess.o: /usr/include/stdlib.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/waitflags.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/waitstatus.h
PlotCalculationProcess.o: /usr/include/endian.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/endian.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/byteswap.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/sys/types.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/types.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/typesizes.h
PlotCalculationProcess.o: /usr/include/time.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/sys/select.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/select.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/sigset.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/time.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/sys/sysmacros.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/pthreadtypes.h
PlotCalculationProcess.o: /usr/include/alloca.h /usr/include/string.h
PlotCalculationProcess.o: /usr/include/xlocale.h /usr/include/stdint.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/wchar.h
PlotCalculationProcess.o: /usr/include/math.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/huge_val.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/huge_valf.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/huge_vall.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/inf.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/nan.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/mathdef.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/mathcalls.h
PlotCalculationProcess.o: /usr/local/include/opencv2/core/core.hpp
PlotCalculationProcess.o: /usr/local/include/opencv2/core/version.hpp
PlotCalculationProcess.o: /usr/local/include/opencv2/core/operations.hpp
PlotCalculationProcess.o: /usr/include/limits.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/posix1_lim.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/local_lim.h
PlotCalculationProcess.o: /usr/include/linux/limits.h
PlotCalculationProcess.o: /usr/include/i386-linux-gnu/bits/posix2_lim.h
PlotCalculationProcess.o: /usr/local/include/opencv2/core/mat.hpp
PlotCalculationProcess.o: /usr/local/include/opencv2/imgproc/imgproc_c.h
PlotCalculationProcess.o: /usr/local/include/opencv2/imgproc/types_c.h
PlotCalculationProcess.o: /usr/local/include/opencv2/imgproc/imgproc.hpp
PlotCalculationProcess.o: /usr/local/include/opencv2/video/tracking.hpp
PlotCalculationProcess.o: /usr/local/include/opencv2/features2d/features2d.hpp
PlotCalculationProcess.o: /usr/local/include/opencv2/flann/miniflann.hpp
PlotCalculationProcess.o: /usr/local/include/opencv2/flann/flann.hpp
PlotCalculationProcess.o: /usr/local/include/opencv2/calib3d/calib3d.hpp
PlotCalculationProcess.o: /usr/local/include/opencv2/objdetect/objdetect.hpp
PlotCalculationProcess.o: /usr/local/include/opencv2/legacy/compat.hpp
PlotCalculationProcess.o: /usr/local/include/opencv/highgui.h
PlotCalculationProcess.o: /usr/local/include/opencv2/highgui/highgui_c.h
PlotCalculationProcess.o: /usr/local/include/opencv2/highgui/highgui.hpp
PlotCalculationProcess.o: PersonTrackingCommon.h pttype.h PosXYT.h
PlotCalculationProcess.o: LinearUniformMotion.h ProbSampleReplace.h
PlotCalculationProcess.o: Trajectory.h RenovateTrajectory.h
