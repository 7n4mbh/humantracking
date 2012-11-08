#CC = gcc
#CXX = g++
CC = icc
CXX = icpc

EXENAME = ../../bin/standalone
SRC = main.cpp StereoVideo.cpp track.cpp ClusterTrajectories.cpp ExtendTrajectories.cpp ExtractLUM.cpp GestureRecognition.cpp MakeLUMSlice.cpp PlotCalculationProcess.cpp RenovateTrajectory.cpp TrackingResultResources.cpp Trajectory.cpp
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

main.o: /usr/local/include/opencv/cv.h
main.o: /usr/local/include/opencv2/core/core_c.h
main.o: /usr/local/include/opencv2/core/types_c.h /usr/include/assert.h
main.o: /usr/include/features.h /usr/include/sys/cdefs.h
main.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
main.o: /usr/include/gnu/stubs-64.h /usr/include/stdlib.h
main.o: /usr/include/sys/types.h /usr/include/bits/types.h
main.o: /usr/include/bits/typesizes.h /usr/include/time.h
main.o: /usr/include/endian.h /usr/include/bits/endian.h
main.o: /usr/include/sys/select.h /usr/include/bits/select.h
main.o: /usr/include/bits/sigset.h /usr/include/bits/time.h
main.o: /usr/include/sys/sysmacros.h /usr/include/bits/pthreadtypes.h
main.o: /usr/include/alloca.h /usr/include/string.h /usr/include/stdint.h
main.o: /usr/include/bits/wchar.h /usr/include/math.h
main.o: /usr/include/bits/huge_val.h /usr/include/bits/mathdef.h
main.o: /usr/include/bits/mathcalls.h
main.o: /usr/local/include/opencv2/core/core.hpp
main.o: /usr/local/include/opencv2/core/version.hpp
main.o: /usr/local/include/opencv2/core/operations.hpp /usr/include/limits.h
main.o: /usr/include/bits/posix1_lim.h /usr/include/bits/local_lim.h
main.o: /usr/include/linux/limits.h /usr/include/bits/posix2_lim.h
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
main.o: /usr/local/include/opencv2/highgui/highgui.hpp ../humantracking.h
main.o: track.h PersonTrackingCommon.h pttype.h PosXYT.h
main.o: LinearUniformMotion.h /usr/include/boost/shared_ptr.hpp
main.o: /usr/include/boost/config.hpp /usr/include/boost/config/user.hpp
main.o: /usr/include/boost/config/select_compiler_config.hpp
main.o: /usr/include/boost/config/compiler/gcc.hpp
main.o: /usr/include/boost/config/select_stdlib_config.hpp
main.o: /usr/include/boost/config/select_platform_config.hpp
main.o: /usr/include/boost/config/posix_features.hpp /usr/include/unistd.h
main.o: /usr/include/bits/posix_opt.h /usr/include/bits/confname.h
main.o: /usr/include/getopt.h /usr/include/boost/config/suffix.hpp
main.o: /usr/include/boost/assert.hpp /usr/include/boost/checked_delete.hpp
main.o: /usr/include/boost/throw_exception.hpp /usr/include/boost/config.hpp
main.o: /usr/include/boost/detail/shared_count.hpp
main.o: /usr/include/boost/detail/bad_weak_ptr.hpp
main.o: /usr/include/boost/detail/sp_counted_base.hpp
main.o: /usr/include/boost/detail/sp_counted_base_gcc_x86.hpp
main.o: /usr/include/boost/detail/sp_counted_impl.hpp
main.o: /usr/include/boost/detail/workaround.hpp ProbSampleReplace.h
main.o: Trajectory.h RenovateTrajectory.h TrackingResultResources.h
main.o: StereoVideo.h
StereoVideo.o: /usr/local/include/opencv/cv.h
StereoVideo.o: /usr/local/include/opencv2/core/core_c.h
StereoVideo.o: /usr/local/include/opencv2/core/types_c.h
StereoVideo.o: /usr/include/assert.h /usr/include/features.h
StereoVideo.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
StereoVideo.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h
StereoVideo.o: /usr/include/stdlib.h /usr/include/sys/types.h
StereoVideo.o: /usr/include/bits/types.h /usr/include/bits/typesizes.h
StereoVideo.o: /usr/include/time.h /usr/include/endian.h
StereoVideo.o: /usr/include/bits/endian.h /usr/include/sys/select.h
StereoVideo.o: /usr/include/bits/select.h /usr/include/bits/sigset.h
StereoVideo.o: /usr/include/bits/time.h /usr/include/sys/sysmacros.h
StereoVideo.o: /usr/include/bits/pthreadtypes.h /usr/include/alloca.h
StereoVideo.o: /usr/include/string.h /usr/include/stdint.h
StereoVideo.o: /usr/include/bits/wchar.h /usr/include/math.h
StereoVideo.o: /usr/include/bits/huge_val.h /usr/include/bits/mathdef.h
StereoVideo.o: /usr/include/bits/mathcalls.h
StereoVideo.o: /usr/local/include/opencv2/core/core.hpp
StereoVideo.o: /usr/local/include/opencv2/core/version.hpp
StereoVideo.o: /usr/local/include/opencv2/core/operations.hpp
StereoVideo.o: /usr/include/limits.h /usr/include/bits/posix1_lim.h
StereoVideo.o: /usr/include/bits/local_lim.h /usr/include/linux/limits.h
StereoVideo.o: /usr/include/bits/posix2_lim.h
StereoVideo.o: /usr/local/include/opencv2/core/mat.hpp
StereoVideo.o: /usr/local/include/opencv2/imgproc/imgproc_c.h
StereoVideo.o: /usr/local/include/opencv2/imgproc/types_c.h
StereoVideo.o: /usr/local/include/opencv2/imgproc/imgproc.hpp
StereoVideo.o: /usr/local/include/opencv2/video/tracking.hpp
StereoVideo.o: /usr/local/include/opencv2/features2d/features2d.hpp
StereoVideo.o: /usr/local/include/opencv2/flann/miniflann.hpp
StereoVideo.o: /usr/local/include/opencv2/flann/flann.hpp
StereoVideo.o: /usr/local/include/opencv2/calib3d/calib3d.hpp
StereoVideo.o: /usr/local/include/opencv2/objdetect/objdetect.hpp
StereoVideo.o: /usr/local/include/opencv2/legacy/compat.hpp
StereoVideo.o: /usr/local/include/opencv/highgui.h
StereoVideo.o: /usr/local/include/opencv2/highgui/highgui_c.h
StereoVideo.o: /usr/local/include/opencv2/highgui/highgui.hpp
StereoVideo.o: ../humantracking.h StereoVideo.h
track.o: /usr/local/include/opencv/cv.h
track.o: /usr/local/include/opencv2/core/core_c.h
track.o: /usr/local/include/opencv2/core/types_c.h /usr/include/assert.h
track.o: /usr/include/features.h /usr/include/sys/cdefs.h
track.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
track.o: /usr/include/gnu/stubs-64.h /usr/include/stdlib.h
track.o: /usr/include/sys/types.h /usr/include/bits/types.h
track.o: /usr/include/bits/typesizes.h /usr/include/time.h
track.o: /usr/include/endian.h /usr/include/bits/endian.h
track.o: /usr/include/sys/select.h /usr/include/bits/select.h
track.o: /usr/include/bits/sigset.h /usr/include/bits/time.h
track.o: /usr/include/sys/sysmacros.h /usr/include/bits/pthreadtypes.h
track.o: /usr/include/alloca.h /usr/include/string.h /usr/include/stdint.h
track.o: /usr/include/bits/wchar.h /usr/include/math.h
track.o: /usr/include/bits/huge_val.h /usr/include/bits/mathdef.h
track.o: /usr/include/bits/mathcalls.h
track.o: /usr/local/include/opencv2/core/core.hpp
track.o: /usr/local/include/opencv2/core/version.hpp
track.o: /usr/local/include/opencv2/core/operations.hpp /usr/include/limits.h
track.o: /usr/include/bits/posix1_lim.h /usr/include/bits/local_lim.h
track.o: /usr/include/linux/limits.h /usr/include/bits/posix2_lim.h
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
track.o: /usr/local/include/opencv2/highgui/highgui.hpp ../humantracking.h
track.o: track.h PersonTrackingCommon.h pttype.h PosXYT.h
track.o: LinearUniformMotion.h /usr/include/boost/shared_ptr.hpp
track.o: /usr/include/boost/config.hpp /usr/include/boost/config/user.hpp
track.o: /usr/include/boost/config/select_compiler_config.hpp
track.o: /usr/include/boost/config/compiler/gcc.hpp
track.o: /usr/include/boost/config/select_stdlib_config.hpp
track.o: /usr/include/boost/config/select_platform_config.hpp
track.o: /usr/include/boost/config/posix_features.hpp /usr/include/unistd.h
track.o: /usr/include/bits/posix_opt.h /usr/include/bits/confname.h
track.o: /usr/include/getopt.h /usr/include/boost/config/suffix.hpp
track.o: /usr/include/boost/assert.hpp /usr/include/boost/checked_delete.hpp
track.o: /usr/include/boost/throw_exception.hpp /usr/include/boost/config.hpp
track.o: /usr/include/boost/detail/shared_count.hpp
track.o: /usr/include/boost/detail/bad_weak_ptr.hpp
track.o: /usr/include/boost/detail/sp_counted_base.hpp
track.o: /usr/include/boost/detail/sp_counted_base_gcc_x86.hpp
track.o: /usr/include/boost/detail/sp_counted_impl.hpp
track.o: /usr/include/boost/detail/workaround.hpp ProbSampleReplace.h
track.o: Trajectory.h RenovateTrajectory.h
ClusterTrajectories.o: /usr/include/stdlib.h /usr/include/features.h
ClusterTrajectories.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
ClusterTrajectories.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h
ClusterTrajectories.o: /usr/include/sys/types.h /usr/include/bits/types.h
ClusterTrajectories.o: /usr/include/bits/typesizes.h /usr/include/time.h
ClusterTrajectories.o: /usr/include/endian.h /usr/include/bits/endian.h
ClusterTrajectories.o: /usr/include/sys/select.h /usr/include/bits/select.h
ClusterTrajectories.o: /usr/include/bits/sigset.h /usr/include/bits/time.h
ClusterTrajectories.o: /usr/include/sys/sysmacros.h
ClusterTrajectories.o: /usr/include/bits/pthreadtypes.h /usr/include/alloca.h
ClusterTrajectories.o: /usr/include/math.h /usr/include/bits/huge_val.h
ClusterTrajectories.o: /usr/include/bits/mathdef.h
ClusterTrajectories.o: /usr/include/bits/mathcalls.h track.h
ClusterTrajectories.o: /usr/local/include/opencv/cv.h
ClusterTrajectories.o: /usr/local/include/opencv2/core/core_c.h
ClusterTrajectories.o: /usr/local/include/opencv2/core/types_c.h
ClusterTrajectories.o: /usr/include/assert.h /usr/include/string.h
ClusterTrajectories.o: /usr/include/stdint.h /usr/include/bits/wchar.h
ClusterTrajectories.o: /usr/local/include/opencv2/core/core.hpp
ClusterTrajectories.o: /usr/local/include/opencv2/core/version.hpp
ClusterTrajectories.o: /usr/local/include/opencv2/core/operations.hpp
ClusterTrajectories.o: /usr/include/limits.h /usr/include/bits/posix1_lim.h
ClusterTrajectories.o: /usr/include/bits/local_lim.h
ClusterTrajectories.o: /usr/include/linux/limits.h
ClusterTrajectories.o: /usr/include/bits/posix2_lim.h
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
ClusterTrajectories.o: LinearUniformMotion.h
ClusterTrajectories.o: /usr/include/boost/shared_ptr.hpp
ClusterTrajectories.o: /usr/include/boost/config.hpp
ClusterTrajectories.o: /usr/include/boost/config/user.hpp
ClusterTrajectories.o: /usr/include/boost/config/select_compiler_config.hpp
ClusterTrajectories.o: /usr/include/boost/config/compiler/gcc.hpp
ClusterTrajectories.o: /usr/include/boost/config/select_stdlib_config.hpp
ClusterTrajectories.o: /usr/include/boost/config/select_platform_config.hpp
ClusterTrajectories.o: /usr/include/boost/config/posix_features.hpp
ClusterTrajectories.o: /usr/include/unistd.h /usr/include/bits/posix_opt.h
ClusterTrajectories.o: /usr/include/bits/confname.h /usr/include/getopt.h
ClusterTrajectories.o: /usr/include/boost/config/suffix.hpp
ClusterTrajectories.o: /usr/include/boost/assert.hpp
ClusterTrajectories.o: /usr/include/boost/checked_delete.hpp
ClusterTrajectories.o: /usr/include/boost/throw_exception.hpp
ClusterTrajectories.o: /usr/include/boost/config.hpp
ClusterTrajectories.o: /usr/include/boost/detail/shared_count.hpp
ClusterTrajectories.o: /usr/include/boost/detail/bad_weak_ptr.hpp
ClusterTrajectories.o: /usr/include/boost/detail/sp_counted_base.hpp
ClusterTrajectories.o: /usr/include/boost/detail/sp_counted_base_gcc_x86.hpp
ClusterTrajectories.o: /usr/include/boost/detail/sp_counted_impl.hpp
ClusterTrajectories.o: /usr/include/boost/detail/workaround.hpp
ClusterTrajectories.o: ProbSampleReplace.h Trajectory.h RenovateTrajectory.h
ExtendTrajectories.o: /usr/include/stdlib.h /usr/include/features.h
ExtendTrajectories.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
ExtendTrajectories.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h
ExtendTrajectories.o: /usr/include/sys/types.h /usr/include/bits/types.h
ExtendTrajectories.o: /usr/include/bits/typesizes.h /usr/include/time.h
ExtendTrajectories.o: /usr/include/endian.h /usr/include/bits/endian.h
ExtendTrajectories.o: /usr/include/sys/select.h /usr/include/bits/select.h
ExtendTrajectories.o: /usr/include/bits/sigset.h /usr/include/bits/time.h
ExtendTrajectories.o: /usr/include/sys/sysmacros.h
ExtendTrajectories.o: /usr/include/bits/pthreadtypes.h /usr/include/alloca.h
ExtendTrajectories.o: track.h /usr/local/include/opencv/cv.h
ExtendTrajectories.o: /usr/local/include/opencv2/core/core_c.h
ExtendTrajectories.o: /usr/local/include/opencv2/core/types_c.h
ExtendTrajectories.o: /usr/include/assert.h /usr/include/string.h
ExtendTrajectories.o: /usr/include/stdint.h /usr/include/bits/wchar.h
ExtendTrajectories.o: /usr/include/math.h /usr/include/bits/huge_val.h
ExtendTrajectories.o: /usr/include/bits/mathdef.h
ExtendTrajectories.o: /usr/include/bits/mathcalls.h
ExtendTrajectories.o: /usr/local/include/opencv2/core/core.hpp
ExtendTrajectories.o: /usr/local/include/opencv2/core/version.hpp
ExtendTrajectories.o: /usr/local/include/opencv2/core/operations.hpp
ExtendTrajectories.o: /usr/include/limits.h /usr/include/bits/posix1_lim.h
ExtendTrajectories.o: /usr/include/bits/local_lim.h
ExtendTrajectories.o: /usr/include/linux/limits.h
ExtendTrajectories.o: /usr/include/bits/posix2_lim.h
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
ExtendTrajectories.o: LinearUniformMotion.h /usr/include/boost/shared_ptr.hpp
ExtendTrajectories.o: /usr/include/boost/config.hpp
ExtendTrajectories.o: /usr/include/boost/config/user.hpp
ExtendTrajectories.o: /usr/include/boost/config/select_compiler_config.hpp
ExtendTrajectories.o: /usr/include/boost/config/compiler/gcc.hpp
ExtendTrajectories.o: /usr/include/boost/config/select_stdlib_config.hpp
ExtendTrajectories.o: /usr/include/boost/config/select_platform_config.hpp
ExtendTrajectories.o: /usr/include/boost/config/posix_features.hpp
ExtendTrajectories.o: /usr/include/unistd.h /usr/include/bits/posix_opt.h
ExtendTrajectories.o: /usr/include/bits/confname.h /usr/include/getopt.h
ExtendTrajectories.o: /usr/include/boost/config/suffix.hpp
ExtendTrajectories.o: /usr/include/boost/assert.hpp
ExtendTrajectories.o: /usr/include/boost/checked_delete.hpp
ExtendTrajectories.o: /usr/include/boost/throw_exception.hpp
ExtendTrajectories.o: /usr/include/boost/config.hpp
ExtendTrajectories.o: /usr/include/boost/detail/shared_count.hpp
ExtendTrajectories.o: /usr/include/boost/detail/bad_weak_ptr.hpp
ExtendTrajectories.o: /usr/include/boost/detail/sp_counted_base.hpp
ExtendTrajectories.o: /usr/include/boost/detail/sp_counted_base_gcc_x86.hpp
ExtendTrajectories.o: /usr/include/boost/detail/sp_counted_impl.hpp
ExtendTrajectories.o: /usr/include/boost/detail/workaround.hpp
ExtendTrajectories.o: ProbSampleReplace.h Trajectory.h RenovateTrajectory.h
ExtractLUM.o: /usr/local/include/opencv/cv.h
ExtractLUM.o: /usr/local/include/opencv2/core/core_c.h
ExtractLUM.o: /usr/local/include/opencv2/core/types_c.h /usr/include/assert.h
ExtractLUM.o: /usr/include/features.h /usr/include/sys/cdefs.h
ExtractLUM.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
ExtractLUM.o: /usr/include/gnu/stubs-64.h /usr/include/stdlib.h
ExtractLUM.o: /usr/include/sys/types.h /usr/include/bits/types.h
ExtractLUM.o: /usr/include/bits/typesizes.h /usr/include/time.h
ExtractLUM.o: /usr/include/endian.h /usr/include/bits/endian.h
ExtractLUM.o: /usr/include/sys/select.h /usr/include/bits/select.h
ExtractLUM.o: /usr/include/bits/sigset.h /usr/include/bits/time.h
ExtractLUM.o: /usr/include/sys/sysmacros.h /usr/include/bits/pthreadtypes.h
ExtractLUM.o: /usr/include/alloca.h /usr/include/string.h
ExtractLUM.o: /usr/include/stdint.h /usr/include/bits/wchar.h
ExtractLUM.o: /usr/include/math.h /usr/include/bits/huge_val.h
ExtractLUM.o: /usr/include/bits/mathdef.h /usr/include/bits/mathcalls.h
ExtractLUM.o: /usr/local/include/opencv2/core/core.hpp
ExtractLUM.o: /usr/local/include/opencv2/core/version.hpp
ExtractLUM.o: /usr/local/include/opencv2/core/operations.hpp
ExtractLUM.o: /usr/include/limits.h /usr/include/bits/posix1_lim.h
ExtractLUM.o: /usr/include/bits/local_lim.h /usr/include/linux/limits.h
ExtractLUM.o: /usr/include/bits/posix2_lim.h
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
ExtractLUM.o: /usr/include/boost/shared_ptr.hpp /usr/include/boost/config.hpp
ExtractLUM.o: /usr/include/boost/config/user.hpp
ExtractLUM.o: /usr/include/boost/config/select_compiler_config.hpp
ExtractLUM.o: /usr/include/boost/config/compiler/gcc.hpp
ExtractLUM.o: /usr/include/boost/config/select_stdlib_config.hpp
ExtractLUM.o: /usr/include/boost/config/select_platform_config.hpp
ExtractLUM.o: /usr/include/boost/config/posix_features.hpp
ExtractLUM.o: /usr/include/unistd.h /usr/include/bits/posix_opt.h
ExtractLUM.o: /usr/include/bits/confname.h /usr/include/getopt.h
ExtractLUM.o: /usr/include/boost/config/suffix.hpp
ExtractLUM.o: /usr/include/boost/assert.hpp
ExtractLUM.o: /usr/include/boost/checked_delete.hpp
ExtractLUM.o: /usr/include/boost/throw_exception.hpp
ExtractLUM.o: /usr/include/boost/config.hpp
ExtractLUM.o: /usr/include/boost/detail/shared_count.hpp
ExtractLUM.o: /usr/include/boost/detail/bad_weak_ptr.hpp
ExtractLUM.o: /usr/include/boost/detail/sp_counted_base.hpp
ExtractLUM.o: /usr/include/boost/detail/sp_counted_base_gcc_x86.hpp
ExtractLUM.o: /usr/include/boost/detail/sp_counted_impl.hpp
ExtractLUM.o: /usr/include/boost/detail/workaround.hpp ProbSampleReplace.h
ExtractLUM.o: Trajectory.h RenovateTrajectory.h
GestureRecognition.o: ../humantracking.h /usr/local/include/opencv/cv.h
GestureRecognition.o: /usr/local/include/opencv2/core/core_c.h
GestureRecognition.o: /usr/local/include/opencv2/core/types_c.h
GestureRecognition.o: /usr/include/assert.h /usr/include/features.h
GestureRecognition.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
GestureRecognition.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h
GestureRecognition.o: /usr/include/stdlib.h /usr/include/sys/types.h
GestureRecognition.o: /usr/include/bits/types.h /usr/include/bits/typesizes.h
GestureRecognition.o: /usr/include/time.h /usr/include/endian.h
GestureRecognition.o: /usr/include/bits/endian.h /usr/include/sys/select.h
GestureRecognition.o: /usr/include/bits/select.h /usr/include/bits/sigset.h
GestureRecognition.o: /usr/include/bits/time.h /usr/include/sys/sysmacros.h
GestureRecognition.o: /usr/include/bits/pthreadtypes.h /usr/include/alloca.h
GestureRecognition.o: /usr/include/string.h /usr/include/stdint.h
GestureRecognition.o: /usr/include/bits/wchar.h /usr/include/math.h
GestureRecognition.o: /usr/include/bits/huge_val.h
GestureRecognition.o: /usr/include/bits/mathdef.h
GestureRecognition.o: /usr/include/bits/mathcalls.h
GestureRecognition.o: /usr/local/include/opencv2/core/core.hpp
GestureRecognition.o: /usr/local/include/opencv2/core/version.hpp
GestureRecognition.o: /usr/local/include/opencv2/core/operations.hpp
GestureRecognition.o: /usr/include/limits.h /usr/include/bits/posix1_lim.h
GestureRecognition.o: /usr/include/bits/local_lim.h
GestureRecognition.o: /usr/include/linux/limits.h
GestureRecognition.o: /usr/include/bits/posix2_lim.h
GestureRecognition.o: /usr/local/include/opencv2/core/mat.hpp
GestureRecognition.o: /usr/local/include/opencv2/imgproc/imgproc_c.h
GestureRecognition.o: /usr/local/include/opencv2/imgproc/types_c.h
GestureRecognition.o: /usr/local/include/opencv2/imgproc/imgproc.hpp
GestureRecognition.o: /usr/local/include/opencv2/video/tracking.hpp
GestureRecognition.o: /usr/local/include/opencv2/features2d/features2d.hpp
GestureRecognition.o: /usr/local/include/opencv2/flann/miniflann.hpp
GestureRecognition.o: /usr/local/include/opencv2/flann/flann.hpp
GestureRecognition.o: /usr/local/include/opencv2/calib3d/calib3d.hpp
GestureRecognition.o: /usr/local/include/opencv2/objdetect/objdetect.hpp
GestureRecognition.o: /usr/local/include/opencv2/legacy/compat.hpp
GestureRecognition.o: GestureRecognition.h
GestureRecognition.o: /usr/local/include/opencv/highgui.h
GestureRecognition.o: /usr/local/include/opencv2/highgui/highgui_c.h
GestureRecognition.o: /usr/local/include/opencv2/highgui/highgui.hpp
MakeLUMSlice.o: track.h /usr/local/include/opencv/cv.h
MakeLUMSlice.o: /usr/local/include/opencv2/core/core_c.h
MakeLUMSlice.o: /usr/local/include/opencv2/core/types_c.h
MakeLUMSlice.o: /usr/include/assert.h /usr/include/features.h
MakeLUMSlice.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
MakeLUMSlice.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h
MakeLUMSlice.o: /usr/include/stdlib.h /usr/include/sys/types.h
MakeLUMSlice.o: /usr/include/bits/types.h /usr/include/bits/typesizes.h
MakeLUMSlice.o: /usr/include/time.h /usr/include/endian.h
MakeLUMSlice.o: /usr/include/bits/endian.h /usr/include/sys/select.h
MakeLUMSlice.o: /usr/include/bits/select.h /usr/include/bits/sigset.h
MakeLUMSlice.o: /usr/include/bits/time.h /usr/include/sys/sysmacros.h
MakeLUMSlice.o: /usr/include/bits/pthreadtypes.h /usr/include/alloca.h
MakeLUMSlice.o: /usr/include/string.h /usr/include/stdint.h
MakeLUMSlice.o: /usr/include/bits/wchar.h /usr/include/math.h
MakeLUMSlice.o: /usr/include/bits/huge_val.h /usr/include/bits/mathdef.h
MakeLUMSlice.o: /usr/include/bits/mathcalls.h
MakeLUMSlice.o: /usr/local/include/opencv2/core/core.hpp
MakeLUMSlice.o: /usr/local/include/opencv2/core/version.hpp
MakeLUMSlice.o: /usr/local/include/opencv2/core/operations.hpp
MakeLUMSlice.o: /usr/include/limits.h /usr/include/bits/posix1_lim.h
MakeLUMSlice.o: /usr/include/bits/local_lim.h /usr/include/linux/limits.h
MakeLUMSlice.o: /usr/include/bits/posix2_lim.h
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
MakeLUMSlice.o: LinearUniformMotion.h /usr/include/boost/shared_ptr.hpp
MakeLUMSlice.o: /usr/include/boost/config.hpp
MakeLUMSlice.o: /usr/include/boost/config/user.hpp
MakeLUMSlice.o: /usr/include/boost/config/select_compiler_config.hpp
MakeLUMSlice.o: /usr/include/boost/config/compiler/gcc.hpp
MakeLUMSlice.o: /usr/include/boost/config/select_stdlib_config.hpp
MakeLUMSlice.o: /usr/include/boost/config/select_platform_config.hpp
MakeLUMSlice.o: /usr/include/boost/config/posix_features.hpp
MakeLUMSlice.o: /usr/include/unistd.h /usr/include/bits/posix_opt.h
MakeLUMSlice.o: /usr/include/bits/confname.h /usr/include/getopt.h
MakeLUMSlice.o: /usr/include/boost/config/suffix.hpp
MakeLUMSlice.o: /usr/include/boost/assert.hpp
MakeLUMSlice.o: /usr/include/boost/checked_delete.hpp
MakeLUMSlice.o: /usr/include/boost/throw_exception.hpp
MakeLUMSlice.o: /usr/include/boost/config.hpp
MakeLUMSlice.o: /usr/include/boost/detail/shared_count.hpp
MakeLUMSlice.o: /usr/include/boost/detail/bad_weak_ptr.hpp
MakeLUMSlice.o: /usr/include/boost/detail/sp_counted_base.hpp
MakeLUMSlice.o: /usr/include/boost/detail/sp_counted_base_gcc_x86.hpp
MakeLUMSlice.o: /usr/include/boost/detail/sp_counted_impl.hpp
MakeLUMSlice.o: /usr/include/boost/detail/workaround.hpp ProbSampleReplace.h
MakeLUMSlice.o: Trajectory.h RenovateTrajectory.h
PlotCalculationProcess.o: ../humantracking.h /usr/local/include/opencv/cv.h
PlotCalculationProcess.o: /usr/local/include/opencv2/core/core_c.h
PlotCalculationProcess.o: /usr/local/include/opencv2/core/types_c.h
PlotCalculationProcess.o: /usr/include/assert.h /usr/include/features.h
PlotCalculationProcess.o: /usr/include/sys/cdefs.h
PlotCalculationProcess.o: /usr/include/bits/wordsize.h
PlotCalculationProcess.o: /usr/include/gnu/stubs.h
PlotCalculationProcess.o: /usr/include/gnu/stubs-64.h /usr/include/stdlib.h
PlotCalculationProcess.o: /usr/include/sys/types.h /usr/include/bits/types.h
PlotCalculationProcess.o: /usr/include/bits/typesizes.h /usr/include/time.h
PlotCalculationProcess.o: /usr/include/endian.h /usr/include/bits/endian.h
PlotCalculationProcess.o: /usr/include/sys/select.h
PlotCalculationProcess.o: /usr/include/bits/select.h
PlotCalculationProcess.o: /usr/include/bits/sigset.h /usr/include/bits/time.h
PlotCalculationProcess.o: /usr/include/sys/sysmacros.h
PlotCalculationProcess.o: /usr/include/bits/pthreadtypes.h
PlotCalculationProcess.o: /usr/include/alloca.h /usr/include/string.h
PlotCalculationProcess.o: /usr/include/stdint.h /usr/include/bits/wchar.h
PlotCalculationProcess.o: /usr/include/math.h /usr/include/bits/huge_val.h
PlotCalculationProcess.o: /usr/include/bits/mathdef.h
PlotCalculationProcess.o: /usr/include/bits/mathcalls.h
PlotCalculationProcess.o: /usr/local/include/opencv2/core/core.hpp
PlotCalculationProcess.o: /usr/local/include/opencv2/core/version.hpp
PlotCalculationProcess.o: /usr/local/include/opencv2/core/operations.hpp
PlotCalculationProcess.o: /usr/include/limits.h
PlotCalculationProcess.o: /usr/include/bits/posix1_lim.h
PlotCalculationProcess.o: /usr/include/bits/local_lim.h
PlotCalculationProcess.o: /usr/include/linux/limits.h
PlotCalculationProcess.o: /usr/include/bits/posix2_lim.h
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
PlotCalculationProcess.o: track.h /usr/local/include/opencv/highgui.h
PlotCalculationProcess.o: /usr/local/include/opencv2/highgui/highgui_c.h
PlotCalculationProcess.o: /usr/local/include/opencv2/highgui/highgui.hpp
PlotCalculationProcess.o: PersonTrackingCommon.h pttype.h PosXYT.h
PlotCalculationProcess.o: LinearUniformMotion.h
PlotCalculationProcess.o: /usr/include/boost/shared_ptr.hpp
PlotCalculationProcess.o: /usr/include/boost/config.hpp
PlotCalculationProcess.o: /usr/include/boost/config/user.hpp
PlotCalculationProcess.o: /usr/include/boost/config/select_compiler_config.hpp
PlotCalculationProcess.o: /usr/include/boost/config/compiler/gcc.hpp
PlotCalculationProcess.o: /usr/include/boost/config/select_stdlib_config.hpp
PlotCalculationProcess.o: /usr/include/boost/config/select_platform_config.hpp
PlotCalculationProcess.o: /usr/include/boost/config/posix_features.hpp
PlotCalculationProcess.o: /usr/include/unistd.h /usr/include/bits/posix_opt.h
PlotCalculationProcess.o: /usr/include/bits/confname.h /usr/include/getopt.h
PlotCalculationProcess.o: /usr/include/boost/config/suffix.hpp
PlotCalculationProcess.o: /usr/include/boost/assert.hpp
PlotCalculationProcess.o: /usr/include/boost/checked_delete.hpp
PlotCalculationProcess.o: /usr/include/boost/throw_exception.hpp
PlotCalculationProcess.o: /usr/include/boost/config.hpp
PlotCalculationProcess.o: /usr/include/boost/detail/shared_count.hpp
PlotCalculationProcess.o: /usr/include/boost/detail/bad_weak_ptr.hpp
PlotCalculationProcess.o: /usr/include/boost/detail/sp_counted_base.hpp
PlotCalculationProcess.o: /usr/include/boost/detail/sp_counted_base_gcc_x86.hpp
PlotCalculationProcess.o: /usr/include/boost/detail/sp_counted_impl.hpp
PlotCalculationProcess.o: /usr/include/boost/detail/workaround.hpp
PlotCalculationProcess.o: ProbSampleReplace.h Trajectory.h
PlotCalculationProcess.o: RenovateTrajectory.h
RenovateTrajectory.o: track.h /usr/local/include/opencv/cv.h
RenovateTrajectory.o: /usr/local/include/opencv2/core/core_c.h
RenovateTrajectory.o: /usr/local/include/opencv2/core/types_c.h
RenovateTrajectory.o: /usr/include/assert.h /usr/include/features.h
RenovateTrajectory.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
RenovateTrajectory.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h
RenovateTrajectory.o: /usr/include/stdlib.h /usr/include/sys/types.h
RenovateTrajectory.o: /usr/include/bits/types.h /usr/include/bits/typesizes.h
RenovateTrajectory.o: /usr/include/time.h /usr/include/endian.h
RenovateTrajectory.o: /usr/include/bits/endian.h /usr/include/sys/select.h
RenovateTrajectory.o: /usr/include/bits/select.h /usr/include/bits/sigset.h
RenovateTrajectory.o: /usr/include/bits/time.h /usr/include/sys/sysmacros.h
RenovateTrajectory.o: /usr/include/bits/pthreadtypes.h /usr/include/alloca.h
RenovateTrajectory.o: /usr/include/string.h /usr/include/stdint.h
RenovateTrajectory.o: /usr/include/bits/wchar.h /usr/include/math.h
RenovateTrajectory.o: /usr/include/bits/huge_val.h
RenovateTrajectory.o: /usr/include/bits/mathdef.h
RenovateTrajectory.o: /usr/include/bits/mathcalls.h
RenovateTrajectory.o: /usr/local/include/opencv2/core/core.hpp
RenovateTrajectory.o: /usr/local/include/opencv2/core/version.hpp
RenovateTrajectory.o: /usr/local/include/opencv2/core/operations.hpp
RenovateTrajectory.o: /usr/include/limits.h /usr/include/bits/posix1_lim.h
RenovateTrajectory.o: /usr/include/bits/local_lim.h
RenovateTrajectory.o: /usr/include/linux/limits.h
RenovateTrajectory.o: /usr/include/bits/posix2_lim.h
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
RenovateTrajectory.o: LinearUniformMotion.h /usr/include/boost/shared_ptr.hpp
RenovateTrajectory.o: /usr/include/boost/config.hpp
RenovateTrajectory.o: /usr/include/boost/config/user.hpp
RenovateTrajectory.o: /usr/include/boost/config/select_compiler_config.hpp
RenovateTrajectory.o: /usr/include/boost/config/compiler/gcc.hpp
RenovateTrajectory.o: /usr/include/boost/config/select_stdlib_config.hpp
RenovateTrajectory.o: /usr/include/boost/config/select_platform_config.hpp
RenovateTrajectory.o: /usr/include/boost/config/posix_features.hpp
RenovateTrajectory.o: /usr/include/unistd.h /usr/include/bits/posix_opt.h
RenovateTrajectory.o: /usr/include/bits/confname.h /usr/include/getopt.h
RenovateTrajectory.o: /usr/include/boost/config/suffix.hpp
RenovateTrajectory.o: /usr/include/boost/assert.hpp
RenovateTrajectory.o: /usr/include/boost/checked_delete.hpp
RenovateTrajectory.o: /usr/include/boost/throw_exception.hpp
RenovateTrajectory.o: /usr/include/boost/config.hpp
RenovateTrajectory.o: /usr/include/boost/detail/shared_count.hpp
RenovateTrajectory.o: /usr/include/boost/detail/bad_weak_ptr.hpp
RenovateTrajectory.o: /usr/include/boost/detail/sp_counted_base.hpp
RenovateTrajectory.o: /usr/include/boost/detail/sp_counted_base_gcc_x86.hpp
RenovateTrajectory.o: /usr/include/boost/detail/sp_counted_impl.hpp
RenovateTrajectory.o: /usr/include/boost/detail/workaround.hpp
RenovateTrajectory.o: ProbSampleReplace.h Trajectory.h RenovateTrajectory.h
TrackingResultResources.o: ../humantracking.h /usr/local/include/opencv/cv.h
TrackingResultResources.o: /usr/local/include/opencv2/core/core_c.h
TrackingResultResources.o: /usr/local/include/opencv2/core/types_c.h
TrackingResultResources.o: /usr/include/assert.h /usr/include/features.h
TrackingResultResources.o: /usr/include/sys/cdefs.h
TrackingResultResources.o: /usr/include/bits/wordsize.h
TrackingResultResources.o: /usr/include/gnu/stubs.h
TrackingResultResources.o: /usr/include/gnu/stubs-64.h /usr/include/stdlib.h
TrackingResultResources.o: /usr/include/sys/types.h /usr/include/bits/types.h
TrackingResultResources.o: /usr/include/bits/typesizes.h /usr/include/time.h
TrackingResultResources.o: /usr/include/endian.h /usr/include/bits/endian.h
TrackingResultResources.o: /usr/include/sys/select.h
TrackingResultResources.o: /usr/include/bits/select.h
TrackingResultResources.o: /usr/include/bits/sigset.h
TrackingResultResources.o: /usr/include/bits/time.h
TrackingResultResources.o: /usr/include/sys/sysmacros.h
TrackingResultResources.o: /usr/include/bits/pthreadtypes.h
TrackingResultResources.o: /usr/include/alloca.h /usr/include/string.h
TrackingResultResources.o: /usr/include/stdint.h /usr/include/bits/wchar.h
TrackingResultResources.o: /usr/include/math.h /usr/include/bits/huge_val.h
TrackingResultResources.o: /usr/include/bits/mathdef.h
TrackingResultResources.o: /usr/include/bits/mathcalls.h
TrackingResultResources.o: /usr/local/include/opencv2/core/core.hpp
TrackingResultResources.o: /usr/local/include/opencv2/core/version.hpp
TrackingResultResources.o: /usr/local/include/opencv2/core/operations.hpp
TrackingResultResources.o: /usr/include/limits.h
TrackingResultResources.o: /usr/include/bits/posix1_lim.h
TrackingResultResources.o: /usr/include/bits/local_lim.h
TrackingResultResources.o: /usr/include/linux/limits.h
TrackingResultResources.o: /usr/include/bits/posix2_lim.h
TrackingResultResources.o: /usr/local/include/opencv2/core/mat.hpp
TrackingResultResources.o: /usr/local/include/opencv2/imgproc/imgproc_c.h
TrackingResultResources.o: /usr/local/include/opencv2/imgproc/types_c.h
TrackingResultResources.o: /usr/local/include/opencv2/imgproc/imgproc.hpp
TrackingResultResources.o: /usr/local/include/opencv2/video/tracking.hpp
TrackingResultResources.o: /usr/local/include/opencv2/features2d/features2d.hpp
TrackingResultResources.o: /usr/local/include/opencv2/flann/miniflann.hpp
TrackingResultResources.o: /usr/local/include/opencv2/flann/flann.hpp
TrackingResultResources.o: /usr/local/include/opencv2/calib3d/calib3d.hpp
TrackingResultResources.o: /usr/local/include/opencv2/objdetect/objdetect.hpp
TrackingResultResources.o: /usr/local/include/opencv2/legacy/compat.hpp
TrackingResultResources.o: TrackingResultResources.h
TrackingResultResources.o: /usr/local/include/opencv/highgui.h
TrackingResultResources.o: /usr/local/include/opencv2/highgui/highgui_c.h
TrackingResultResources.o: /usr/local/include/opencv2/highgui/highgui.hpp
TrackingResultResources.o: Trajectory.h PersonTrackingCommon.h pttype.h
TrackingResultResources.o: PosXYT.h LinearUniformMotion.h
TrackingResultResources.o: /usr/include/boost/shared_ptr.hpp
TrackingResultResources.o: /usr/include/boost/config.hpp
TrackingResultResources.o: /usr/include/boost/config/user.hpp
TrackingResultResources.o: /usr/include/boost/config/select_compiler_config.hpp
TrackingResultResources.o: /usr/include/boost/config/compiler/gcc.hpp
TrackingResultResources.o: /usr/include/boost/config/select_stdlib_config.hpp
TrackingResultResources.o: /usr/include/boost/config/select_platform_config.hpp
TrackingResultResources.o: /usr/include/boost/config/posix_features.hpp
TrackingResultResources.o: /usr/include/unistd.h
TrackingResultResources.o: /usr/include/bits/posix_opt.h
TrackingResultResources.o: /usr/include/bits/confname.h /usr/include/getopt.h
TrackingResultResources.o: /usr/include/boost/config/suffix.hpp
TrackingResultResources.o: /usr/include/boost/assert.hpp
TrackingResultResources.o: /usr/include/boost/checked_delete.hpp
TrackingResultResources.o: /usr/include/boost/throw_exception.hpp
TrackingResultResources.o: /usr/include/boost/config.hpp
TrackingResultResources.o: /usr/include/boost/detail/shared_count.hpp
TrackingResultResources.o: /usr/include/boost/detail/bad_weak_ptr.hpp
TrackingResultResources.o: /usr/include/boost/detail/sp_counted_base.hpp
TrackingResultResources.o: /usr/include/boost/detail/sp_counted_base_gcc_x86.hpp
TrackingResultResources.o: /usr/include/boost/detail/sp_counted_impl.hpp
TrackingResultResources.o: /usr/include/boost/detail/workaround.hpp
TrackingResultResources.o: GestureRecognition.h /usr/include/zlib.h
TrackingResultResources.o: /usr/include/zconf.h
Trajectory.o: Trajectory.h PersonTrackingCommon.h pttype.h
Trajectory.o: /usr/include/stdint.h /usr/include/features.h
Trajectory.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
Trajectory.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h
Trajectory.o: /usr/include/bits/wchar.h PosXYT.h /usr/include/math.h
Trajectory.o: /usr/include/bits/huge_val.h /usr/include/bits/mathdef.h
Trajectory.o: /usr/include/bits/mathcalls.h LinearUniformMotion.h
Trajectory.o: /usr/include/boost/shared_ptr.hpp /usr/include/boost/config.hpp
Trajectory.o: /usr/include/boost/config/user.hpp
Trajectory.o: /usr/include/boost/config/select_compiler_config.hpp
Trajectory.o: /usr/include/boost/config/compiler/gcc.hpp
Trajectory.o: /usr/include/boost/config/select_stdlib_config.hpp
Trajectory.o: /usr/include/boost/config/select_platform_config.hpp
Trajectory.o: /usr/include/boost/config/posix_features.hpp
Trajectory.o: /usr/include/unistd.h /usr/include/bits/posix_opt.h
Trajectory.o: /usr/include/bits/types.h /usr/include/bits/typesizes.h
Trajectory.o: /usr/include/bits/confname.h /usr/include/getopt.h
Trajectory.o: /usr/include/boost/config/suffix.hpp /usr/include/limits.h
Trajectory.o: /usr/include/bits/posix1_lim.h /usr/include/bits/local_lim.h
Trajectory.o: /usr/include/linux/limits.h /usr/include/bits/posix2_lim.h
Trajectory.o: /usr/include/boost/assert.hpp /usr/include/assert.h
Trajectory.o: /usr/include/boost/checked_delete.hpp
Trajectory.o: /usr/include/boost/throw_exception.hpp
Trajectory.o: /usr/include/boost/config.hpp
Trajectory.o: /usr/include/boost/detail/shared_count.hpp
Trajectory.o: /usr/include/boost/detail/bad_weak_ptr.hpp
Trajectory.o: /usr/include/boost/detail/sp_counted_base.hpp
Trajectory.o: /usr/include/boost/detail/sp_counted_base_gcc_x86.hpp
Trajectory.o: /usr/include/boost/detail/sp_counted_impl.hpp
Trajectory.o: /usr/include/boost/detail/workaround.hpp
