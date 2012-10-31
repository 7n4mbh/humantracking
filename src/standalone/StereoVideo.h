#ifndef __HEADER_STEREOVIDEO__
#define __HEADER_STEREOVIDEO__

#include "FlyCapture2.h"
#include "triclops.h"
#include "opencv/cv.h"
#include "../humantracking.h"

#ifdef WINDOWS_OS
#include <windows.h>
#endif

class StereoVideo{
public:
    cv::Mat image;

private:
    FlyCapture2::CameraInfo camInfo;
    TriclopsContext triclops;
    cv::VideoCapture video;
    std::string strVideoFile;
    std::vector<unsigned long long> frame_to_timestamp;
    cv::Mat H;
    int frame;

public:
    StereoVideo();
    virtual ~StereoVideo();
    bool init( std::string strVideoFile );
    bool load_extrinsic_parameters();
    bool grab();
    int get_frame() { return frame; }
    unsigned long long get_timestamp() { return frame_to_timestamp[ frame ]; }
};

#endif