#ifndef __HEADER_STEREOVIDEO__
#define __HEADER_STEREOVIDEO__

#include <fstream>

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
    cv::Mat image_rectified;
    cv::Mat image_depth;
    cv::Mat image_occupancy;
    cv::Mat occupancy;
    cv::Mat geometry;
    cv::Mat silhouette;

private:
    FlyCapture2::CameraInfo camInfo;
    TriclopsContext triclops;
    cv::VideoCapture video;
    std::string strVideoFile;
    std::vector<unsigned long long> frame_to_timestamp;
    cv::Mat H;
    int frame;
    int width, height;
    cv::Mat img_background;
    cv::Mat img_background_cam;
    //std::ofstream* p_ofs_log;

public:
    StereoVideo();
    StereoVideo( const StereoVideo& obj );
    virtual ~StereoVideo();
    bool init( std::string strVideoFile );
    bool SetStereoParameters( int width, int height, bool flg_output_msg = true );
    bool load_extrinsic_parameters();
    bool load_background();
    bool grab();
    int get_frame() { return frame; }
    unsigned long long get_timestamp() { return frame_to_timestamp[ frame ]; }
    void create_pepmap();

private:
    void stereo( TriclopsImage16* pDst, cv::Mat* pDstRefImg, const cv::Mat& src );
};

#endif
