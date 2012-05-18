#ifndef __HEADER_GESTURERECOGNITION__
#define __HEADER_GESTURERECOGNITION__

#include "../humantracking.h"

#include <map>
#include <string>
#include <fstream>

#ifdef WINDOWS_OS
#include <windows.h>
#endif

#include "opencv/cv.h"
#include "opencv/highgui.h"

class GestureRecognition {
public:
    std::map<int,int> status;

private:
    std::string strRecognitionResultPath;
    std::map<int,std::ofstream> ofs;

public:
    GestureRecognition();
    virtual ~GestureRecognition();

    void init( std::string recognition_result_path );
    void SetSilhouette( int id, unsigned long long timeStamp, const cv::Mat& silhouette );
};

#endif