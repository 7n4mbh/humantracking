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
    std::map<int,cv::Mat> silhouette;

public:
    GestureRecognition();
    virtual ~GestureRecognition();

    void init( std::string recognition_result_path );
    void set_silhouette( int id, unsigned long long timeStamp, const cv::Mat& silhouette );
    cv::Mat get_silhouette( int id );
    void clear_silhouette( int id );
    void recognize( int id, unsigned long long timeStamp );
};

#endif