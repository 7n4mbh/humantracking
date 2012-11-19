#ifndef __HEADER_HUMANTRACKING__
#define __HEADER_HUMANTRACKING__

#if defined(WINDOWS) || defined(_WIN32)
#define WINDOWS_OS
#elif defined(linux) || defined(__linux__)
#define LINUX_OS
#endif

#include <string>

#ifdef WINDOWS_OS
#include <windows.h>
#endif
#ifdef LINUX_OS
#include <sys/time.h>
#endif

#include "opencv/cv.h"

#define DELTA_EPOCH_IN_MICROSECS  11644473600000000ULL

#define SIZE_BUFFER ( 10000 )

const CvScalar color_table[] = { CV_RGB( 255,   0,   0 )
                       , CV_RGB( 0  , 255,   0 )
                       , CV_RGB( 0  ,   0, 255 )
                       , CV_RGB( 128,   0, 128 )
                       , CV_RGB( 128, 128,   0 )
                       , CV_RGB( 0  , 128, 128 )
                       , CV_RGB( 128,   0,   0 )
                       , CV_RGB( 0  , 128,   0 )
                       , CV_RGB( 0  ,   0, 128 )
                       , CV_RGB( 255,   0, 255 )
                       , CV_RGB( 255, 255,   0 )
                       , CV_RGB( 0  , 255, 255 ) };
const int sizeColorTable = sizeof( color_table ) / sizeof( CvScalar );

typedef struct {
    unsigned int serialNumber;
    unsigned long long timeStamp;
    std::string data;
} PEPMapInfo;

typedef struct {
    unsigned int serialNumber;
    unsigned long long timeStamp;
    int width, height;
    std::string data;
} CameraImageInfo;

typedef struct {
    unsigned int serialNumber;
    unsigned long long timeStamp;
    int width, height;
    std::string data;
} GeometryMapInfo;

inline unsigned long long getTimeStamp()
{
#ifdef WINDOWS_OS
    FILETIME ft;
    unsigned __int64 tmpres = 0;
    time_t _sec, _usec;
    GetSystemTimeAsFileTime( &ft );
    tmpres |= ft.dwHighDateTime;
    tmpres <<= 32;
    tmpres |= ft.dwLowDateTime;
    tmpres /= 10;
    tmpres -= DELTA_EPOCH_IN_MICROSECS; 

    return (unsigned long long)tmpres;
#endif
#ifdef LINUX_OS
    timeval tv;
    gettimeofday( &tv, NULL );
    return (unsigned long long)tv.tv_sec * 1000000ULL + (unsigned long long)tv.tv_usec;
#endif
}

inline void copy( cv::Mat& img_dst, int dst_x, int dst_y, cv::Mat& img_src, int src_x, int src_y, int width, int height )
{
    //cout << "copy(): "
    //     << "width=" << width
    //     << ", height=" << height 
    //     << ", img_src.size().width=" << img_src.size().width
    //     << ", img_src.size().heigh=" << img_src.size().height
    //     << ", img_dst.size().width=" << img_dst.size().width
    //     << ", img_dst.size().height=" << img_dst.size().height
    //     << endl;

    if( dst_x + width > img_dst.size().width || dst_y + height > img_dst.size().height ) {
        return;
    }
    if( src_x + width > img_src.size().width || src_y + height > img_src.size().height ) {
        return;
    }

    //cv::Mat dst_roi( img_dst, cv::Rect( dst_x, dst_y, width, height ) );
    //cv::Mat src_roi( img_src, cv::Rect( src_x, src_y, width, height ) );
    //dst_roi = src_roi.clone();

    for( int x = 0; x < width; ++x ) {
        for( int y = 0; y < height; ++y ) {
            for ( int channel = 0; channel < img_dst.channels(); ++channel ) {
                *(unsigned char*)( img_dst.data + ( dst_y + y ) * img_dst.step + ( dst_x + x ) * img_dst.channels() + channel )
                    = *(unsigned char*)( img_src.data + ( src_y + y ) * img_src.step + ( src_x + x ) * img_src.channels() + channel );
            }
        }
    }
}

#endif
