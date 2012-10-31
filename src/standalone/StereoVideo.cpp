#include <iostream>
#include <fstream>
#include <sstream>
#include <map>

#include "FlyCapture2.h"
#include "triclops.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "../humantracking.h"
#include "StereoVideo.h"

#define _HANDLE_TRICLOPS_ERROR( description, error )    \
{ \
   if( error != TriclopsErrorOk ) \
   { \
      printf( \
     "*** Triclops Error '%s' at line %d :\n\t%s\n", \
     triclopsErrorToString( error ), \
     __LINE__, \
     description ); \
      printf( "Press any key to exit...\n" ); \
      getchar(); \
      exit( 1 ); \
   } \
} \

using namespace std;
using namespace cv;

void getfilename( const string src, string* p_str_path, string* p_str_name, string* p_str_noextname );

StereoVideo::StereoVideo()
{
    H.create( 3, 4, CV_32F );
    frame = -1;
}

StereoVideo::~StereoVideo()
{
}

bool StereoVideo::load_extrinsic_parameters()
{
    ifstream ifs;

    string strPath, strName, strNoextName;
    
    //if( flgVideoFile ) {
        getfilename( strVideoFile, &strPath, &strName, &strNoextName );
    //}

    ostringstream oss;
#ifdef LINUX_OS
    if( strPath == "" ) {
        oss << "/home/kumalab/project/HumanTracking/bin/";
    }
#endif
    oss << strPath << "Extrinsic" << camInfo.serialNumber << ".txt";
    ifs.open( oss.str().c_str() );

    if( !ifs ) {
        return false;
    }

    float a;
    for( int i = 0; i < 12; ++i ) {
        //ifs >> a;
        ifs >> H.at<float>( i / 4, i % 4 );
    }
    H.at<float>( 0, 3 ) = H.at<float>( 0, 3 ) / 1000.0f;
    H.at<float>( 1, 3 ) = H.at<float>( 1, 3 ) / 1000.0f;
    H.at<float>( 2, 3 ) = H.at<float>( 2, 3 ) / 1000.0f;

    return true;

    //H.at<float>( 0, 0 ) = 0.556880; H.at<float>( 0, 1 ) = -0.326309; H.at<float>( 0, 2 ) = 0.763811; H.at<float>( 0, 3 ) = -1.818401435;
    //H.at<float>( 1, 0 ) = -0.829754; H.at<float>( 1, 1 ) = -0.259884; H.at<float>( 1, 2 ) = 0.493932; H.at<float>( 1, 3 ) = -1.335204279;
    //H.at<float>( 2, 0 ) = 0.037327; H.at<float>( 2, 1 ) = -0.908836; H.at<float>( 2, 2 ) = -0.415480; H.at<float>( 2, 3 ) = 2.237207354;
}

bool StereoVideo::init( std::string strVideoFile )
{
    TriclopsError te;

    this->strVideoFile = strVideoFile;

    if( !video.open( strVideoFile ) ) {
        return false;
    }
    cout << "Opened a video file." << endl;

    string strPath, strName, strNoextName;
    getfilename( strVideoFile, &strPath, &strName, &strNoextName );
    camInfo.serialNumber = atoi( strNoextName.c_str() );

    // Show information
    size_t nFrames;
    cout << endl;
    cout << " Serial Number: " << camInfo.serialNumber << endl;
    cout << " # of frames: " << ( nFrames = (size_t)video.get( CV_CAP_PROP_FRAME_COUNT ) ) << endl;

    // Read time stamp information
    {
        ostringstream oss;
        oss << strPath << camInfo.serialNumber << ".txt";
        ifstream ifs( oss.str().c_str() );
        if( !ifs.is_open() ) {
            return false;
        }
        unsigned long long timestamp;
        string str;
        for( size_t i = 0; i < nFrames; ++i ) {
            ifs >> timestamp;
            frame_to_timestamp.push_back( timestamp );
        }
    }

    // Create a Triclops context from the cameras calibration file
    {
        ostringstream oss;
        oss << strPath << "calibration" << camInfo.serialNumber << ".txt";
        te = triclopsGetDefaultContextFromFile( &triclops, (char*)oss.str().c_str() );
        _HANDLE_TRICLOPS_ERROR( "triclopsGetDefaultContextFromFile()", te );

        cout << oss.str() << " Loaded." << endl;
    }



    return true;
}

bool StereoVideo::grab()
{
    if( !video.read( image ) ) {
        return false;
    }
    
    ++frame;

    return true;
}




