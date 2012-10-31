#include <iostream>
#include <vector>
#include <deque>
#include <map>
#include <string>
#include <sstream>
#include <fstream>

#include "triclops.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "../humantracking.h"
#include "StereoVideo.h"

#ifdef LINUX_OS
#include <sys/time.h>
#endif

using namespace std;
using namespace cv;

string strStereoVideoFilePath;

bool flgCompatible = false;
bool flgPEPMapFile = true;

float roi_width, roi_height;
float roi_x, roi_y;
float scale_m2px, scale_m2px_silhouette;

inline void copy( Mat& img_dst, int dst_x, int dst_y, Mat& img_src, int src_x, int src_y, int width, int height )
{
    for( int x = 0; x < width; ++x ) {
        for( int y = 0; y < height; ++y ) {
            for ( int channel = 0; channel < img_dst.channels(); ++channel ) {
                *(unsigned char*)( img_dst.data + ( dst_y + y ) * img_dst.step + ( dst_x + x ) * img_dst.channels() + channel )
                    = *(unsigned char*)( img_src.data + ( src_y + y ) * img_src.step + ( src_x + x ) * img_src.channels() + channel );
                //*(unsigned char*)( image_record.data + y * image_record.step + x * image_record.channels() + 1 )
                //    = *(unsigned char*)( img_src.data + src_y * img_src.step + src_x * img_src.channels() + 1 );
                //*(unsigned char*)( image_record.data + y * image_record.step + x * image_record.channels() + 2 )
                //    = *(unsigned char*)( img_src.data + src_y * img_src.step + src_x * img_src.channels() + 2 );
            }
        }
    }
}

void getfilename( const string src, string* p_str_path, string* p_str_name, string* p_str_noextname )
{
    int idxExt = src.rfind( ".", src.npos );
#ifdef WINDOWS_OS
    int idxPath = src.rfind( "\\", src.npos );
#else
    int idxPath = src.rfind( "/", src.npos );
#endif
    *p_str_path = src.substr( 0, idxPath + 1 );
    *p_str_name = src.substr( idxPath + 1 );
    *p_str_noextname = src.substr( idxPath + 1, idxExt - idxPath - 1 );
}

bool load_pepmap_config()
{
    ifstream ifs;

    string strPath, strName, strNoextName;
    
    //if( flgPEPMapFile ) {
    //    getfilename( strPEPMapFile, &strPath, &strName, &strNoextName );
    //}

    ostringstream oss;
    oss << strStereoVideoFilePath << "pepmap.cfg";
    ifs.open( oss.str().c_str() );

    if( !ifs.is_open() ) {
        return false;
    }

    char buf[ 1000 ];
    vector<float> value;
    while( !ifs.eof() ) {
        ifs.getline( buf, sizeof( buf ) );
        string str( buf );
        if( str[ 0 ] == '#' ) {
            continue;
        }
        float v = atof( str.c_str() );
        value.push_back( v );
    }

    if( value.size() != 5 ) {
        return false;
    }

    roi_width = value[ 0 ];
    roi_height = value[ 1 ];
    roi_x = value[ 2 ];
    roi_y = value[ 3 ];
    scale_m2px = value[ 4 ];
    scale_m2px_silhouette = scale_m2px * 3.0;

    return true;
}

int main( int argc, char *argv[] )
{
    //map<unsigned long long,PEPMapInfo> pepmap;
    vector<unsigned int> serialNumber;
    //vector<ifstream> ifs;
    //vector<PEPMapInfo> pepmap;
    //vector<CameraImageInfo> cam_image;
    //vector<GeometryMapInfo> geometry;
    //string str;
    unsigned long long time_start = 0;

    for( int i = 1; i < argc; ++i ) {
        string strOpt = argv[ i ];
        if( strOpt == "-d" ) {
            strStereoVideoFilePath = string( argv[ ++i ] );
        } else if( strOpt == "-t" ) {
            istringstream iss( string( argv[ ++i ] ).c_str() );
            iss >> time_start;
        } else if( strOpt == "--compatible" ) {
            flgCompatible = true;
        } else {
            serialNumber.push_back( atoi( argv[ i ]) );
        }
    }

    load_pepmap_config();
    //load_track_parameters( strPath, strTrackingConfigFile );

    vector<StereoVideo> stereoVideo( serialNumber.size() );
    for( int i = 0;i < serialNumber.size(); ++i ) {
        ostringstream oss;
        oss << strStereoVideoFilePath << serialNumber[ i ] << ".avi";
        if( !stereoVideo[ i ].init( oss.str() ) ) {
            cout << "Failed in opening the video file: " << oss.str() << "." << endl;
            exit( 1 );
        }
        // Load extrinsic parameters
        if( !stereoVideo[ i ].load_extrinsic_parameters() ) {
            cout << "Failed in loading the extrinsic parameters." << endl;
            exit( 1 );
        }

        cout << "<Initialized>" << endl;
        cout << serialNumber[i] << endl;
    }

    //
    // test code
    VideoWriter video;
    const int fps = 30;
    ostringstream oss;
    oss << strStereoVideoFilePath << "integrated_cameraview.avi";
    if( !video.open( oss.str(), CV_FOURCC('X','V','I','D'), fps, Size( 1280, 960 ) ) ) {
        cerr << "Couldn't open " <<  oss.str() << "." <<  endl;
        exit( 1 );
    }

    Mat image_record( 960, 1280, CV_8UC3 );

    unsigned long long time = time_start;
    int frame = 0;

    for( int i = 0; i < stereoVideo.size(); ++i ) {
        if( !stereoVideo[ i ].grab() ) {
            cerr << "Error in grabbing an image from the video file: " 
                    << serialNumber[ i ] << ".avi" << endl;
            exit( 1 );
        }
    }

    for( ; ; ) {
        for( int i = 0; i < stereoVideo.size(); ++i ) {
            while( stereoVideo[ i ].get_timestamp() < time ) {
                if( !stereoVideo[ i ].grab() ) {
                    cerr << "Error in grabbing an image from the video file: " 
                         << serialNumber[ i ] << ".avi" << endl;
                    exit( 1 );
                }
            } 
        }
        
        //image_record = stereoVideo[ 0 ].image;
        copy( image_record, 0, 0, stereoVideo[ 0 ].image, 640, 0, 640, 480 );
        copy( image_record, 640, 0, stereoVideo[ 1 ].image, 640, 0, 640, 480 );
        copy( image_record, 0, 480, stereoVideo[ 2 ].image, 640, 0, 640, 480 );
        copy( image_record, 640, 480, stereoVideo[ 3 ].image, 640, 0, 640, 480 );


        imshow( "Record", image_record );
        // Exit when ESC is hit.
        char c = cvWaitKey( 1 );
        if ( c == 27 ) {
            break;
        }
        video.write( image_record );
        ++frame;
        time = time_start + ( flgCompatible ? ( ( 10000000ULL * (unsigned long long)frame ) / (unsigned long long)fps ) 
                                            : ( ( 1000000ULL  * (unsigned long long)frame ) / (unsigned long long)fps ) );
    }
}