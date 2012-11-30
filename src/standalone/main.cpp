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
#include "track.h"
//#include "TrackingResultResources.h"
#include "ResultRenderer2.h"
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

const int stereo_width = 512, stereo_height = 384;

bool flgOutputTrackingProcessData2Files = true;
bool flgSegmentationComparisonMode = false;

//TrackingResultResources resTracking;
ResultRenderer2 resultRenderer;
/*
inline void copy( Mat& img_dst, int dst_x, int dst_y, Mat& img_src, int src_x, int src_y, int width, int height )
{
    //cout << "copy(): "
    //     << "width=" << width
    //     << ", height=" << height 
    //     << ", img_src.size().width=" << img_src.size().width
    //     << ", img_src.size().heigh=" << img_src.size().height
    //     << ", img_dst.size().width=" << img_dst.size().width
    //     << ", img_dst.size().height=" << img_dst.size().height
    //     << endl;

    if( img_src.size().width < width || img_src.size().height < height ) {
        return;
    }

    for( int x = 0; x < width; ++x ) {
	//cout << "x=" << x;
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
	//cout << " done." << endl;
    }
}
*/
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
    string strTrackingConfigFile = "tracking.cfg";
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
        } else if( strOpt == "--segmentation-comparison" ) {
            flgSegmentationComparisonMode = true;
        } else {
            serialNumber.push_back( atoi( argv[ i ]) );
        }
    }

    initialize_tracker();

    load_pepmap_config();
    load_track_parameters( strStereoVideoFilePath, strTrackingConfigFile );

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

        // Set Triclops parameter for stereo
        stereoVideo[ i ].SetStereoParameters( stereo_width, stereo_height );
        cout << "<Stereo Parameters Changed>" << endl;

        // Load a background image
        cout << endl;
        if( stereoVideo[ i ].load_background() ) {
            cout << "Background image loaded." << endl;
        } else {
            cout << "No background image loaded." << endl;
            exit( 1 );
        }
    }

    //
    // test code
#ifdef WINDOWS_OS
    const string strSilhouettePath = strStereoVideoFilePath + "result_silhouette\\";
#endif
#ifdef LINUX_OS
    const string strSilhouettePath = strStereoVideoFilePath + "result_silhouette/";
#endif
/*
    resTracking.init( strStereoVideoFilePath + "result.txt"
                    , strStereoVideoFilePath + "result_pepmap.avi"
                    , strStereoVideoFilePath + "result_camera.avi"
                    , strSilhouettePath );
    resTracking.SetDelayUpdate( 10 );
    resTracking.clear();
    resTracking.EnableViewWindow();
*/  
    //map<unsigned long long,string> result_cameravideo_filename;
    //for( int i = 0; i < serialNumber.size(); ++i ) {
    //    ostringstream oss, oss2;
    //    oss << strStereoVideoFilePath << "Segmentation_" << serialNumber[ i ] << ".avi"; 
    //    result_cameravideo_filename[ serialNumber[ i ] ] = oss.str();
    //}
    const int fps = 30;

    resultRenderer.init( strStereoVideoFilePath + "result_pepmap.avi", strStereoVideoFilePath + "result_pepmap_without_region.avi", strStereoVideoFilePath + "segmentation.avi", fps );

    Mat image_record( 960, 1280, CV_8UC3 );
    Mat image_depth_record( stereo_height * 2, stereo_width * 2, CV_8U );
    Mat image_depth_before_subtraction_record( stereo_height * 2, stereo_width * 2, CV_8U );
    Mat image_occupancy_record( (int)( scale_m2px * roi_height ) * 2, (int)( scale_m2px * roi_width ) * 2, CV_8U );
    Mat image_occupancy_record2( (int)( scale_m2px * roi_height ), (int)( scale_m2px * roi_width ), CV_8U );

    VideoWriter video_camera, video_depth, video_depth_without_subtraction, video_occupancy, video_occupancy2;
    {
        ostringstream oss;
        oss << strStereoVideoFilePath << "integrated_cameraview.avi";
        if( !video_camera.open( oss.str(), CV_FOURCC('X','V','I','D'), fps, Size( 1280, 960 ) ) ) {
            cerr << "Couldn't open " <<  oss.str() << "." <<  endl;
            exit( 1 );
        }
    }
    {
        ostringstream oss;
        oss << strStereoVideoFilePath << "integrated_depthmap.avi";
        if( !video_depth.open( oss.str(), CV_FOURCC('X','V','I','D'), fps, Size( image_depth_record.size().width, image_depth_record.size().height ) ) ) {
            cerr << "Couldn't open " <<  oss.str() << "." <<  endl;
            exit( 1 );
        }
    }
    {
        ostringstream oss;
        oss << strStereoVideoFilePath << "integrated_depthmap_without_subtraction.avi";
        if( !video_depth_without_subtraction.open( oss.str(), CV_FOURCC('X','V','I','D'), fps, Size( image_depth_before_subtraction_record.size().width, image_depth_before_subtraction_record.size().height ) ) ) {
            cerr << "Couldn't open " <<  oss.str() << "." <<  endl;
            exit( 1 );
        }
    }
    {
        ostringstream oss;
        oss << strStereoVideoFilePath << "integrated_occupancymap.avi";
        if( !video_occupancy.open( oss.str(), CV_FOURCC('X','V','I','D'), fps, Size( image_occupancy_record.size().width, image_occupancy_record.size().height ) ) ) {
            cerr << "Couldn't open " <<  oss.str() << "." <<  endl;
            exit( 1 );
        }
    }
    {
        ostringstream oss;
        oss << strStereoVideoFilePath << "integrated_occupancymap2.avi";
        if( !video_occupancy2.open( oss.str(), CV_FOURCC('X','V','I','D'), fps, Size( image_occupancy_record2.size().width, image_occupancy_record2.size().height ) ) ) {
            cerr << "Couldn't open " <<  oss.str() << "." <<  endl;
            exit( 1 );
        }
    }

    unsigned long long time = time_start;
    int frame = 0;

    for( int i = 0; i < stereoVideo.size(); ++i ) {
        if( !stereoVideo[ i ].grab() ) {
            cerr << "Error in grabbing an image from the video file: " 
                    << serialNumber[ i ] << ".avi" << endl;
            exit( 1 );
        }
    }

    map<unsigned long long,PEPMapInfoEx> sort_buffer;

    bool flgLoop = true;
    bool flgPlaying = false;
    for( ; ; ) {
        for( int i = 0; i < stereoVideo.size() && flgLoop; ++i ) {
            while( stereoVideo[ i ].get_timestamp() < time ) {
                bool ret;

                // Grab an image from a video file.
                ret = stereoVideo[ i ].grab();
                if( !ret ) {
                    cerr << "Error in grabbing an image from the video file: " 
                         << serialNumber[ i ] << ".avi" << endl;
                    flgLoop = false;
                    break;
                }

                if( flgPlaying ) {
                    // Create an occupancy map
                    cout << "create_pepmap()...";
                    stereoVideo[ i ].create_pepmap();
                    cout << "done." << endl;
                    image_occupancy_record2 = stereoVideo[ i ].image_occupancy.clone();

                    //resTracking.UpdateView();

                    // Sort occupancy maps in the buffer
                    unsigned long long timestamp = stereoVideo[ i ].get_timestamp();
                    if( flgCompatible ) {
                        timestamp /= 10;
                        timestamp -= DELTA_EPOCH_IN_MICROSECS; 
                    }

                    PEPMapInfoEx pepmap;
                    pepmap.serialNumber = serialNumber[ i ];
                    pepmap.timeStamp = timestamp;
                    pepmap.occupancy = stereoVideo[ i ].occupancy.clone();
                    sort_buffer[ timestamp ] = pepmap;

                    GeometryMapInfoEx geometry;
                    geometry.serialNumber = serialNumber[ i ];
                    geometry.timeStamp = timestamp;
                    geometry.width = stereo_width;
                    geometry.height = stereo_height;
                    geometry.geometry = stereoVideo[ i ].geometry.clone();

                    GeometryMapInfoEx silhouette;
                    silhouette.serialNumber = serialNumber[ i ];
                    silhouette.timeStamp = timestamp;
                    silhouette.width = stereo_width;
                    silhouette.height = stereo_height;
                    silhouette.geometry = stereoVideo[ i ].silhouette.clone();

                    CameraImageInfoEx camera;
                    camera.serialNumber = serialNumber[ i ];
                    camera.timeStamp = timestamp;
                    camera.width = stereo_width;
                    camera.height = stereo_height;
                    camera.image = stereoVideo[ i ].image_rectified.clone();

                    resultRenderer.AddPEPMapInfo( pepmap );
                    resultRenderer.AddCameraImageInfo( camera );
                    resultRenderer.AddGeometryMapInfo( geometry );
                    resultRenderer.AddSilhouetteMapInfo( silhouette );
                    

                    for( ; ; ) {
                        if( sort_buffer.size() < 2 ) {
                            break;
                        }

                        unsigned long long diff = sort_buffer.rbegin()->first - sort_buffer.begin()->first;
                        if( diff < 300000 ) {
                            break;
                        }
                                                
                        timestamp = sort_buffer.begin()->first;
                        pepmap = sort_buffer.begin()->second;
                        sort_buffer.erase( sort_buffer.begin() );

                        // Tracking
                        map< unsigned long long, map<int,Point2d> > result;
                        map<unsigned long long, multimap<int,Point2d> > ext_result;
                        if( track( &result, &ext_result, pepmap.occupancy, timestamp ) ) {
                            // Store result view resources
                            //resTracking.AddResultTrajectories( result, ext_result );
                            resultRenderer.AddResultTrajectories( result, ext_result );
                            resultRenderer.Render();
                        }
			            //cout << "AddPEPMapInfo()...";
                        //resTracking.AddPEPMapInfo( pepmap );
                        //resultRenderer.AddPEPMapInfo( pepmap );
			            //cout << "done." << endl;
                    }
                    
                }
            } 
        }

        if( flgLoop == false ) {
            break;
        }

        flgPlaying = true;
        //cout << "flgPlaying = true;" << endl;

        {
            //image_record = stereoVideo[ 0 ].image;
            copy( image_record, 0, 0, stereoVideo[ 0 ].image, 640, 0, 640, 480 );
            copy( image_record, 640, 0, stereoVideo[ 1 ].image, 640, 0, 640, 480 );
            copy( image_record, 0, 480, stereoVideo[ 2 ].image, 640, 0, 640, 480 );
            copy( image_record, 640, 480, stereoVideo[ 3 ].image, 640, 0, 640, 480 );
            //cout << "copy done." << endl;
            imshow( "Record", image_record );
	    //cout << "imshow( \"Record\", image_record );" << endl;
            video_camera.write( image_record );
        }

        //imshow( "Rectified", stereoVideo[ 0 ].image_rectified );

        {
            copy( image_depth_record, 0, 0, stereoVideo[ 0 ].image_depth, 0, 0, stereo_width, stereo_height );
            copy( image_depth_record, stereo_width, 0, stereoVideo[ 1 ].image_depth, 0, 0, stereo_width, stereo_height );
            copy( image_depth_record, 0, stereo_height, stereoVideo[ 2 ].image_depth, 0, 0, stereo_width, stereo_height );
            copy( image_depth_record, stereo_width, stereo_height, stereoVideo[ 3 ].image_depth, 0, 0, stereo_width, stereo_height );
            imshow( "Depth Map", image_depth_record );
            Mat tmp( image_depth_record.size(), CV_8UC3 );
            cvtColor( image_depth_record, tmp, CV_GRAY2BGR );
            video_depth.write( tmp/*image_depth_record*/ );
        }
        
        {
            copy( image_depth_before_subtraction_record, 0, 0, stereoVideo[ 0 ].image_depth_before_subtraction, 0, 0, stereo_width, stereo_height );
            copy( image_depth_before_subtraction_record, stereo_width, 0, stereoVideo[ 1 ].image_depth_before_subtraction, 0, 0, stereo_width, stereo_height );
            copy( image_depth_before_subtraction_record, 0, stereo_height, stereoVideo[ 2 ].image_depth_before_subtraction, 0, 0, stereo_width, stereo_height );
            copy( image_depth_before_subtraction_record, stereo_width, stereo_height, stereoVideo[ 3 ].image_depth_before_subtraction, 0, 0, stereo_width, stereo_height );
            //imshow( "Depth Map (Before Subtraction)", image_depth_before_subtraction_record );
            Mat tmp( image_depth_before_subtraction_record.size(), CV_8UC3 );
            cvtColor( image_depth_before_subtraction_record, tmp, CV_GRAY2BGR );
            video_depth_without_subtraction.write( tmp/*image_depth_before_subtraction_record*/ );
        }

	//cout << "imshow( \"Depth Map\", image_depth_record );" << endl;

        {
            const int width = (int)( scale_m2px * roi_width );
            const int height = (int)( scale_m2px * roi_height );
            copy( image_occupancy_record, 0, 0, stereoVideo[ 0 ].image_occupancy, 0, 0, width, height );
            copy( image_occupancy_record, width, 0, stereoVideo[ 1 ].image_occupancy, 0, 0, width, height );
            copy( image_occupancy_record, 0, height, stereoVideo[ 2 ].image_occupancy, 0, 0, width, height );
            copy( image_occupancy_record, width, height, stereoVideo[ 3 ].image_occupancy, 0, 0, width, height );
            imshow( "Occupancy map", image_occupancy_record );
            Mat tmp( image_occupancy_record.size(), CV_8UC3 );
            cvtColor( image_occupancy_record, tmp, CV_GRAY2BGR );
            video_occupancy.write( tmp/*image_occupancy_record*/ );

            tmp.create( image_occupancy_record2.size(), CV_8UC3 );
            cvtColor( image_occupancy_record2, tmp, CV_GRAY2BGR );
            video_occupancy2.write( tmp );

        }
	//cout << "imshow( \"Occupancy map\", image_occupancy_record );" << endl;

        // Exit when ESC is hit.
        char c = cvWaitKey( 10 );
        if ( c == 27 ) {
            break;
        }

        ++frame;
        time = time_start + ( flgCompatible ? ( ( 10000000ULL * (unsigned long long)frame ) / (unsigned long long)fps ) 
                                            : ( ( 1000000ULL  * (unsigned long long)frame ) / (unsigned long long)fps ) );
    }
/*
    while( resTracking.hasDataToDisplay() ) {
#ifdef WINDOWS_OS
	Sleep( 1000 );
#endif
#ifdef LINUX_OS     
        sleep( 1 );
#endif
    }

    resTracking.TerminateViewWindow();
*/
}
