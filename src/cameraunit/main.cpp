#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include "FlyCapture2.h"
#include "triclops.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

//#include "BlobResult.h"

#include "zlib.h"

#include "../humantracking.h"

#include "CalculationProcessLogger.h"

#ifdef WINDOWS_OS
#include <conio.h>
#include "FrameRateCounter.h"
#endif

#ifdef LINUX_OS
#include <sys/time.h>
//#include <sys/times.h>
//#include <termios.h>
//#include <unistd.h>
//#include <fcntl.h>
//#include <curses.h>
#endif

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
using namespace FlyCapture2;
using namespace cv;

Camera bumblebee;
CameraInfo camInfo;
TriclopsContext triclops;

bool flgWindow = true;
bool flgVideoFile = false;
string strVideoFile;
VideoCapture video;
vector<unsigned long long> frame_to_timestamp;

bool flgSavePEPMap = false;
bool flgSaveCamImage = false;
bool flgSaveGeometryMap = false;
bool flgSaveDisparityMap = false;
bool flgStdOutPEPMap = true;
bool flgStdOutCamImage = true;
bool flgStdOutGeometryMap = true;
bool flgStdOutDisparityMap = true;
bool flgCompatible = false;
bool flgNullPEPMap = false;

int deinterlace_mode = 0;

int iMaxCols = 1280, iMaxRows = 960;
int stereo_width = 512, stereo_height = 384;
//int stereo_width = 640, stereo_height = 480;
Mat img_background( stereo_height, stereo_width, CV_32F );
Mat img_background_cam( stereo_height, stereo_width, CV_32F );
Mat H( 3, 4, CV_32F );

volatile bool flgEscape;

float roi_width, roi_height;
float roi_x, roi_y;
float scale_m2px, scale_m2px_silhouette;

#ifdef LINUX_OS
/*
int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}
*/
#endif

#ifdef WINDOWS_OS
DWORD WINAPI KeyInThread( LPVOID p_param )
#else
void* KeyInThread( void* p_param )
#endif
{
    char buf[ 10 ];
    string str;

    for( ; ; ) {
        cin >> str;
        if( str == "q" ) {
            flgEscape = true;
            break;
        }
    }
#ifdef WINDOWS_OS
    return 1;
#endif
#ifdef LINUX_OS
    return NULL;
#endif
}

void PrintError( Error error )
{
    error.PrintErrorTrace();
}

void PrintCameraInfo()
{
    cout << endl;
    cout << " Serial Number: " << camInfo.serialNumber << endl;
    cout << " Camera model: " << camInfo.modelName << endl;
    cout << " Color/Mono: ";
    if( camInfo.isColorCamera ) {
        cout << "Color" << endl;
    } else {
        cout << "Mono" << endl;
    }
    cout << " Vendor name: " << camInfo.vendorName << endl;
    cout << " Sensor info: " << camInfo.sensorInfo << endl;
    cout << " Sensor Resolution: " << camInfo.sensorResolution << endl;
    cout << " Firmware Version: " << camInfo.firmwareVersion << endl;
    cout << " Firmware Build Time: " << camInfo.firmwareBuildTime << endl;
    cout << " Driver Name: " << camInfo.driverName << endl;
}

bool InitializeWithBumblebee()
{
    Error err;
    TriclopsError te;

    // Connect to a camera
    err = bumblebee.Connect();
    if( err != PGRERROR_OK ) {
        PrintError( err );
        return false;
    }
    cout << "Connected to a Camera." << endl;

    // Get and print the camera information
    err = bumblebee.GetCameraInfo( &camInfo );
    if ( err != PGRERROR_OK ) {
        PrintError( err );
        return false;
    }
    PrintCameraInfo();

    // Create a Triclops context from the cameras calibration file
    ostringstream oss;
#ifdef LINUX_OS
    oss << "/home/kumalab/project/HumanTracking/bin/";
#endif
    oss << "calibration" << camInfo.serialNumber << ".txt";
    te = triclopsGetDefaultContextFromFile( &triclops, (char*)oss.str().c_str() );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetDefaultContextFromFile()", te );

    cout << oss.str() << " Loaded." << endl;


    return true;
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

bool InitializeWithVideo()
{
    TriclopsError te;

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

bool SetBumblebeeParameters()
{
    //FlyCaptureError fe;

    cout << endl;
    cout << "Changing camera parameters..." << endl; 

    //// Set the bus speed as high as possible.
    //FlyCaptureBusSpeed bs1, bs2;
    //fe = flycaptureGetBusSpeed( flycapture, &bs1, &bs2 );
    //cout << endl;
    //cout << " Bus Speed(old): asyncBusSpeed=" << bs1 << ", isochBusSpeed=" << bs2 << endl;
    //fe = flycaptureSetBusSpeed( flycapture, FLYCAPTURE_S_FASTEST, FLYCAPTURE_S_FASTEST );
    //cout << " Bus Speed(new): asyncBusSpeed=" << bs1 << ", isochBusSpeed=" << bs2 << endl;
    //cout << " Make sure both speeds are no less than 2 (FLYCAPTURE_S400)." << endl;

    // Use the center and right cameras for stereo processing.
    Property prop;
    prop.type = PAN;
    bumblebee.GetProperty( &prop );
    cout << endl;
    cout << " PAN(old): " << "valueA=" << prop.valueA << ", valueB=" << prop.valueB << ", autoManualMode=" << prop.autoManualMode << endl;
    prop.valueA = 1;
    prop.valueB = 0;
    bumblebee.SetProperty( &prop );
    bumblebee.GetProperty( &prop );
    cout << " PAN(new): " << "valueA=" << prop.valueA << ", valueB=" << prop.valueB << ", autoManualMode=" << prop.autoManualMode << endl;
    cout << " Make sure valueA=1 (Use center and right cameras for stereo)." << endl;

    // Set FPS 12.
    prop.type = FRAME_RATE;
    bumblebee.GetProperty( &prop );
    cout << endl;
    cout << " FRAME_RATE(old): " << "valueA=" << prop.valueA << ", valueB=" << prop.valueB << ", autoManualMode=" << prop.autoManualMode << endl;
    //prop.valueA = 1;
    //prop.valueB = 0;
    //bumblebee.SetProperty( &prop );
    //bumblebee.GetProperty( &prop );
    //cout << " FRAME_RATE(new): " << "valueA=" << prop.valueA << ", valueB=" << prop.valueB << ", autoManualMode=" << prop.autoManualMode << endl;

    cout << endl << "Done." << endl;

    return true;
}

bool SetStereoParameters( int width, int height, bool flg_output_msg = true ) {
    TriclopsError te;

    //
    // Change stereo parameters
    cout << endl;
    cout << "Changing stereo parameters..." << endl; 
    
    // Set the base-line of the stereo camera narrow.
    TriclopsCameraConfiguration triclopscamconfig;
    te = triclopsGetCameraConfiguration( triclops, &triclopscamconfig );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetCameraConfiguration()", te );  
    if( flg_output_msg ) cout << endl;
    if( flg_output_msg ) cout << " Base-line setting(old): " << triclopscamconfig << endl;
    te = triclopsSetCameraConfiguration( triclops, TriCfg_2CAM_HORIZONTAL_NARROW );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetCameraConfiguration()", te );  
    te = triclopsGetCameraConfiguration( triclops, &triclopscamconfig );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetCameraConfiguration()", te );  
    if( flg_output_msg ) cout << " Base-line setting(new): " << triclopscamconfig << endl;
    if( flg_output_msg ) cout << " Make sure the value is 1 (TriCfg_2CAM_HORIZONTAL_NARROW)" << endl;

    // set rectified resolution to width x height 
    int nrows, ncols;
    te = triclopsGetResolution( triclops, &nrows, &ncols );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetResolution()", te );
    if( flg_output_msg ) cout << endl;
    if( flg_output_msg ) cout << " Resolution(old): " << ncols << " x " << nrows << endl;
    te = triclopsSetResolution( triclops, height, width );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetResolution()", te );
    te = triclopsGetResolution( triclops, &nrows, &ncols );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetResolution()", te );
    if( flg_output_msg ) cout << " Resolution(new): " << ncols << " x " << nrows << endl;

    // Set disparity range
    int minDisparity, maxDisparity;
    te = triclopsGetDisparity( triclops, &minDisparity, &maxDisparity );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetDisparity()", te );   
    if( flg_output_msg ) cout << endl;
    if( flg_output_msg ) cout << " Disparity range(old): " << minDisparity << " - " << maxDisparity << endl;
    te = triclopsSetDisparity( triclops, 5, 60 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparity()", te );   
    te = triclopsGetDisparity( triclops, &minDisparity, &maxDisparity );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetDisparity()", te );   
    if( flg_output_msg ) cout << " Disparity range(new): " << minDisparity << " - " << maxDisparity << endl;

    // Set the window size for stereo matching.
    int size;
    te = triclopsGetStereoMask( triclops, &size );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetStereoMask()", te );   
    if( flg_output_msg ) cout << endl;
    if( flg_output_msg ) cout << " Window size(old): " << size << endl;
    te = triclopsSetStereoMask( triclops, 11 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetStereoMask()", te );   
    te = triclopsGetStereoMask( triclops, &size );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetStereoMask()", te );   
    if( flg_output_msg ) cout << " Window size(new): " << size << endl;

    // Turn off texture validation
    TriclopsBool on;
    te = triclopsGetTextureValidation( triclops, &on );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetTextureValidation()", te );
    if( flg_output_msg ) cout << endl;
    if( flg_output_msg ) cout << " Texture Validation(old): " << on << endl;
    te = triclopsSetTextureValidation( triclops, 0 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetTextureValidation()", te );
    te = triclopsGetTextureValidation( triclops, &on );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetTextureValidation()", te );
    if( flg_output_msg ) cout << " Texture Validation(new): " << on << endl;

    // Turn off texture validation
    te = triclopsGetUniquenessValidation( triclops, &on );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetUniquenessValidation()", te );
    if( flg_output_msg ) cout << endl;
    if( flg_output_msg ) cout << " Uniqueness Validation(old): " << on << endl;
    te = triclopsSetUniquenessValidation( triclops, 0 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetUniquenessValidation()", te );
    te = triclopsGetUniquenessValidation( triclops, &on );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetUniquenessValidation()", te );
    if( flg_output_msg ) cout << " Uniqueness Validation(new): " << on << endl;

    // Turn on sub-pixel interpolation
    te = triclopsGetSubpixelInterpolation( triclops, &on );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetSubpixelInterpolation()", te );
    if( flg_output_msg ) cout << endl;
    if( flg_output_msg ) cout << " SubpixelInterpolation(old): " << on << endl;
    te = triclopsSetSubpixelInterpolation( triclops, 1 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", te );
    te = triclopsGetSubpixelInterpolation( triclops, &on );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetSubpixelInterpolation()", te );
    if( flg_output_msg ) cout << " SubpixelInterpolation(new): " << on << endl;


    if( flg_output_msg ) cout << endl;
    float diff;
    te = triclopsGetSurfaceValidation( triclops, &on );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetSurfaceValidation()", te );
    if( flg_output_msg ) cout << " Surfafe Validation(old): " << on << endl;
    te = triclopsGetSurfaceValidationDifference( triclops, &diff );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetSurfaceValidationDifference()", te );
    if( flg_output_msg ) cout << " Maximum disparity difference(old): " << diff << endl;
    te = triclopsGetSurfaceValidationSize( triclops, &size );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetSurfaceValidationSize()", te );
    if( flg_output_msg ) cout << " Surface validation size(old): " << size << endl;

    te = triclopsSetSurfaceValidation( triclops, 1 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetSurfaceValidation()", te );
    te = triclopsSetSurfaceValidationDifference( triclops, 2/*diff*/ );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetSurfaceValidationDifference()", te );
    te = triclopsSetSurfaceValidationSize( triclops, 1000/*size*/ );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetSurfaceValidationSize()", te );

    te = triclopsGetSurfaceValidation( triclops, &on );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetSurfaceValidation()", te );
    if( flg_output_msg ) cout << " Surfafe Validation(new): " << on << endl;
    te = triclopsGetSurfaceValidationDifference( triclops, &diff );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetSurfaceValidationDifference()", te );
    if( flg_output_msg ) cout << " Maximum disparity difference(new): " << diff << endl;
    te = triclopsGetSurfaceValidationSize( triclops, &size );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetSurfaceValidationSize()", te );
    if( flg_output_msg ) cout << " Surface validation size(new): " << size << endl;

    // With Linux, Set # of thread to one in order to avoid a segmentation 
    // falut at triclopsStereo(), which seems a bug in the triclopsSDK.
    if( flg_output_msg ) cout << endl;
    int maxThreadCount;
    te = triclopsGetMaxThreadCount( triclops, &maxThreadCount );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetMaxThreadCount()", te );
    if( flg_output_msg ) cout << " The maximum number of threads(old): " << maxThreadCount << endl;
#ifdef LINUX_OS
    te = triclopsSetMaxThreadCount( triclops, 1 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetMaxThreadCount()", te );
    te = triclopsGetMaxThreadCount( triclops, &maxThreadCount );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetMaxThreadCount()", te );
    if( flg_output_msg ) cout << " The maximum number of threads(new): " << maxThreadCount << endl;
#endif

    cout << endl << "Done." << endl;

    return true;
}

bool CloseBumblebee()
{
    Error err;

    err = bumblebee.Disconnect();
    if( err != PGRERROR_OK ) {
        PrintError( err );
        return false;
    }

    cout << endl << "Camera" << " closed." << endl;

    return true;
}

void PrintImageInformation( Image& image )
{
    cout << "# of cols = " << image.GetCols()
         << ", # of rows = " << image.GetRows()
         << ", # of bits/pixel = " << image.GetBitsPerPixel()
         << ", stride = " << image.GetStride()
         << ", data size = " << image.GetDataSize()
         << ", Pixel format = " << image.GetPixelFormat()// << "(" << PIXEL_FORMAT_RAW16 << ")"
         << ", BayerTileFormat = " << image.GetBayerTileFormat();// << "(" << GBRG << ")" 
}

void stereo( TriclopsImage16* pDst/*Mat* pDst*/, Mat* pDstRefImg, const Mat& src/*TriclopsInput& triclopsInput*/ )
{
    TriclopsError te;
    TriclopsInput triclopsInput;

    int imageCols = src.cols / 2;
    int imageRows = src.rows;
    int imageRowInc = src.elemSize1() * src.cols * src.channels();
    unsigned long timeStampSeconds = 0;//12880571209;//rawImage.GetTimeStamp().seconds;
    unsigned long timeStampMicroSeconds = 0;//9890000;//rawImage.GetTimeStamp().microSeconds;

    // Pointers to positions in the mono buffer that correspond to the beginning
    // of the red, green and blue sections
    unsigned char* redMono = NULL;
    unsigned char* greenMono = NULL;
    unsigned char* blueMono = NULL;

    redMono = src.data;//deinterlacedImage.GetData();
    greenMono = redMono + imageCols;
    blueMono = redMono + imageCols;

    // Use the row interleaved images to build up an RGB TriclopsInput.  
    // An RGB triclops input will contain the 3 raw images (1 from each camera).
    te = triclopsBuildRGBTriclopsInput(
        imageCols, 
        imageRows, 
        imageRowInc,  
        timeStampSeconds, 
        timeStampMicroSeconds, 
        redMono, 
        greenMono, 
        blueMono, 
        &triclopsInput );
    _HANDLE_TRICLOPS_ERROR( "triclopsBuildRGBTriclopsInput()", te );
            
    // Preprocessing the image
    te = triclopsRectify( triclops, &triclopsInput );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectify()", te );

    if( pDst ) {
        // Do stereo processing
        te = triclopsStereo( triclops );
        _HANDLE_TRICLOPS_ERROR( "triclopsStereo()", te );

        // Retrieve the interpolated depth image from the context
        te = triclopsGetImage16( triclops, 
                    TriImg16_DISPARITY, 
                    TriCam_REFERENCE, 
                    pDst );
        _HANDLE_TRICLOPS_ERROR( "triclopsGetImage16()", te );
    }

    if( pDstRefImg ) {
        TriclopsImage rectifiedImage;
        te = triclopsGetImage( triclops, TriImg_RECTIFIED, TriCam_REFERENCE, &rectifiedImage );
        _HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );

        pDstRefImg->create( rectifiedImage.nrows, rectifiedImage.ncols, CV_8U );
        memcpy( pDstRefImg->data, rectifiedImage.data, rectifiedImage.rowinc * rectifiedImage.nrows );
    }
}

void grab_from_bumblebee( Mat* pDst, unsigned long long* p_time_stamp = NULL )
{
    Error err;
    Image rawImage;
    unsigned char* buffer;

    err = bumblebee.RetrieveBuffer( &rawImage );
    if( err != PGRERROR_OK ) {
        PrintError( err );
        return;
    }

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
    _sec = tmpres / 1000000ULL;
    _usec = tmpres % 1000000ULL;
    cout << "Time Stamp: " << _sec << "." << _usec;
    if( p_time_stamp ) {
      *p_time_stamp = (unsigned long long)tmpres;
      cout << endl << " -> " << *p_time_stamp 
                << ", " << ctime( &_sec ) << endl;
        
    }
#endif
#ifdef LINUX_OS
    timeval tv;
    gettimeofday( &tv, NULL );
    cout << "Time Stamp: " << tv.tv_sec << "." << tv.tv_usec;
    if( p_time_stamp ) {
      *p_time_stamp = (unsigned long long)tv.tv_sec * 1000000ULL + (unsigned long long)tv.tv_usec;
      cout << endl << " -> " << *p_time_stamp 
                << ", " << ctime( &tv.tv_sec ) << endl;
        
    }
    cout << endl;
#endif

    pDst->create( iMaxRows, iMaxCols * 2, CV_8U );
    buffer = pDst->data;

    // de-interlace
    // ## Should be optimized for faster processing
    for( int x = 0; x < iMaxCols; ++x ) {
        for( int y = 0; y < iMaxRows; ++y ) {
            const unsigned char* data = rawImage.GetData();
	    if( deinterlace_mode == 0 ) {
	      
	      // left
	      buffer[ x + iMaxCols * 2 * y ] = data[ 2 * x + iMaxCols * 2 * y ];
	      // right
	      buffer[ iMaxCols + x + iMaxCols * 2 * y ] = data[ 2 * x + 1 + iMaxCols * 2 * y ];
	    } else {

              // left
              buffer[ x + iMaxCols * 2 * y ] = data[ 2 * x + 1 + iMaxCols * 2 * y ];
              // right
              buffer[ iMaxCols + x + iMaxCols * 2 * y ] = data[ 2 * x + iMaxCols * 2 * y ];
	    }
        }
    }
}

void grab_from_bumblebee_color( Mat* pDst, unsigned long long* p_time_stamp = NULL )
{
    Error err;
    Image rawImage;
    unsigned char* buffer;

    err = bumblebee.RetrieveBuffer( &rawImage );
    if( err != PGRERROR_OK ) {
        PrintError( err );
        return;
    }
    
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
    _sec = tmpres / 1000000ULL;
    _usec = tmpres % 1000000ULL;
    cout << "Time Stamp: " << _sec << "." << _usec;
    if( p_time_stamp ) {
      *p_time_stamp = (unsigned long long)tmpres;
      cout << endl << " -> " << *p_time_stamp 
                << ", " << ctime( &_sec ) << endl;
        
    }
#endif
#ifdef LINUX_OS
    timeval tv;
    gettimeofday( &tv, NULL );
    cout << "Time Stamp: " << tv.tv_sec << "." << tv.tv_usec;
    if( p_time_stamp ) {
      *p_time_stamp = (unsigned long long)tv.tv_sec * 1000000ULL + (unsigned long long)tv.tv_usec;
      cout << endl << " -> " << *p_time_stamp 
                << ", " << ctime( &tv.tv_sec ) << endl;
        
    }
    cout << endl;
#endif

    pDst->create( iMaxRows, iMaxCols * 2, CV_8UC4 );
    buffer = new unsigned char[ iMaxCols * 2 * iMaxRows ];
    Image deinterlacedImage( iMaxRows
                           , iMaxCols * 2
                           , iMaxCols * 2
                           , buffer
                           , iMaxCols * 2 * iMaxRows
                           , PIXEL_FORMAT_RAW8
                           , GBRG );
    Image convertedImage( pDst->data, 4 * iMaxCols * 2 * iMaxRows );

    // de-interlace
    // ## Should be optimized for faster processing
    for( int x = 0; x < iMaxCols; ++x ) {
        for( int y = 0; y < iMaxRows; ++y ) {
            const unsigned char* data = rawImage.GetData();
            if( deinterlace_mode == 0 ) {
	            // left
	            buffer[ x + iMaxCols * 2 * y ] = data[ 2 * x + iMaxCols * 2 * y ];
	            // right
	            buffer[ iMaxCols + x + iMaxCols * 2 * y ] = data[ 2 * x + 1 + iMaxCols * 2 * y ];
            } else {
                // left
                buffer[ x + iMaxCols * 2 * y ] = data[ 2 * x + 1 + iMaxCols * 2 * y ];
                // right
                buffer[ iMaxCols + x + iMaxCols * 2 * y ] = data[ 2 * x + iMaxCols * 2 * y ];
            }
        }
    }

    err = deinterlacedImage.Convert( PIXEL_FORMAT_BGRU, &convertedImage );
    if( err != PGRERROR_OK ) {
        PrintError( err );
        exit( 1 );
    }
}


bool grab_from_video( Mat* pDst )
{
    Mat tmp;
    if( !video.read( tmp ) ) {
        return false;
    }
    //video >> tmp;//imgFromVideo;
    //tmp.convertTo( imgFromVideo, CV_8U );
    //int a = tmp.channels();
    //Mat tmp2( 480, 1280, CV_8UC1 );
    //cvtColor( tmp, tmp2, CV_BGR2GRAY );
    //resize( tmp2, imgFromVideo, imgFromVideo.size() );
    //a = imgFromVideo.channels();

    pDst->create( tmp.size(), CV_8U );
    cvtColor( tmp, *pDst, CV_BGR2GRAY );

    //imshow( "video", imgFromVideo );
    //cvWaitKey( 0 );

    return true;
}

void execute( int start_frame = 0 )
{
    int width = stereo_width, height = stereo_height;
    Error err;

    CalculationProcessLogger logCalc;
    {
        string strPath, strName, strNoextName;
        getfilename( strVideoFile, &strPath, &strName, &strNoextName );
        ostringstream oss;
        oss << strPath << camInfo.serialNumber << ".log";
        logCalc.init( oss.str() );
    }
    

    if( !flgVideoFile ) {
        // Configure Format7
        Format7ImageSettings imageSettings;
        imageSettings.width = iMaxCols;
        imageSettings.height = iMaxRows;
        imageSettings.offsetX = 0;
        imageSettings.offsetY = 0;
        imageSettings.mode = MODE_3;
        imageSettings.pixelFormat= PIXEL_FORMAT_RAW16;
        err = bumblebee.SetFormat7Configuration( &imageSettings, 100.0f );
        if( err != PGRERROR_OK ) {
            PrintError( err );
            exit( 1 );
        }

        // Start capturing images
        err = bumblebee.StartCapture();
        if( err != PGRERROR_OK ) {
            PrintError( err );
            exit( 1 );
        }
    }

    ofstream ofs;
    if( flgSavePEPMap ) {
        string strPath, strName, strNoextName;
        getfilename( strVideoFile, &strPath, &strName, &strNoextName );
        ostringstream oss;
        oss << strPath << "pepmap" << camInfo.serialNumber << ".dat";
        ofs.open( oss.str().c_str() );
    }

    if( flgVideoFile ) {
        video.set( CV_CAP_PROP_POS_FRAMES, start_frame );
    }

#ifdef WINDOWS_OS
    FrameRateCounter framerate;
    framerate.SetFrameRate( 0.0 );
#endif

    const size_t len_compress_buf = (int)( roi_height * scale_m2px ) * (int)( roi_width * scale_m2px ) * 2;
    unsigned char* compress_buf = new unsigned char[ len_compress_buf ];
    const size_t len_compress_buf_geometry = width * height * 2;
    unsigned char* compress_buf_geometry = new unsigned char[ len_compress_buf_geometry ];
    
    Mat image;
    //Image rawImage;
    //unsigned char* buffer = new unsigned char[ iMaxCols * 2 * iMaxRows ];
    //Image deinterlacedImage( iMaxRows, iMaxCols * 2, iMaxCols * 2, buffer, iMaxRows * iMaxCols * 2, PIXEL_FORMAT_RAW8, GBRG );
    //Mat img( iMaxRows, iMaxCols * 2, CV_8UC4 );
    Mat img_display( height, width, CV_8U );
    Mat img_occupancy( (int)( scale_m2px * roi_height ), (int)( scale_m2px * roi_width ), CV_8U );
    //Mat img_display2( (int)( scale_m2px * roi_height ) * 10, (int)( scale_m2px * roi_width ) * 10, CV_8U );
    Mat img_display2( (int)( roi_height * 80 ), (int)( roi_width * 80 ), CV_8U );
    //Image convertedImage( img.data, 4 * iMaxCols * 2 * iMaxRows );//img.rows() * img.cols() * 3 );
    TriclopsImage16 depthImage16;
    Mat img_depth( height, width, CV_32F );
    Mat img_camera( height, width, CV_8U );

    unsigned long long timeStamp;

    flgEscape = false;
#ifdef WINDOWS_OS
    DWORD idThread;
    HANDLE hThread = CreateThread( NULL, 0, KeyInThread, NULL, NULL, &idThread );
#else
    pthread_t thread;
    pthread_create(&thread , NULL , KeyInThread , NULL);
#endif
    int frame = 0; // debug. # of frames should be obtained by video.get( CV_CAP_PROP_FRAMES ).
    for( ; ; ) {
        // Exit when 'q' and enter keys are hit.
        if( flgWindow ) {
            (void)cvWaitKey( 1 );
        }
        if( flgEscape ) {
            break;
        }

        logCalc.start();

        // Retrieve an image
        if( !flgVideoFile ) {
	        grab_from_bumblebee( &image, &timeStamp );
        } else {
            if( !grab_from_video( &image ) ) {
                break;
            }
            //int frame = video.get( CV_CAP_PROP_POS_FRAMES );
            timeStamp = frame_to_timestamp[ frame ];
            ++frame; // debug. should be removed later.
            if( flgCompatible ) {
                // The old version of capturing program stores time stamp using GetSystemTimeAsFileTime().
                // Conversion is required.
                timeStamp /= 10;
                timeStamp -= DELTA_EPOCH_IN_MICROSECS; 
            }
            cout << "Next frame # is " << frame << endl;
            
        }
#ifdef WINDOWS_OS
        framerate.NewFrame();
        cout << "Frame Rate: " << framerate.GetFrameRate() << endl;
#endif
        // Retrieve an image
        // debug code for showing convertedImage
        //{
        //    // Create a converted image
        //    err = deinterlacedImage.Convert( PIXEL_FORMAT_BGRU, &convertedImage );
        //    if( err != PGRERROR_OK ) {
        //        PrintError( err );
        //        exit( 1 );
        //    }  

        //    //cout << "Converted image: ";
        //    //PrintImageInformation( convertedImage );
        //    //cout << endl;

        //    memcpy( img.data, convertedImage.GetData(), convertedImage.GetDataSize() );
        //    resize( img, img_display2, img_display2.size(), 0, 0 );
        //    imshow( "image", img_display2 );
        //}

        logCalc.set_timestamp( timeStamp );

        logCalc.stereo_processing( CalculationProcessLogger::Start );

        // Stereo Processing
        clock_t t = clock();
        stereo( &depthImage16, &img_camera, image );
        cout << (double)( clock() - t ) / (double)CLOCKS_PER_SEC << "[sec]" << endl;
        cout << "done." << endl << flush;

        unsigned short disparity;
        float xx, yy, zz;
        
        // （OpenMPなどで高速化の余地あり）
        // zzの代入が、zzを表示する行を入れないとなぜか成功しない（真っ黒の画面が表示される）。
        // debugビルドではちゃんと表示されるので最適化の問題か？
        for( int x = 0; x < depthImage16.ncols; x++ ) {
            for( int y = 0; y < depthImage16.nrows; y++ ) {
                disparity = *(unsigned short*)((unsigned char*)depthImage16.data + depthImage16.rowinc * y + x * 2 );
                triclopsRCD16ToXYZ( triclops, y, x, disparity, &xx, &yy, &zz );
                if( disparity >= 0xff00 ) {
                    zz = 0.0f;
                    xx = 0.0f;
                    yy = 0.0f;
                }
                img_depth.at<float>( y, x ) = zz;
                ////img_display.at<unsigned char>( y, x ) = (unsigned char)( 25.0f * zz );
                if( x == 100 && y == 100 ) {
                    cout << zz << ", " << 25.0f * zz << ", " << (int)( (25.0f * img_depth.at<float>( y, x )/*zz*/) ) << endl;//", ";
                }
            }
        }

        logCalc.stereo_processing( CalculationProcessLogger::End );

        //TriclopsImage3d* pImage3d;
        //te = triclopsCreateImage3d( triclops, &pImage3d );
        //te = triclopsExtractImage3d( triclops, pImage3d );
        //for( int x = 0; x < pImage3d->ncols; ++x ) {
        //  for( int y = 0; y < pImage3d->nrows; ++y ) {
        //      img_depth.at<float>( y, x ) = pImage3d->points[ x + y * pImage3d->ncols ].point[ 2 ];
        //  }
        //}
        //triclopsDestroyImage3d( &pImage3d );


        //// test code
        //// blob detection and labling/filtering
        //CBlobResult blobs;

        //IplImage img_blob = img_depth;
        //blobs = CBlobResult( &img_blob, NULL, 0 );
        //cout << "nBlobs:" << blobs.GetNumBlobs() << endl;

        //Mat matimg( img_depth.size(), CV_8UC3 );
        //IplImage imgd = matimg;
        //for ( int i = 0; i < blobs.GetNumBlobs(); i++ ) {
        //    CBlob* currentBlob = blobs.GetBlob(i);
        //    currentBlob->FillBlob( &imgd, CV_RGB(255,0,0));
        //}
        //imshow( "Blob", matimg );

        logCalc.makingmaps( CalculationProcessLogger::Start );

        // 
        // Create an occupancy map with foreground data
        // （ベクトル計算で高速化の余地あり）
        Mat occupancy = Mat::zeros( (int)( roi_height * scale_m2px ), (int)( roi_width * scale_m2px ), CV_16U );
        Mat occupancy_2 = Mat::zeros( (int)( 3.0 * scale_m2px ), (int)( roi_height * scale_m2px ), CV_16U );
        Mat occupancy_3 = Mat::zeros( (int)( roi_height * scale_m2px ), (int)( roi_width * scale_m2px ), CV_16U );
        Mat geometry = Mat::zeros( height, width, CV_16U );
        Mat geometry_2 = Mat::zeros( height, width, CV_16U );
        Mat disparitymap = Mat::zeros( height, width, CV_16U );
        int row, col;
        if( !flgNullPEPMap ) {
            //vector<Point3f> point_foreground;
            Mat xvec( 4, 1, CV_32F );
            Mat point_planview( 3, 1, CV_32F );
            for( int x = 0; x < img_depth.cols; ++x ) {
                for( int y  = 0; y < img_depth.rows; ++y ) {
                    //if( abs( img_depth.at<float>( y, x ) - img_background.at<float>( y, x ) ) < 0.2f ) {
                    int a, b;
                    if( abs( ( a = (int)img_camera.at<unsigned char>( y, x ) ) - ( b = (int)img_background_cam.at<unsigned char>( y, x ) ) ) < 20
                        || abs( img_depth.at<float>( y, x ) - img_background.at<float>( y, x ) ) < 0.2f ) {
                        img_depth.at<float>( y, x ) = 0.0f;
                        geometry.at<unsigned short>( y, x ) = 0;
                        geometry_2.at<unsigned short>( y, x ) = 0;
                        disparitymap.at<unsigned short>( y, x ) = 0xff00;
                    } else {
                        disparity = *(unsigned short*)((unsigned char*)depthImage16.data + depthImage16.rowinc * y + x * 2 );
                        if( disparity < 0xff00 ) {
                            disparitymap.at<unsigned short>( y, x ) = disparity;
                            triclopsRCD16ToXYZ( triclops, y, x, disparity, &xx, &yy, &zz );
                            xvec.at<float>( 0, 0 ) = xx; xvec.at<float>( 1, 0 ) = yy; xvec.at<float>( 2, 0 ) = zz; xvec.at<float>( 3, 0 ) = 1.0;
                            point_planview =  H * xvec ;
                            //point_foreground.push_back( Point3f( point_planview.at<float>( 0, 0 ), point_planview.at<float>( 1, 0 ), point_planview.at<float>( 2, 0 ) ) );
                            float pv_x = point_planview.at<float>( 0, 0 ), pv_y = point_planview.at<float>( 1, 0 ), pv_z = point_planview.at<float>( 2, 0 );
                            row = (int)( scale_m2px * ( ( pv_x - roi_x ) + roi_height / 2.0f ) ); // X axis is projected on the vertical axis of the occupancy map.
                            col = (int)( scale_m2px * ( ( pv_y - roi_y ) + roi_width / 2.0f ) ); // Y axis is projected on the horizontal axis of the occupancy map.
                            if( row >= 0 && row < occupancy.rows && col >= 0 && col < occupancy.cols /*&& pv_z < 2.0*/ ) {
                                occupancy.at<unsigned short>( row, col ) = occupancy.at<unsigned short>( row, col ) + 1;
                            }
                            geometry.at<unsigned short>( y, x ) = row * occupancy.cols + col + 1;
                            
                            //col = row; // X axis is projected on the horizontal axis of the geometry map2.
                            //row = (int)( occupancy_2.rows - scale_m2px * pv_z ); // Z axis is projeted on the vertical axis of the geometry map2.
                            col = (int)( scale_m2px_silhouette * ( ( pv_x - roi_x ) + roi_height / 2.0f ) ); // X axis is projected on the horizontal axis of the geometry map2.
                            row = (int)( ( 3.0 * scale_m2px_silhouette ) - scale_m2px_silhouette * pv_z ); // Z axis is projeted on the vertical axis of the geometry map2.
                            if( row >= 0 && row < ( 3.0 * scale_m2px_silhouette ) && col >= 0 && col < ( scale_m2px_silhouette * roi_height ) ) {
                                //occupancy_2.at<unsigned short>( row, col ) = occupancy_2.at<unsigned short>( row, col ) + 1;
                                geometry_2.at<unsigned short>( y, x ) = row * ( scale_m2px_silhouette * roi_height ) + col + 1;
                            } else {
                                geometry_2.at<unsigned short>( y, x ) = 0;
                            }

                            //col = (int)( scale_m2px * ( ( pv_x - roi_x ) + roi_width / 2.0f ) );
                            //if( row >= 0 && row < occupancy_3.rows && col >= 0 && col < occupancy_3.cols ) {
                            //    occupancy_3.at<unsigned short>( row, col ) = occupancy_3.at<unsigned short>( row, col ) + 1;
                            //}

                        } else {
                            img_depth.at<float>( y, x ) = 0.0f;
                            geometry.at<unsigned short>( y, x ) = 0;
                            geometry_2.at<unsigned short>( y, x ) = 0;
                            disparitymap.at<unsigned short>( y, x ) = 0xff00;
                        }
                    }
                }
            }

            //Mat occupancy = Mat::zeros( (int)( roi_height * scale_m2px ), (int)( roi_width * scale_m2px ), CV_16U );
            //for( vector<Point3f>::iterator it = point_foreground.begin(); it != point_foreground.end(); ++it ) {
            //    int row = (int)( scale_m2px * ( ( it->x - roi_x ) + roi_width / 2.0f ) ), col = (int)( scale_m2px * ( ( it->y - roi_y ) + roi_width / 2.0f ) );
            //    if( row >= 0 && row < occupancy.rows && col >= 0 && col < occupancy.cols ) {
            //        occupancy.at<unsigned short>( row, col ) = occupancy.at<unsigned short>( row, col ) + 1;
            //    }
            //}

            for( int row = 0; row < occupancy.rows; ++row ) {
                for( int col = 0; col < occupancy.cols; ++col ) {
                    if( occupancy.at<unsigned short>( row, col ) < /*10*/50 ) {
                        occupancy.at<unsigned short>( row, col ) = 0;
                    }
                }
            }
        }

        logCalc.makingmaps( CalculationProcessLogger::End );

        logCalc.send_occupancy( CalculationProcessLogger::Start );

        // Compress occupancy map
        if( flgStdOutPEPMap || flgSavePEPMap ) {
            unsigned long long t_start_compress;
            t_start_compress = getTimeStamp();
            uLongf len_compressed = len_compress_buf;
            compress( compress_buf
                    , &len_compressed
                    , occupancy.data
                    , len_compress_buf );
            cout << "Compressed the occupancy map: size=" << len_compress_buf << " -> " << len_compressed << "[bytes] (" << getTimeStamp() - t_start_compress << "us)" << endl;

            
            // test code for checking compression validity, where the occupancy maps are restored with the compressed data.
            //uLongf len_uncompressed = len_compress_buf;
            //uncompress( occupancy.data
            //            , &len_uncompressed
            //            , compress_buf
            //            , len_compressed );
            //cout << "(Test Code!)Unompressed the occupancy map: size=" << len_compressed << " -> " << len_uncompressed << "[bytes]" << endl;

	        // Send PEPMap data to stdout
            if( flgStdOutPEPMap ) {
	            cout << "<PEPMap>" << endl // Header
                     << camInfo.serialNumber << endl // Serial Number
                     << timeStamp << endl // Time stamp             
                     << len_compressed << endl; // PEPMap data length
	            for( size_t i = 0; i < len_compressed; ++i ) { // PEPMap data
	              cout << hex << setw(2) << setfill( '0' ) << (int)compress_buf[ i ];
	            }
	            cout << dec << endl;
            }

            // Save PEPMap data to file
            if( flgSavePEPMap ) {
                ofs << "<PEPMap>" << endl // Header
                    << camInfo.serialNumber << endl // Serial Number
                    << timeStamp << endl // Time stamp
                    << len_compressed << endl; // PEPMap data length
	            for( size_t i = 0; i < len_compressed; ++i ) { // PEPMap data
	                ofs << hex << setw(2) << setfill( '0' ) << (int)compress_buf[ i ];
	            }
	            ofs << dec << endl;
            }
        }

        logCalc.send_occupancy( CalculationProcessLogger::End );

        logCalc.send_disparity( CalculationProcessLogger::Start );

        // Send Disparity map
        if( flgSaveDisparityMap || flgStdOutDisparityMap ) {
            unsigned long long t_start_compress;
            // Compress geometry map
            t_start_compress = getTimeStamp();
            uLongf len_compressed_depthmap = len_compress_buf_geometry;
            compress( compress_buf_geometry
                    , &len_compressed_depthmap
                    , disparitymap.data
                    , len_compress_buf_geometry );
            cout << "Compressed the depth map: size=" << len_compress_buf_geometry << " -> " << len_compressed_depthmap << "[bytes] (" << getTimeStamp() - t_start_compress << "us)" << endl;            

	        // Send depth map to stdout
            if( flgStdOutGeometryMap ) {
	            cout << "<DisparityMap>" << endl // Header
                     << camInfo.serialNumber << endl // Serial Number
                     << timeStamp << endl // Time stamp             
                     << disparitymap.cols << endl // Width
                     << disparitymap.rows << endl // Height
                     << len_compressed_depthmap << endl; // Disparity map data length
	            for( size_t i = 0; i < len_compressed_depthmap; ++i ) { // depthImage16 map data
	                cout << hex << setw(2) << setfill( '0' ) << (int)compress_buf_geometry[ i ];
	            }
	            cout << dec << endl;
            }

            // Save depth map to the file
            if( flgSavePEPMap ) {
	            ofs << "<DisparityMap>" << endl // Header
                     << camInfo.serialNumber << endl // Serial Number
                     << timeStamp << endl // Time stamp             
                     << disparitymap.cols << endl // Width
                     << disparitymap.rows << endl // Height
                     << len_compressed_depthmap << endl; // Disparity map data length
	            for( size_t i = 0; i < len_compressed_depthmap; ++i ) { // depthImage16 map data
	                ofs << hex << setw(2) << setfill( '0' ) << (int)compress_buf_geometry[ i ];
                    //ofs << (char)compress_buf_geometry[ i ];
	            }
	            ofs << dec << endl;
            }
        }
        logCalc.send_disparity( CalculationProcessLogger::End );

        logCalc.send_camimage( CalculationProcessLogger::Start );
       
        if( flgStdOutCamImage || flgSaveCamImage ) {
	        // Send grabbed image to stdout

            // JPEG compression of the grabbed image
            unsigned long long t_start_compress;
            t_start_compress = getTimeStamp();
            vector<uchar> buff;//buffer for coding
            vector<int> param = vector<int>(2);
            param[0]=CV_IMWRITE_JPEG_QUALITY;
            param[1]=70;//default(95) 0-100

            //Mat imgCamRight = image( Range::all(), Range(1, image.cols / 2 + 1 ) );
            //resize( imgCamRight, img_camera, img_camera.size() );
            //imshow( "Camera", img_camera );

 
            imencode(".jpg",img_camera,buff,param);
            //cout<<"coded file size(jpg)"<<buff.size()<<endl;//fit buff size automatically.
            //Mat jpegimage = imdecode(Mat(buff),CV_LOAD_IMAGE_COLOR);        
            cout << "Compressed the camera image: size=" << buff.size() << "[bytes] (" << getTimeStamp() - t_start_compress << "us)" << endl;

            if( flgStdOutCamImage ) {
                cout << "<CameraImage>" << endl // Header
                     << camInfo.serialNumber << endl // Serial Number
                     << timeStamp << endl // Time stamp
                     << img_camera.cols << endl // Width
                     << img_camera.rows << endl // Height
                     << buff.size() << endl; // data length
	            for( size_t i = 0; i < buff.size(); ++i ) { // PEPMap data
	              cout << hex << setw(2) << setfill( '0' ) << (int)buff[ i ];
	            }
	            cout << dec << endl;
            }

            if( flgSaveCamImage ) {
                ofs << "<CameraImage>" << endl // Header
                     << camInfo.serialNumber << endl // Serial Number
                     << timeStamp << endl // Time stamp
                     << img_camera.cols << endl // Width
                     << img_camera.rows << endl // Height
                     << buff.size() << endl; // data length
	            for( size_t i = 0; i < buff.size(); ++i ) { // PEPMap data
	                ofs << hex << setw(2) << setfill( '0' ) << (int)buff[ i ];
                    //ofs << (char)buff[ i ];
	            }
	            ofs << dec << endl;
            }
        }

        logCalc.send_camimage( CalculationProcessLogger::End );

        logCalc.send_geometry( CalculationProcessLogger::Start );

        if( flgStdOutGeometryMap || flgSaveGeometryMap ) {
            //
            // Send Geometry data to stdout
            // Compress geometry map
            unsigned long long t_start_compress;
            t_start_compress = getTimeStamp();
            uLongf len_compressed_geometry = len_compress_buf_geometry;
            compress( compress_buf_geometry
                    , &len_compressed_geometry
                    , geometry.data
                    , len_compress_buf_geometry );
            cout << "Compressed the geometry map: size=" << len_compress_buf_geometry << " -> " << len_compressed_geometry << "[bytes] (" << getTimeStamp() - t_start_compress << "us)" << endl;

	        // Send Geometry map to stdout
            if( flgStdOutGeometryMap ) {
	            cout << "<Geometry>" << endl // Header
                        << camInfo.serialNumber << endl // Serial Number
                        << timeStamp << endl // Time stamp             
                        << geometry.cols << endl // Width
                        << geometry.rows << endl // Height
                        << len_compressed_geometry << endl; // PEPMap data length
	            for( size_t i = 0; i < len_compressed_geometry; ++i ) { // Geometry map data
	                cout << hex << setw(2) << setfill( '0' ) << (int)compress_buf_geometry[ i ];
	            }
	            cout << dec << endl;
            }

            // Save Geometry map to the file
            if( flgSaveGeometryMap ) {
	            ofs << "<Geometry>" << endl // Header
                        << camInfo.serialNumber << endl // Serial Number
                        << timeStamp << endl // Time stamp             
                        << geometry.cols << endl // Width
                        << geometry.rows << endl // Height
                        << len_compressed_geometry << endl; // PEPMap data length
	            for( size_t i = 0; i < len_compressed_geometry; ++i ) { // Geometry map data
	                ofs << hex << setw(2) << setfill( '0' ) << (int)compress_buf_geometry[ i ];
                    //ofs << (char)compress_buf_geometry[ i ];
	            }
	            ofs << dec << endl;
            }
        }

        logCalc.send_geometry( CalculationProcessLogger::End );

        logCalc.send_geometry2( CalculationProcessLogger::Start );

        if( flgStdOutGeometryMap || flgSaveGeometryMap ) {
            //
            // Send Geometry data 2 (for silhouette) to stdout
            // Compress geometry map
            unsigned long long t_start_compress;
            t_start_compress = getTimeStamp();
            uLongf len_compressed_geometry_2 = len_compress_buf_geometry;
            compress( compress_buf_geometry
                    , &len_compressed_geometry_2
                    , geometry_2.data
                    , len_compress_buf_geometry );
            cout << "Compressed the geometry_2 map: size=" << len_compress_buf_geometry << " -> " << len_compressed_geometry_2 << "[bytes] (" << getTimeStamp() - t_start_compress << "us)" << endl;

	        // Send geometry_2 map to stdout
            if( flgStdOutGeometryMap ) {
	            cout << "<Geometry2>" << endl // Header
                        << camInfo.serialNumber << endl // Serial Number
                        << timeStamp << endl // Time stamp             
                        << geometry_2.cols << endl // Width
                        << geometry_2.rows << endl // Height
                        << len_compressed_geometry_2 << endl; // PEPMap data length
	            for( size_t i = 0; i < len_compressed_geometry_2; ++i ) { // geometry_2 map data
	                cout << hex << setw(2) << setfill( '0' ) << (int)compress_buf_geometry[ i ];
	            }
	            cout << dec << endl;
            }

            // Save geometry_2 map to the file
            if( flgSaveGeometryMap ) {
	            ofs << "<Geometry2>" << endl // Header
                        << camInfo.serialNumber << endl // Serial Number
                        << timeStamp << endl // Time stamp             
                        << geometry_2.cols << endl // Width
                        << geometry_2.rows << endl // Height
                        << len_compressed_geometry_2 << endl; // PEPMap data length
	            for( size_t i = 0; i < len_compressed_geometry_2; ++i ) { // geometry_2 map data
	                ofs << hex << setw(2) << setfill( '0' ) << (int)compress_buf_geometry[ i ];
                    //ofs << (char)compress_buf_geometry[ i ];
	            }
	            ofs << dec << endl;
            }
        }

        logCalc.send_geometry2( CalculationProcessLogger::End );

        if( flgWindow ) {
            occupancy.convertTo( img_occupancy, CV_8U );
            resize( img_occupancy, img_display2, img_display2.size() );
            imshow( "Occupancy Map", img_display2 );

            //occupancy_2.convertTo( img_occupancy, CV_8U );
            //resize( img_occupancy, img_display2, img_display2.size() );
            //imshow( "Occupancy Map 2", img_display2 );

            //occupancy_3.convertTo( img_occupancy, CV_8U );
            //resize( img_occupancy, img_display2, img_display2.size() );
            //imshow( "Occupancy Map 3", img_display2 );

            imshow( "Camera", img_camera );
        }

        //{
        //    int hbins = 255, sbins = 255;
        //    int histSize[] = { hbins, sbins };
        //    float hranges[] = { 0, 255 };
        //    float sranges[] = { 0, 255 };
        //    const float* ranges[] = { hranges, sranges };
        //    MatND hist;
        //    int channels [] = { 0 };
        //    calcHist( &img_occupancy, 1, channels, Mat(), hist, 2, histSize, ranges, true, false );

        //    double maxVal=0;
        //    minMaxLoc(hist, 0, &maxVal, 0, 0);

        //    int scale = 10;
        //    Mat histImg = Mat::zeros(sbins*scale, hbins*10, CV_8UC3);

        //    for( int h = 0; h < hbins; h++ ) {
        //        for( int s = 0; s < sbins; s++ )
        //        {
        //            float binVal = hist.at<float>(h, s);
        //            int intensity = cvRound(binVal*255/maxVal);
        //            rectangle( histImg, Point(h*scale, s*scale),
        //                        Point( (h+1)*scale - 1, (s+1)*scale - 1),
        //                        Scalar::all(intensity),
        //                        CV_FILLED );
        //        }
        //     }

        //     imshow( "histogram", histImg );
        //}

        logCalc.end_and_output2file();

        if( flgWindow ) {
            img_depth.convertTo( img_display, CV_8U, 25.0, 0.0 );

            imshow( "Disparity", img_display );
        }
    }

    if( !flgVideoFile ) {
        err = bumblebee.StopCapture();
    }

    destroyWindow( "Disparity" );
    destroyWindow( "Occupancy Map" );
    destroyWindow( "Occupancy Map 2" );
    destroyWindow( "Occupancy Map 3" );
#ifdef WINDOWS_OS
    WaitForSingleObject( hThread, INFINITE );
#else
    pthread_join(thread , NULL);
#endif

    //delete [] buffer;
    delete [] compress_buf;
    delete [] compress_buf_geometry;
}

bool load_extrinsic_parameters()
{
    ifstream ifs;

    string strPath, strName, strNoextName;
    
    if( flgVideoFile ) {
        getfilename( strVideoFile, &strPath, &strName, &strNoextName );
    }

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

bool load_pepmap_config()
{
    ifstream ifs;

    string strPath, strName, strNoextName;
    
    if( flgVideoFile ) {
        getfilename( strVideoFile, &strPath, &strName, &strNoextName );
    }

    ostringstream oss;
#ifdef LINUX_OS
    if( strPath == "" ) {
        oss << "/home/kumalab/project/HumanTracking/bin/";
    }
#endif
    oss << strPath << "pepmap.cfg";

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

bool load_background()
{
    ifstream ifs;
    string strPath, strName, strNoextName;
    
    if( flgVideoFile ) {
        getfilename( strVideoFile, &strPath, &strName, &strNoextName );
    }

    ostringstream oss;
#ifdef LINUX_OS
    if( strPath == "" ) {
        strPath = "/home/kumalab/project/HumanTracking/bin/";
    }
#endif
    oss << strPath << "background" << camInfo.serialNumber << ".dat";
    ifs.open( oss.str().c_str(), ios::in | ios::binary );

    if( !ifs ) {
        return false;
    }

    int rowinc = (int)( img_background.step1() * img_background.elemSize1() );
    ifs.read( (char*)img_background.data, img_background.rows * rowinc ); 

    ifs.close();

    oss.str( "" );
    oss.clear();
#ifdef LINUX_OS
    if( strPath == "" ) {
        strPath = "/home/kumalab/project/HumanTracking/bin/";
    }
#endif
    oss << strPath << "background" << camInfo.serialNumber << ".bmp";
    Mat _img_background_cam = imread( oss.str().c_str() );
    if( _img_background_cam.channels() == 3 ) {
        cvtColor( _img_background_cam, img_background_cam, CV_BGR2GRAY );
    }
    if( img_background_cam.data == NULL ) {
        return false;
    }

    return true;
}

bool save_background()
{
    ofstream ofs;
    string strPath, strName, strNoextName;

    if( flgVideoFile ) {
        getfilename( strVideoFile, &strPath, &strName, &strNoextName );
    }

    ostringstream oss;
#ifdef LINUX_OS
    if( strPath == "" ) {
        oss << "/home/kumalab/project/HumanTracking/bin/";
    }
#endif
    oss << strPath << "background" << camInfo.serialNumber << ".dat";
    string tmp = oss.str();

    ofs.open( oss.str().c_str(), ios::out | ios::binary | ios::trunc );
    if( !ofs ) {
        cout << "Error occured in saving the background depth image." << endl;
        return false;
    }

    int rowinc = (int)( img_background.step1() * img_background.elemSize1() );
    ofs.write( (char*)img_background.data, img_background.rows * rowinc ); 

    ofs.close();

    oss.str( "" );
    oss.clear();
#ifdef LINUX_OS
    if( strPath == "" ) {
        strPath = "/home/kumalab/project/HumanTracking/bin/";
    }
#endif
    oss << strPath << "background" << camInfo.serialNumber << ".bmp";
    if( !imwrite( oss.str().c_str(), img_background_cam ) ) {
        cout << "Error occured in saving the background image." << endl;
        return false;
    }

    return true;
}

void show_background()
{
    if( flgWindow ) {
        Mat img_display( img_background.size(), CV_8U );
        img_background.convertTo( img_display, CV_8U, 25.0, 0.0 );
        imshow( "background depth image", img_display );  
        imshow( "background image", img_background_cam );

        while( 1 ) {
            // Exit when ESC is hit.
            char c = cvWaitKey( 10 );
            if ( c == 27 ) {
                break;
            }
        }

        destroyWindow( "background image" );
        destroyWindow( "background depth image" );
    }
}

void update_background( int nFrame, int start_frame = 0 )
{
    int width = stereo_width, height = stereo_height;
    Error err;

    if( !flgVideoFile ) {
        // Configure Format7
        Format7ImageSettings imageSettings;
        imageSettings.width = iMaxCols;
        imageSettings.height = iMaxRows;
        imageSettings.offsetX = 0;
        imageSettings.offsetY = 0;
        imageSettings.mode = MODE_3;
        imageSettings.pixelFormat= PIXEL_FORMAT_RAW16;
        err = bumblebee.SetFormat7Configuration( &imageSettings, 100.0f );
        if( err != PGRERROR_OK ) {
            PrintError( err );
            exit( 1 );
        }

        // Start capturing images
        err = bumblebee.StartCapture();
        if( err != PGRERROR_OK ) {
            PrintError( err );
            exit( 1 );
        }
    }

    if( flgVideoFile ) {
        video.set( CV_CAP_PROP_POS_FRAMES, start_frame );
    }

    //Image rawImage;
    //Mat imgFromVideo;
    //unsigned char* buffer = new unsigned char[ iMaxCols * 2 * iMaxRows ];
    //Image deinterlacedImage( iMaxRows, iMaxCols * 2, iMaxCols * 2, buffer, iMaxRows * iMaxCols * 2, PIXEL_FORMAT_RAW8, GBRG );
    //Mat img_display( height, width, CV_8U );
    TriclopsImage16 depthImage16;
    Mat cnt( height, width, CV_8U );
    int framecnt = 0;
    //TriclopsInput triclopsInput;

    Mat image;

    for( int x = 0; x < img_background.cols; ++x ) {
        for( int y  = 0; y < img_background.rows; ++y ) {
            img_background.at<float>( y, x ) = 0.0;
            cnt.at<unsigned char>( y, x ) = 0;
        }
    }

    for( int frame = 0; frame < nFrame; ++frame ) {

        cout << "# of frames: " << frame + 1 << endl;

        // Retrieve an image
        if( !flgVideoFile ) {
            grab_from_bumblebee( &image );
        } else {
            grab_from_video( &image );
        }


        // Stereo Processing
        clock_t t = clock();
        stereo( &depthImage16, &img_background_cam, image/*triclopsInput*/ );
        cout << (double)( clock() - t ) / (double)CLOCKS_PER_SEC << "[sec]" << endl;
        cout << "done." << endl << flush;

        unsigned short disparity;
        float xx, yy, zz;
        
        // （OpenMPなどで高速化の余地あり）
        // zzの代入が、zzを表示する行を入れないとなぜか成功しない（真っ黒の画面が表示される）。
        // debugビルドではちゃんと表示されるので最適化の問題か？
        for( int x = 0; x < depthImage16.ncols; x++ ) {
            for( int y = 0; y < depthImage16.nrows; y++ ) {
                disparity = *(unsigned short*)((unsigned char*)depthImage16.data + depthImage16.rowinc * y + x * 2 );
                triclopsRCD16ToXYZ( triclops, y, x, disparity, &xx, &yy, &zz );
                if( disparity >= 0xff00 ) {
                    zz = 0.0f;
                    xx = 0.0f;
                    yy = 0.0f;
                } else {
                    cnt.at<unsigned char>( y, x ) = cnt.at<unsigned char>( y, x ) + 1;
                }
                img_background.at<float>( y, x ) += zz;
                if( x == 100 && y == 100 ) {
                    cout << zz << ", " << 25.0f * zz << ", " << (int)( (25.0f * zz) ) << endl;//", ";
                }
            }
        }
    }

    if( !flgVideoFile ) {
        err = bumblebee.StopCapture();
    }


    for( int x = 0; x < img_background.cols; ++x ) {
        for( int y  = 0; y < img_background.rows; ++y ) {
            unsigned char k = cnt.at<unsigned char>( y, x );
            if( k > 0 ) {
                img_background.at<float>( y, x ) = img_background.at<float>( y, x ) / (float)k;
            }
        }
    }
    
    // Save the background image
    if( save_background() ) {
        cout << "Background image saved." << endl;
    } else {
        cout << "Saving background image failed." << endl;
    }

    // Show the background image
    show_background();

    //cout << "Next frame # is " << video.get( CV_CAP_PROP_POS_FRAMES ) << endl;

    //delete [] buffer;
}

void capture( Mat* pDst = NULL)
{
    Error err;

    // Configure Format7
    Format7ImageSettings imageSettings;
    imageSettings.width = iMaxCols;
    imageSettings.height = iMaxRows;
    imageSettings.offsetX = 0;
    imageSettings.offsetY = 0;
    imageSettings.mode = MODE_3;
    imageSettings.pixelFormat= PIXEL_FORMAT_RAW16;
    err = bumblebee.SetFormat7Configuration( &imageSettings, 100.0f );
    if( err != PGRERROR_OK ) {
        PrintError( err );
        exit( 1 );
    }

    // Start capturing images
    err = bumblebee.StartCapture();
    if( err != PGRERROR_OK ) {
        PrintError( err );
        exit( 1 );
    }

    int width, height;

    //SetBumblebeeParameters( width = iMaxCols, height = iMaxRows );
    SetStereoParameters( width = iMaxCols, height = iMaxRows );

    Image rawImage;
    unsigned char* buffer = new unsigned char[ iMaxCols * 2 * iMaxRows ];
    Image deinterlacedImage( iMaxRows, iMaxCols * 2, iMaxCols * 2, buffer, iMaxRows * iMaxCols * 2, PIXEL_FORMAT_RAW8, GBRG );
    Mat img_display( height, width, CV_8U );
    Mat img_display2( height, width, CV_8U );
    //Image convertedImage( img.data, 4 * iMaxCols * 2 * iMaxRows );//img.rows() * img.cols() * 3 );
    TriclopsImage16 depthImage16;
    Mat img_depth( height, width, CV_32F );

    // Retrieve an image
    err = bumblebee.RetrieveBuffer( &rawImage );
    if( err != PGRERROR_OK ) {
        PrintError( err );
        exit( 1 );
    }

    //cout << "Raw image: ";
    //PrintImageInformation( rawImage );
    //cout << endl;

    // de-interlace
    // ## Should be optimized for faster processing
    for( int x = 0; x < iMaxCols; ++x ) {
        for( int y = 0; y < iMaxRows; ++y ) {
            const unsigned char* data = rawImage.GetData();
/*
            // left
            buffer[ x + iMaxCols * 2 * y ] = data[ 2 * x + iMaxCols * 2 * y ];
            // right
            buffer[ iMaxCols + x + iMaxCols * 2 * y ] = data[ 2 * x + 1 + iMaxCols * 2 * y ];
*/

            // left
            buffer[ x + iMaxCols * 2 * y ] = data[ 2 * x + 1 + iMaxCols * 2 * y ];
            // right
            buffer[ iMaxCols + x + iMaxCols * 2 * y ] = data[ 2 * x + iMaxCols * 2 * y ];

        }
    }

    // debug code for showing convertedImage
    //{
    //    // Create a converted image
    //    err = deinterlacedImage.Convert( PIXEL_FORMAT_BGRU, &convertedImage );
    //    if( err != PGRERROR_OK ) {
    //        PrintError( err );
    //        exit( 1 );
    //    }  

    //    //cout << "Converted image: ";
    //    //PrintImageInformation( convertedImage );
    //    //cout << endl;

    //    memcpy( img.data, convertedImage.GetData(), convertedImage.GetDataSize() );
    //    resize( img, img_display2, img_display2.size(), 0, 0 );
    //    imshow( "image", img_display2 );
    //}

    // Stereo Processing
    TriclopsError te;
    TriclopsInput triclopsInput;

    int imageCols = iMaxCols;
    int imageRows = iMaxRows;
    int imageRowInc = rawImage.GetStride();
    unsigned long timeStampSeconds = rawImage.GetTimeStamp().seconds;
    unsigned long timeStampMicroSeconds = rawImage.GetTimeStamp().microSeconds;

    // Pointers to positions in the mono buffer that correspond to the beginning
    // of the red, green and blue sections
    unsigned char* redMono = NULL;
    unsigned char* greenMono = NULL;
    unsigned char* blueMono = NULL;

    redMono = deinterlacedImage.GetData();
    greenMono = redMono + imageCols;
    blueMono = redMono + imageCols;

    // Use the row interleaved images to build up an RGB TriclopsInput.  
    // An RGB triclops input will contain the 3 raw images (1 from each camera).
    te = triclopsBuildRGBTriclopsInput(
        imageCols, 
        imageRows, 
        imageRowInc,  
        timeStampSeconds, 
        timeStampMicroSeconds, 
        redMono, 
        greenMono, 
        blueMono, 
        &triclopsInput );
    _HANDLE_TRICLOPS_ERROR( "triclopsBuildRGBTriclopsInput()", te );


    // Preprocessing the image
    te = triclopsRectify( triclops, &triclopsInput );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectify()", te );

    // Stop Capturing
    err = bumblebee.StopCapture();


    TriclopsImage rectifiedImage;
    te = triclopsGetImage( triclops, TriImg_RECTIFIED, TriCam_REFERENCE, &rectifiedImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );
    memcpy( img_display2.data, rectifiedImage.data, rectifiedImage.rowinc * rectifiedImage.nrows );

    if( pDst ) {
        *pDst = img_display2;
    }

    imwrite( "image.png", img_display2 );

    if( flgWindow ) {
        imshow( "image", img_display2 );
        while( true ) {
            // Exit when ESC is hit.
            char c = cvWaitKey( 1 );
            if ( c == 27 ) {
                break;
            }
        }
        destroyWindow( "image" );
    }

    //SetBumblebeeParameters( stereo_width, stereo_height );
    SetStereoParameters( stereo_width, stereo_height );

    delete [] buffer;
}

void record( int width, int height )
{
    Error err;

    if( flgVideoFile ) {
        return;
    }

    string strPath, strName, strNoextName;
    getfilename( strVideoFile, &strPath, &strName, &strNoextName );
    ostringstream oss;
    oss << strPath << camInfo.serialNumber;

    VideoWriter video;
    if( !video.open( oss.str() + ".avi", CV_FOURCC('X','V','I','D'), 10, Size( width, height ) ) ) {
        cerr << "Couldn't open " <<  oss.str() << "." <<  endl;
        exit( 1 );
    }

    oss << ".txt";
    ofstream ofs( oss.str().c_str() );

    // Configure Format7
    Format7ImageSettings imageSettings;
    imageSettings.width = iMaxCols;
    imageSettings.height = iMaxRows;
    imageSettings.offsetX = 0;
    imageSettings.offsetY = 0;
    imageSettings.mode = MODE_3;
    imageSettings.pixelFormat= PIXEL_FORMAT_RAW16;
    err = bumblebee.SetFormat7Configuration( &imageSettings, 100.0f );
    if( err != PGRERROR_OK ) {
        PrintError( err );
        exit( 1 );
    }

    // Start capturing images
    err = bumblebee.StartCapture();
    if( err != PGRERROR_OK ) {
        PrintError( err );
        exit( 1 );
    }

    Mat image, image_tmp, image_record;
    unsigned long long timeStamp;

    flgEscape = false;
#ifdef WINDOWS_OS
    DWORD idThread;
    HANDLE hThread = CreateThread( NULL, 0, KeyInThread, NULL, NULL, &idThread );
#else
    pthread_t thread;
    pthread_create(&thread , NULL , KeyInThread , NULL);
#endif
    for( ; ; ) {
        if( flgWindow ) {
            (void)cvWaitKey( 1 );
        }
        if( flgEscape ) {
            break;
        }

        // Retrieve an image
	    grab_from_bumblebee_color( &image, &timeStamp );

        image_tmp.create( image.rows, image.cols, CV_8UC3 );
        image_record.create( height, width, CV_8UC3 );
        cvtColor( image, image_tmp, CV_BGRA2BGR );
        resize( image_tmp, image_record, image_record.size() );
        video.write( image_record );
        ofs << timeStamp << endl;
        if( flgWindow ) {
            imshow( "Camera", image_record );
        }
    }

    err = bumblebee.StopCapture();
    destroyWindow( "Camera" );

#ifdef WINDOWS_OS
    WaitForSingleObject( hThread, INFINITE );
#else
    pthread_join(thread , NULL);
#endif
}

void calibrate()
{
    int width = stereo_width, height = stereo_height;
    Error err;

    if( !flgVideoFile ) {
        // Configure Format7
        Format7ImageSettings imageSettings;
        imageSettings.width = iMaxCols;
        imageSettings.height = iMaxRows;
        imageSettings.offsetX = 0;
        imageSettings.offsetY = 0;
        imageSettings.mode = MODE_3;
        imageSettings.pixelFormat= PIXEL_FORMAT_RAW16;
        err = bumblebee.SetFormat7Configuration( &imageSettings, 100.0f );
        if( err != PGRERROR_OK ) {
            PrintError( err );
            exit( 1 );
        }

        // Start capturing images
        err = bumblebee.StartCapture();
        if( err != PGRERROR_OK ) {
            PrintError( err );
            exit( 1 );
        }
    }

    vector<Point3f> corners3d_cam; // 3D positions of the corners (Camera Coordinate)
    vector<Point3f> corners3d_w; // 3D positions of the corners (World Coordinate)

    Mat image;
    TriclopsImage16 depthImage16;
    Mat img_camera( height, width, CV_8U );
    int stage = 0;
    const int width_pattern = 4, height_pattern = 7;

    for( ; stage < 2; ) {
        cout << "Set the calibration pattern at ";
        if( stage == 0 ) {
            cout << "the lower plane";
        } else {
            cout << "the higher plane";
        }
        cout << ", and press any key." << endl;
        //while( cvWaitKey( 1 ) == -1 );
	string str_tmp;
	cin >> str_tmp;

        // Retrieve an image
        if( !flgVideoFile ) {
	        grab_from_bumblebee( &image );
        } else {
            if( !grab_from_video( &image ) ) {
                cerr << "Error occured in grabbing an image from the video file. Abort." << endl;
                return;
            }
        }

	cout << "  Image retrieved." << endl;

        // Stereo Processing
        clock_t t = clock();
        stereo( &depthImage16, &img_camera, image );
	//imshow( "Camera", img_camera );

        Size patternsize( width_pattern, height_pattern );
        vector<Point2f> corners; // 2D positions of the detected corners
        vector<Point3f> corners3d;  // 3D positions of the detected corners
        unsigned short disparity;
        float xx, yy, zz;

        if( findChessboardCorners( img_camera, patternsize, corners ) ) {
            bool flgAllCornersAvailable = true;
            for( vector<Point2f>::iterator it = corners.begin(); it != corners.end(); ++it ) {
                cout << "  " << it->x << ", " << it->y;
                disparity = *(unsigned short*)((unsigned char*)depthImage16.data + depthImage16.rowinc * (int)(it->y) + (int)(it->x) * 2 );
                triclopsRCD16ToXYZ( triclops, (int)it->y, (int)it->x, disparity, &xx, &yy, &zz );
                if( disparity >= 0xff00 ) {
                    zz = 0.0f;
                    xx = 0.0f;
                    yy = 0.0f;
                    flgAllCornersAvailable = false;
                }

                corners3d.push_back( Point3f( xx, yy, zz ) );
                cout << " -> " << xx << ", " << yy << ", " << zz << endl;
            }

            if( !flgAllCornersAvailable ) {
                cout << "No depth information measured at (a) corner(s). Try again." << endl;
                continue;
            }
        } else {
            cout << "No corners detected. Try again." << endl;
            continue;
        }

        // Corner detection succeeded.
	cout << endl;
        corners3d_cam.insert( corners3d_cam.end(), corners3d.begin(), corners3d.end() );
        for( int i = 0; i < width_pattern * height_pattern; ++i ) {
            float ref_x, ref_y, ref_z;
            ref_x = 0.135f * (float)( height_pattern - 1 ) - (float)( i / width_pattern ) * 0.135f;
            ref_y = 0.135f * (float)( width_pattern - 1 ) - (float)( i % width_pattern ) * 0.135f;
            ref_z = ( stage == 0 ) ? 0.75f : 0.75f + 0.725; // Measurement required      
            corners3d_w.push_back( Point3f( ref_x, ref_y, ref_z ) );
	    cout << "  " << ref_x << ", " << ref_y << ", " << ref_z << endl;
        }
        drawChessboardCorners( img_camera, patternsize, Mat( corners ), true );
	imshow( "Corners", img_camera );
	cvWaitKey( 1000 );
        ++stage;
    }

    cout << endl;

    // Calculate extrinsic parameters
    Mat A( 3 * corners3d_w.size(), 12, CV_32F );
    Mat b( 3 * corners3d_w.size(), 1, CV_32F );
    Mat p;

    for( int i = 0; i < corners3d_w.size(); ++i ) {
        const int row = 3 * i;
        A.at<float>( row, 0 ) = A.at<float>( row + 1, 3 ) = A.at<float>( row + 2, 6 ) = corners3d_cam[ i ].x;
        A.at<float>( row, 1 ) = A.at<float>( row + 1, 4 ) = A.at<float>( row + 2, 7 ) = corners3d_cam[ i ].y;
        A.at<float>( row, 2 ) = A.at<float>( row + 1, 5 ) = A.at<float>( row + 2, 8 ) = corners3d_cam[ i ].z;
        A.at<float>( row, 9 ) = A.at<float>( row + 1, 10 ) = A.at<float>( row + 2, 11 ) = 1.0f;

        b.at<float>( row, 0 ) = corners3d_w[ i ].x;
        b.at<float>( row + 1, 0 ) = corners3d_w[ i ].y;
        b.at<float>( row + 2, 0 ) = corners3d_w[ i ].z;
    }

    solve( A, b, p, DECOMP_SVD );

    H.create( 3, 4, CV_32F );

    ofstream ofs;
    string strPath, strName, strNoextName;
    getfilename( strVideoFile, &strPath, &strName, &strNoextName );
    ostringstream oss;
    oss << strPath << "Extrinsic" << camInfo.serialNumber << ".txt";
    ofs.open( oss.str().c_str() );
    ofs << ( H.at<float>( 0, 0 ) = p.at<float>( 0,  0 ) ) << " "
        << ( H.at<float>( 0, 1 ) = p.at<float>( 1,  0 ) ) << " "
        << ( H.at<float>( 0, 2 ) = p.at<float>( 2,  0 ) ) << " "
        << ( H.at<float>( 0, 3 ) = p.at<float>( 9,  0 ) ) * 1.0e3 << endl
        << ( H.at<float>( 1, 0 ) = p.at<float>( 3,  0 ) ) << " "
        << ( H.at<float>( 1, 1 ) = p.at<float>( 4,  0 ) ) << " "
        << ( H.at<float>( 1, 2 ) = p.at<float>( 5,  0 ) ) << " "
        << ( H.at<float>( 1, 3 ) = p.at<float>( 10, 0 ) ) * 1.0e3 << endl
        << ( H.at<float>( 2, 0 ) = p.at<float>( 6,  0 ) ) << " "
        << ( H.at<float>( 2, 1 ) = p.at<float>( 7,  0 ) ) << " "
        << ( H.at<float>( 2, 2 ) = p.at<float>( 8,  0 ) ) << " "
        << ( H.at<float>( 2, 3 ) = p.at<float>( 11, 0 ) ) * 1.0e3 << endl;
    
    ofs.close();

}

void findcorners( int width, int height )
{
    Error err;
    string strPath, strName, strNoextName;

    if( flgVideoFile ) {
        getfilename( strVideoFile, &strPath, &strName, &strNoextName );
    }

    ostringstream oss;
#ifdef LINUX_OS
    if( strPath == "" ) {
        oss << "/home/kumalab/project/HumanTracking/bin/";
    }
#endif
    oss << strPath << "corners" << camInfo.serialNumber << ".txt";
    ofstream ofs( oss.str().c_str() );

    if( !ofs.is_open() ) {
        cerr << "Error occured in creating " << oss.str() << ".";
        exit( 1 );
    }


    // Configure Format7
    Format7ImageSettings imageSettings;
    imageSettings.width = iMaxCols;
    imageSettings.height = iMaxRows;
    imageSettings.offsetX = 0;
    imageSettings.offsetY = 0;
    imageSettings.mode = MODE_3;
    imageSettings.pixelFormat= PIXEL_FORMAT_RAW16;
    err = bumblebee.SetFormat7Configuration( &imageSettings, 100.0f );
    if( err != PGRERROR_OK ) {
        PrintError( err );
        exit( 1 );
    }

    // Start capturing images
    err = bumblebee.StartCapture();
    if( err != PGRERROR_OK ) {
        PrintError( err );
        exit( 1 );
    }

    Mat image;
    TriclopsImage16 depthImage16;
    Mat img_camera( iMaxRows, iMaxCols, CV_8U );
    Mat img_save( height, width, CV_8UC3 );
    int cnt = 0;
    const int width_pattern = 4, height_pattern = 7;
    int flags = CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE;

    for( ; ; ) {
        cout << "# " << cnt << ". 'test' to check a camera image, 'end' to exit the command. 'f0', 'f1' and 'f2' to change the flags. Any other string to execute." << endl;
        string strtmp;
        cin >> strtmp;
        bool flgTest = false;
        if( strtmp == "end" ) {
            break;
        } else if( strtmp == "test" ) {
            flgTest = true;
        } else if( strtmp == "f0" ) {
            flags = 0;
        } else if( strtmp == "f1" ) {
            flags = CV_CALIB_CB_ADAPTIVE_THRESH;
        } else if( strtmp == "f2" ) {
            flags = CV_CALIB_CB_NORMALIZE_IMAGE;
        }

        // Retrieve an image
	    grab_from_bumblebee( &image );

        // Stereo Processing
        clock_t t = clock();
        
        SetStereoParameters( iMaxCols, iMaxRows, false );
        stereo( NULL, &img_camera, image );

        SetStereoParameters( width, height, false );
        stereo( &depthImage16, NULL, image );

        Size patternsize( width_pattern, height_pattern );
        vector<Point2f> corners; // 2D positions of the detected corners
        vector<Point3f> corners3d;  // 3D positions of the detected corners
        unsigned short disparity;
        float xx, yy, zz;

        if( findChessboardCorners( img_camera, patternsize, corners, flags ) ) {
            bool flgAllCornersAvailable = true;
            for( vector<Point2f>::iterator it = corners.begin(); it != corners.end(); ++it ) {
                it->x *= (float)width / (float)iMaxCols;
                it->y *= (float)height / (float)iMaxRows;
                //cout << "  " << it->x << ", " << it->y;
                disparity = *(unsigned short*)((unsigned char*)depthImage16.data + depthImage16.rowinc * (int)(it->y) + (int)(it->x) * 2 );
                triclopsRCD16ToXYZ( triclops, (int)it->y, (int)it->x, disparity, &xx, &yy, &zz );
                if( disparity >= 0xff00 ) {
                    zz = 0.0f;
                    xx = 0.0f;
                    yy = 0.0f;
                    flgAllCornersAvailable = false;
                }

                corners3d.push_back( Point3f( xx, yy, zz ) );
                //cout << " -> " << xx << ", " << yy << ", " << zz;
            }

            if( !flgAllCornersAvailable ) {
                cout << "No depth information measured at (a) corner(s). Try again." << endl;
                Mat img_camera_small( height, width, CV_8U );
                resize( img_camera, img_camera_small, img_camera_small.size() ); 
                imshow( "Image", img_camera_small );
                cvWaitKey( 300 );
                continue;
            }
        } else {
            cout << "No corners detected. Try again." << endl;
            Mat img_camera_small( height, width, CV_8U );
            resize( img_camera, img_camera_small, img_camera_small.size() ); 
            imshow( "Image", img_camera_small );
            cvWaitKey( 300 );
            continue;
        }

        // Corner detection succeeded.
        for( int i = 0; i < width_pattern * height_pattern; ++i ) {
            if( !flgTest ) {
                ofs << corners[ i ].x << ", "
                    << corners[ i ].y << ", "
                    << corners3d[ i ].x << ", "
                    << corners3d[ i ].y << ", "
                    << corners3d[ i ].z << endl;
            }
            cout << corners[ i ].x << ", "
                 << corners[ i ].y << ", "
                 << corners3d[ i ].x << ", "
                 << corners3d[ i ].y << ", "
                 << corners3d[ i ].z << endl;
        }
        ofs << endl;
        cout << endl;

        Mat img_camera_small( height, width, CV_8U );
        resize( img_camera, img_camera_small, img_camera_small.size() ); 
        cvtColor( img_camera_small, img_save, CV_GRAY2BGR );
        drawChessboardCorners( img_save, patternsize, Mat( corners ), true );
        ostringstream oss;
        oss << strPath << "corners" << camInfo.serialNumber << "_" << cnt << ".png";
	if( !flgTest ) {
	    imwrite( oss.str(), img_save );
	}
        imshow( "Detected Corners", img_save );
        cvWaitKey( 500 );
        
	if( !flgTest ) {
            ++cnt;
	}
    }

    destroyWindow( "Image" );
    destroyWindow( "Detected Corners" );
    
    err = bumblebee.StopCapture();

    SetStereoParameters( stereo_width, stereo_height, false );
}

int main( int argc, char *argv[] )
{
    bool ret;

    // Option
    for( int i = 0; i < argc; ++i ) {
        string strOpt = argv[ i ];
        if( strOpt == "--nowindow" ) {
            cout << "No window mode." << endl;
            flgWindow = false;
        } else if( strOpt == "-f" ) {
            flgVideoFile = true;
            strVideoFile = string( argv[ ++i ] );
        } else if( strOpt == "--save-pepmap" ) {
            flgSavePEPMap = true;
        } else if( strOpt == "--save-camimage" ) {
            flgSaveCamImage = true;
        } else if( strOpt == "--save-geometry" ) {
            flgSaveGeometryMap = true;
        } else if( strOpt == "--save-disparity" ) {
            flgSaveDisparityMap = true;
        } else if( strOpt == "--no-stdout-pepmap" ) {
            flgStdOutPEPMap = false;
        } else if( strOpt == "--no-stdout-camimage" ) {
            flgStdOutCamImage = false;
        } else if( strOpt == "--no-stdout-geometry" ) {
            flgStdOutGeometryMap = false;
        } else if( strOpt == "--no-stdout-disparity" ) {
            flgStdOutDisparityMap = false;
        } else if( strOpt == "--null-pepmap" ) {
            flgNullPEPMap = true;
        } else if( strOpt == "--compatible" ) {
            flgCompatible = true;
        }
    }

    if( !flgVideoFile ) {
        // Initialize Bumblebee
        ret = InitializeWithBumblebee();
        if( ret == false ) {
            cout << "Camera initialization failed." << endl;
            exit( 1 );
        }
    } else {
        // Open the video file
        ret = InitializeWithVideo();
        if( ret == false ) {
            cout << "Failed in opening the video file." << endl;
            exit( 1 );
        }

    }
    // Load extrinsic parameters
    if( !flgNullPEPMap ) {
        if( !load_extrinsic_parameters() ) {
            cout << "Failed in loading the extrinsic parameters." << endl;
            exit( 1 );
        }
    } else {
        H.at<float>( 0, 0 ) = 1.0; H.at<float>( 0, 1 ) = 0.0; H.at<float>( 0, 2 ) = 0.0; H.at<float>( 0, 3 ) = 0.0;
        H.at<float>( 1, 0 ) = 0.0; H.at<float>( 1, 1 ) = 1.0; H.at<float>( 1, 2 ) = 0.0; H.at<float>( 1, 3 ) = 0.0;
        H.at<float>( 2, 0 ) = 0.0; H.at<float>( 2, 1 ) = 0.0; H.at<float>( 2, 2 ) = 1.0; H.at<float>( 2, 3 ) = 0.0;
    }
    cout << "<Initialized>" << endl;
    cout << camInfo.serialNumber << endl;

    if( !load_pepmap_config() ) {
        cout << "Failed in loading the PEP-map config file." << endl;
        exit( 1 );
    }

    {
      ostringstream oss;
#ifdef LINUX_OS
      oss << "/home/kumalab/project/HumanTracking/bin/";
#endif
      oss << "deinterlace1";

      ifstream ifs;
      ifs.open( oss.str().c_str() );
      if( ifs.is_open() ) {
	deinterlace_mode = 1;
      }
      cout << "Deinterlace mode " << deinterlace_mode << endl;    
    }

    if( !flgVideoFile ) {
        // Set bumblebee parameters
        SetBumblebeeParameters();
        cout << "<Camera Parameters Changed>" << endl;
    }

    // Set Triclops parameter for stereo
    SetStereoParameters( stereo_width, stereo_height );
    cout << "<Stereo Parameters Changed>" << endl;

    // Load a background image
    cout << endl;
    if( load_background() ) {
        cout << "Background image loaded." << endl;
    } else {
        cout << "No background image loaded." << endl;
    }

    //
    // Command Prompt
    vector<string> strCmd;
    string str;
    while( 1 ) {
        cout << ">";
        //cin >> str;
        getline( cin, str );
        if( str.empty() ) {
            continue;
        }

        strCmd.clear();
        char* cstrcmd = new char[ str.size() + 1 ];
        strcpy( cstrcmd, str.c_str() );
        
        char* tp = strtok( cstrcmd, " " );
        strCmd.push_back( string( tp ) );
        while ( tp != NULL ) {
            tp = strtok( NULL, " " );
            if ( tp != NULL ) {
                strCmd.push_back( string( tp ) );
            }
        }

        delete [] cstrcmd;
        cstrcmd = NULL;

        if( strCmd[ 0 ] == "run" ) {
            execute( strCmd.size() == 2 ? atoi( strCmd[ 1 ].c_str() ) : 0 );
        //} else if( strCmd[ 0 ] == "update" ) {
        //    update_background( 20 );
        //} else if( strCmd[ 0 ] == "background" ) {
        //    show_background();
        } else if( strCmd[ 0 ] == "calibrate" ) {
            calibrate();
        } else if( strCmd[ 0 ] == "capture" ) {
            capture();
        } else if( strCmd[ 0 ] == "background" ) {
            show_background();
        } else if( strCmd[ 0 ] == "update" ) {
            update_background( 20, strCmd.size() == 2 ? atoi( strCmd[ 1 ].c_str() ) : 0 );
        } else if( strCmd[ 0 ] == "record" ) {
            record( 1280, 480 );
        } else if( strCmd[ 0 ] == "findcorners" ) {
            findcorners( stereo_width, stereo_height );
        } else if( strCmd[ 0 ] == "quit" || strCmd[ 0 ] == "exit" ) {
            break;
        } else {
            cout << "Unkown command." << endl;
        }
    }

    if( !flgVideoFile ) {
        // Close Bumblebee
        CloseBumblebee();
    } else {
        // Close the video file
        video.release();
    }
    return 0;
}
