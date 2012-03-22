#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include "FlyCapture2.h"
#include "triclops.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "zlib.h"

#if defined(WINDOWS) || defined(_WIN32)
#define WINDOWS_OS
#elif defined(linux) || defined(__linux__)
#define LINUX_OS
#endif

#ifdef WINDOWS_OS
#include <conio.h>
#include "FrameRateCounter.h"
#endif

#ifdef LINUX_OS
#include <sys/times.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <curses.h>
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

int iMaxCols = 1280, iMaxRows = 960;
int stereo_width = 512, stereo_height = 384;
//int stereo_width = 640, stereo_height = 480;
Mat img_background( stereo_height, stereo_width, CV_32F );
Mat H( 3, 4, CV_32F );

volatile bool flgEscape;

const float roi_width = 4.0f, roi_height = 4.0f;
const float roi_x = 0.0f, roi_y = 6.0f;
const float scale_m2px = 20.0f;

#ifdef LINUX_OS

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

bool SetStereoParameters( int width, int height ) {
    TriclopsError te;

    //
    // Change stereo parameters
    cout << endl;
    cout << "Changing stereo parameters..." << endl; 
    
    // Set the base-line of the stereo camera narrow.
    TriclopsCameraConfiguration triclopscamconfig;
    te = triclopsGetCameraConfiguration( triclops, &triclopscamconfig );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetCameraConfiguration()", te );  
    cout << endl;
    cout << " Base-line setting(old): " << triclopscamconfig << endl;
    te = triclopsSetCameraConfiguration( triclops, TriCfg_2CAM_HORIZONTAL_NARROW );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetCameraConfiguration()", te );  
    te = triclopsGetCameraConfiguration( triclops, &triclopscamconfig );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetCameraConfiguration()", te );  
    cout << " Base-line setting(new): " << triclopscamconfig << endl;
    cout << " Make sure the value is 1 (TriCfg_2CAM_HORIZONTAL_NARROW)" << endl;

    // set rectified resolution to width x height 
    int nrows, ncols;
    te = triclopsGetResolution( triclops, &nrows, &ncols );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetResolution()", te );
    cout << endl;
    cout << " Resolution(old): " << ncols << " x " << nrows << endl;
    te = triclopsSetResolution( triclops, height, width );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetResolution()", te );
    te = triclopsGetResolution( triclops, &nrows, &ncols );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetResolution()", te );
    cout << " Resolution(new): " << ncols << " x " << nrows << endl;

    // Set disparity range
    int minDisparity, maxDisparity;
    te = triclopsGetDisparity( triclops, &minDisparity, &maxDisparity );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetDisparity()", te );   
    cout << endl;
    cout << " Disparity range(old): " << minDisparity << " - " << maxDisparity << endl;
    te = triclopsSetDisparity( triclops, 5, 60 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparity()", te );   
    te = triclopsGetDisparity( triclops, &minDisparity, &maxDisparity );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetDisparity()", te );   
    cout << " Disparity range(new): " << minDisparity << " - " << maxDisparity << endl;

    // Set the window size for stereo matching.
    int size;
    te = triclopsGetStereoMask( triclops, &size );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetStereoMask()", te );   
    cout << endl;
    cout << " Window size(old): " << size << endl;
    te = triclopsSetStereoMask( triclops, 11 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetStereoMask()", te );   
    te = triclopsGetStereoMask( triclops, &size );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetStereoMask()", te );   
    cout << " Window size(new): " << size << endl;

    // Turn off texture validation
    TriclopsBool on;
    te = triclopsGetTextureValidation( triclops, &on );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetTextureValidation()", te );
    cout << endl;
    cout << " Texture Validation(old): " << on << endl;
    te = triclopsSetTextureValidation( triclops, 0 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetTextureValidation()", te );
    te = triclopsGetTextureValidation( triclops, &on );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetTextureValidation()", te );
    cout << " Texture Validation(new): " << on << endl;

    // Turn off texture validation
    te = triclopsGetUniquenessValidation( triclops, &on );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetUniquenessValidation()", te );
    cout << endl;
    cout << " Uniqueness Validation(old): " << on << endl;
    te = triclopsSetUniquenessValidation( triclops, 0 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetUniquenessValidation()", te );
    te = triclopsGetUniquenessValidation( triclops, &on );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetUniquenessValidation()", te );
    cout << " Uniqueness Validation(new): " << on << endl;

    // Turn on sub-pixel interpolation
    te = triclopsGetSubpixelInterpolation( triclops, &on );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetSubpixelInterpolation()", te );
    cout << endl;
    cout << " SubpixelInterpolation(old): " << on << endl;
    te = triclopsSetSubpixelInterpolation( triclops, 1 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", te );
    te = triclopsGetSubpixelInterpolation( triclops, &on );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetSubpixelInterpolation()", te );
    cout << " SubpixelInterpolation(new): " << on << endl;

    // With Linux, Set # of thread to one in order to avoid a segmentation 
    // falut at triclopsStereo(), which seems a bug in the triclopsSDK.
#ifdef LINUX_OS
    int maxThreadCount;
    te = triclopsGetMaxThreadCount( triclops, &maxThreadCount );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetMaxThreadCount()", te );
    cout << " The maximum number of threads(old): " << maxThreadCount << endl;
    te = triclopsSetMaxThreadCount( triclops, 1 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetMaxThreadCount()", te );
    te = triclopsGetMaxThreadCount( triclops, &maxThreadCount );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetMaxThreadCount()", te );
    cout << " The maximum number of threads(new): " << maxThreadCount << endl;
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

void stereo( TriclopsImage16* pDst/*Mat* pDst*/, const Mat& src/*TriclopsInput& triclopsInput*/ )
{
    TriclopsError te;
    TriclopsInput triclopsInput;

    int imageCols = src.cols / 2;
    int imageRows = src.rows;
    int imageRowInc = src.elemSize1() * src.cols * src.channels();
    unsigned long timeStampSeconds = 12880571209;//rawImage.GetTimeStamp().seconds;
    unsigned long timeStampMicroSeconds = 9890000;//rawImage.GetTimeStamp().microSeconds;

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

void grab_from_bumblebee( Mat* pDst )
{
    Error err;
    Image rawImage;
    unsigned char* buffer = pDst->data;

    err = bumblebee.RetrieveBuffer( &rawImage );
    if( err != PGRERROR_OK ) {
        PrintError( err );
        return;
    }

    pDst->create( iMaxRows, iMaxCols * 2, CV_8U );

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
}

void grab_from_video( Mat* pDst )
{
    Mat tmp;
    video >> tmp;//imgFromVideo;
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
}

void execute()
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

    ofstream ofs;
    if( flgSavePEPMap ) {
        string strPath, strName, strNoextName;
        getfilename( strVideoFile, &strPath, &strName, &strNoextName );
        ostringstream oss;
        oss << strPath << "pepmap" << camInfo.serialNumber << ".dat";
        ofs.open( oss.str().c_str() );
    }

    // debug code
    if( flgVideoFile ) {
        video.set( CV_CAP_PROP_POS_FRAMES, 1500 );
    }

#ifdef WINDOWS_OS
    FrameRateCounter framerate;
    framerate.SetFrameRate( 0.0 );
#endif

    const size_t len_compress_buf = (int)( roi_height * scale_m2px ) * (int)( roi_width * scale_m2px ) * 2;
    unsigned char* compress_buf = new unsigned char[ len_compress_buf ];
    
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
        // Exit when 'q' and enter keys are hit.
        if( flgWindow ) {
            (void)cvWaitKey( 1 );
        }
        if( flgEscape ) {
            break;
        }

        // Retrieve an image
        if( !flgVideoFile ) {
            grab_from_bumblebee( &image );
            timeStamp = 0; // debug
        } else {
            grab_from_video( &image );
            int frame = video.get( CV_CAP_PROP_POS_FRAMES );
            timeStamp = frame_to_timestamp[ frame ];
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

        // Stereo Processing
        clock_t t = clock();
        stereo( &depthImage16, image );
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
                //img_display.at<unsigned char>( y, x ) = (unsigned char)( 25.0f * zz );
                if( x == 100 && y == 100 ) {
                    cout << zz << ", " << 25.0f * zz << ", " << (int)( (25.0f * zz) ) << endl;//", ";
                }
            }
        }


        //TriclopsImage3d* pImage3d;
        //te = triclopsCreateImage3d( triclops, &pImage3d );
        //te = triclopsExtractImage3d( triclops, pImage3d );
        //for( int x = 0; x < pImage3d->ncols; ++x ) {
        //  for( int y = 0; y < pImage3d->nrows; ++y ) {
        //      img_depth.at<float>( y, x ) = pImage3d->points[ x + y * pImage3d->ncols ].point[ 2 ];
        //  }
        //}
        //triclopsDestroyImage3d( &pImage3d );

        // background subtraction（ベクトル計算で高速化の余地あり）
        vector<Point3f> point_foreground;
        Mat xvec( 4, 1, CV_32F );
        Mat point_planview( 3, 1, CV_32F );
        for( int x = 0; x < img_depth.cols; ++x ) {
            for( int y  = 0; y < img_depth.rows; ++y ) {
                if( abs( img_depth.at<float>( y, x ) - img_background.at<float>( y, x ) ) < 0.2f ) {
                    img_depth.at<float>( y, x ) = 0.0f;
                } else {
                    disparity = *(unsigned short*)((unsigned char*)depthImage16.data + depthImage16.rowinc * y + x * 2 );
                    if( disparity < 0xff00 ) {
                        triclopsRCD16ToXYZ( triclops, y, x, disparity, &xx, &yy, &zz );
                        xvec.at<float>( 0, 0 ) = xx; xvec.at<float>( 1, 0 ) = yy; xvec.at<float>( 2, 0 ) = zz; xvec.at<float>( 3, 0 ) = 1.0;
                        point_planview =  H * xvec ;
                        point_foreground.push_back( Point3f( point_planview.at<float>( 0, 0 ), point_planview.at<float>( 1, 0 ), point_planview.at<float>( 2, 0 ) ) );
                    }                    
                }
            }
        }

        // 
        // Create an occupancy map with foreground data
        Mat occupancy = Mat::zeros( (int)( roi_height * scale_m2px ), (int)( roi_width * scale_m2px ), CV_16U );
        for( vector<Point3f>::iterator it = point_foreground.begin(); it != point_foreground.end(); ++it ) {
            int row = (int)( scale_m2px * ( ( it->x - roi_x ) + roi_width / 2.0f ) ), col = (int)( scale_m2px * ( ( it->y - roi_y ) + roi_width / 2.0f ) );
            if( row >= 0 && row < occupancy.rows && col >= 0 && col < occupancy.cols ) {
                occupancy.at<unsigned short>( row, col ) = occupancy.at<unsigned short>( row, col ) + 1;
            }
        }
        for( int row = 0; row < occupancy.rows; ++row ) {
            for( int col = 0; col < occupancy.cols; ++col ) {
                if( occupancy.at<unsigned short>( row, col ) < 100 ) {
                    occupancy.at<unsigned short>( row, col ) = 0;
                }
            }
        }

        // Compress occupancy map
        uLongf len_compressed = len_compress_buf;
        compress( compress_buf
                , &len_compressed
                , occupancy.data
                , len_compress_buf );
        cout << "Compressed the occupancy map: size=" << len_compress_buf << " -> " << len_compressed << "[bytes]" << endl;

            
        // test code for checking compression validity, where the occupancy maps are restored with the compressed data.
        uLongf len_uncompressed = len_compress_buf;
        uncompress( occupancy.data
                    , &len_uncompressed
                    , compress_buf
                    , len_compressed );
        cout << "(Test Code!)Unompressed the occupancy map: size=" << len_compressed << " -> " << len_uncompressed << "[bytes]" << endl;

	    // Send PEPMap data to stdout
	    cout << "<PEPMap>" << endl // Header
             << camInfo.serialNumber << endl // Serial Number
             << timeStamp << endl // Time stamp             
             << len_compressed << endl; // PEPMap data length
	    for( size_t i = 0; i < len_compressed; ++i ) { // PEPMap data
	      cout << hex << setw(2) << setfill( '0' ) << (int)compress_buf[ i ];
	    }
	    cout << dec << endl;

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

        if( flgWindow ) {
            occupancy.convertTo( img_occupancy, CV_8U );
            resize( img_occupancy, img_display2, img_display2.size() );
            imshow( "Occupancy Map", img_display2 );
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

#ifdef WINDOWS_OS
    WaitForSingleObject( hThread, INFINITE );
#else
    pthread_join(thread , NULL);
#endif

    //delete [] buffer;
    delete [] compress_buf;
}

bool load_extrinsic_parameters()
{
    ifstream ifs;

    string strPath, strName, strNoextName;
    
    if( flgVideoFile ) {
        getfilename( strVideoFile, &strPath, &strName, &strNoextName );
    }

    ostringstream oss;
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

bool load_background()
{
    ifstream ifs;

    ostringstream oss;
#ifdef LINUX_OS
    oss << "/home/kumalab/project/HumanTracking/bin/";
#endif
    oss << "background" << camInfo.serialNumber << ".dat";
    ifs.open( oss.str().c_str(), ios::in | ios::binary );

    if( !ifs ) {
        return false;
    }

    int rowinc = (int)( img_background.step1() * img_background.elemSize1() );
    ifs.read( (char*)img_background.data, img_background.rows * rowinc ); 

    ifs.close();

    return true;
}

bool save_background()
{
    ofstream ofs;

    ostringstream oss;
#ifdef LINUX_OS
    oss << "/home/kumalab/project/HumanTracking/bin/";
#endif
    oss << "background" << camInfo.serialNumber << ".dat";

    ofs.open( oss.str().c_str(), ios::out | ios::binary | ios::trunc );
    if( !ofs ) {
        cout << "Error occured in saving the background image." << endl;
        return false;
    }

    int rowinc = (int)( img_background.step1() * img_background.elemSize1() );
    ofs.write( (char*)img_background.data, img_background.rows * rowinc ); 

    ofs.close();

    return true;
}

void show_background()
{
    if( flgWindow ) {
        Mat img_display( img_background.size(), CV_8U );
        img_background.convertTo( img_display, CV_8U, 25.0, 0.0 );
        imshow( "background image", img_display );  

        while( 1 ) {
            // Exit when ESC is hit.
            char c = cvWaitKey( 10 );
            if ( c == 27 ) {
                break;
            }
        }

        destroyWindow( "background image" );
    }
}

void update_background( int nFrame )
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
        stereo( &depthImage16, image/*triclopsInput*/ );
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

void capture()
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
    if( !load_extrinsic_parameters() ) {
        cout << "Failed in loading the extrinsic parameters." << endl;
        exit( 1 );
    }
    cout << "<Initialized>" << endl;
    cout << camInfo.serialNumber << endl;

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
    string strCmd;
    while( 1 ) {
        cout << ">";
        cin >> strCmd;
        if( strCmd == "run" ) {
            execute();
        //} else if( strCmd == "update" ) {
        //    update_background( 20 );
        //} else if( strCmd == "background" ) {
        //    show_background();
        //} else if( strCmd == "calibrate" ) {
        //    calibration();
        } else if( strCmd == "capture" ) {
            capture();
        } else if( strCmd == "background" ) {
            show_background();
        } else if( strCmd == "update" ) {
            update_background( 20 );
        } else if( strCmd == "quit" || strCmd == "exit" ) {
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
