#include <iostream>
#include <sstream>
#include <fstream>

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

int iMaxCols = 1280, iMaxRows = 960;
int stereo_width = 512, stereo_height = 384;
//int width = 320, height = 240 ;
Mat img_background( stereo_height, stereo_width, CV_32F );
Mat H( 3, 4, CV_32F );

volatile bool flgEscape;

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

bool InitializeBumblebee()
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
    cout << oss.str() << " Loaded." << endl;
    te = triclopsGetDefaultContextFromFile( &triclops, (char*)oss.str().c_str() );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetDefaultContextFromFile()", te );

    return true;
}

bool SetBumblebeeParameteres( int width, int height )
{
    TriclopsError te;

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

void stereo( TriclopsImage16* pDst/*Mat* pDst*/, TriclopsInput& triclopsInput )
{
    TriclopsError te;

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

void execute()
{
    int width = stereo_width, height = stereo_height;
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

#ifdef WINDOWS_OS
    FrameRateCounter framerate;
    framerate.SetFrameRate( 0.0 );
#endif

    const float roi_width = 4.0f, roi_height = 4.0f;
    const float scale_m2px = 20.0f;
    const size_t len_compress_buf = (int)( roi_height * scale_m2px ) * (int)( roi_width * scale_m2px ) * 2;

    unsigned char* compress_buf = new unsigned char[ len_compress_buf ];
    
    Image rawImage;
    unsigned char* buffer = new unsigned char[ iMaxCols * 2 * iMaxRows ];
    Image deinterlacedImage( iMaxRows, iMaxCols * 2, iMaxCols * 2, buffer, iMaxRows * iMaxCols * 2, PIXEL_FORMAT_RAW8, GBRG );
    //Mat img( iMaxRows, iMaxCols * 2, CV_8UC4 );
    Mat img_display( height, width, CV_8U );
    Mat img_occupancy( (int)( scale_m2px * roi_height ), (int)( scale_m2px * roi_width ), CV_8U );
    Mat img_display2( (int)( scale_m2px * roi_height ) * 10, (int)( scale_m2px * roi_width ) * 10, CV_8U );
    //Image convertedImage( img.data, 4 * iMaxCols * 2 * iMaxRows );//img.rows() * img.cols() * 3 );
    TriclopsImage16 depthImage16;
    Mat img_depth( height, width, CV_32F );

    flgEscape = false;
#ifdef WINDOWS_OS
    DWORD idThread;
    HANDLE hThread = CreateThread( NULL, 0, KeyInThread, NULL, NULL, &idThread );
#else
    pthread_t thread;
    pthread_create(&thread , NULL , KeyInThread , NULL);
#endif
    for( ; ; ) {
        // Exit when ESC is hit.
//        char c = 0;
        if( flgWindow ) {
            (void)cvWaitKey( 1 );
        }
//        } else {
//#ifdef WINDOWS_OS
//            if( _kbhit() ) {
//#else
//            if( kbhit() ) {
//#endif
//                c = getchar();
//            }
//        }
//        if ( c == 27 || c == 'q' ) {
//            break;
//        }
        if( flgEscape ) {
            break;
        }

        // Retrieve an image
        err = bumblebee.RetrieveBuffer( &rawImage );
        if( err != PGRERROR_OK ) {
            PrintError( err );
            continue;
        }
#ifdef WINDOWS_OS
        framerate.NewFrame();
        cout << "Frame Rate: " << framerate.GetFrameRate() << endl;
#endif
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

        clock_t t = clock();
        stereo( &depthImage16, triclopsInput );
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
            int row = (int)( scale_m2px * ( it->x + roi_width / 2.0f ) ), col = (int)( scale_m2px * ( it->y + roi_width / 2.0f ) );
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

    err = bumblebee.StopCapture();
    destroyWindow( "Disparity" );
    destroyWindow( "Occupancy Map" );

#ifdef WINDOWS_OS
    WaitForSingleObject( hThread, INFINITE );
#else
    pthread_join(thread , NULL);
#endif

    delete [] buffer;
    delete [] compress_buf;
}

void calibration()
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

    SetBumblebeeParameteres( width = iMaxCols, height = iMaxRows );

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

    clock_t t = clock();
    stereo( &depthImage16, triclopsInput );
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
            img_display.at<unsigned char>( y, x ) = (unsigned char)( 25.0f * zz );
            if( x == 100 && y == 100 ) {
                cout << zz << ", " << 25.0f * zz << ", " << (int)( (25.0f * zz) ) << endl;//", ";
            }
        }
    }
    
    // Stop Capturing
    err = bumblebee.StopCapture();


    imshow( "Disparity", img_display );

    // find corners
    TriclopsImage rectifiedImage;
    te = triclopsGetImage( triclops, TriImg_RECTIFIED, TriCam_REFERENCE, &rectifiedImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );
    memcpy( img_display2.data, rectifiedImage.data, rectifiedImage.rowinc * rectifiedImage.nrows );

    const int width_pattern = 4, height_pattern = 7;
    Size patternsize( width_pattern,  height_pattern );
    vector<Point2f> corners;
    vector<Point3f> corners3d;
    vector<Point3f> corners3d_ref;
    bool ret = findChessboardCorners( img_display2, patternsize, corners );

    if( ret ) {
        cout << endl << "Corners detected";


        //float A[ 12 * 3 * width_pattern * height_pattern ];
        //float b[ 3 * width_pattern * height_pattern ];
        //float params[ 12 ];

        //ZeroMemory( A, 12 * 3 * width_pattern * height_pattern );

        Mat A = Mat::zeros( 3 * width_pattern * height_pattern, 12, CV_32F );
        Mat A2 = Mat::zeros( 2 * width_pattern * height_pattern, 8, CV_32F );
        Mat b( 3 * width_pattern * height_pattern, 1, CV_32F );
        Mat b2( 2 * width_pattern * height_pattern, 1, CV_32F );
        Mat params( 12, 1, CV_32F ), params2( 8, 1, CV_32F );
        //Mat ref_mat( 3, width_pattern * height_pattern, CV_32F ), pos_mat( 3, width_pattern * height_pattern, CV_32F );
        //Mat ref_mat( width_pattern * height_pattern, 3, CV_32F ), pos_mat( width_pattern * height_pattern, 3, CV_32F );
        Mat ref_mat( width_pattern * height_pattern, 1, CV_32FC3 ), pos_mat( width_pattern * height_pattern, 1, CV_32FC3 );

        //vector<Point3f> inliers( width_pattern * height_pattern );
        //vector<int> inliers( width_pattern * height_pattern );
        Mat inliers;

        int index = 0;
        for( vector<Point2f>::iterator it = corners.begin(); it != corners.end(); ++it, ++index ) {
	        cout << it->x << ", " << it->y;
            disparity = *(unsigned short*)((unsigned char*)depthImage16.data + depthImage16.rowinc * (int)(it->y) + (int)(it->x) * 2 );
            triclopsRCD16ToXYZ( triclops, (int)it->y, (int)it->x, disparity, &xx, &yy, &zz );
            if( disparity >= 0xff00 ) {
                zz = 0.0f;
                xx = 0.0f;
                yy = 0.0f;
            }

            corners3d.push_back( Point3f( xx, yy, zz ) );
	        cout << " -> " << xx << ", " << yy << ", " << zz;

            //pos_mat.at<float>( 0, index ) = xx;
            //pos_mat.at<float>( 1, index ) = yy;            
            //pos_mat.at<float>( 2, index ) = zz;
            pos_mat.at<float>( index, 0 ) = xx;
            pos_mat.at<float>( index, 1 ) = yy;            
            pos_mat.at<float>( index, 2 ) = zz;
            
            
            float ref_x, ref_y, ref_z;
            ref_x = (float)( index / width_pattern ) * 0.1485f;
            ref_y = (float)( index % width_pattern ) * 0.1485f;
            ref_z = 0.0f;

            //ref_mat.at<float>( 0, index ) = ref_x;
            //ref_mat.at<float>( 1, index ) = ref_y;
            //ref_mat.at<float>( 2, index ) = ref_z;
            ref_mat.at<float>( index, 0 ) = ref_x;
            ref_mat.at<float>( index, 1 ) = ref_y;
            ref_mat.at<float>( index, 2 ) = ref_z;

            corners3d_ref.push_back( Point3f( ref_x, ref_y, ref_z ) );
            cout << "(ref: " << ref_x << ", " << ref_y << ", " << ref_z << ")" << endl;
            
            //const int colinc = 3 * width_pattern * height_pattern;
            const int row = 3 * index;
            //A[ row                  ] = A[ ( row + 1 ) + ( 3 * colinc ) ] = A[ ( row + 2 ) + ( 6 * colinc ) ] = xx;
            //A[ row + ( 1 * colinc ) ] = A[ ( row + 1 ) + ( 4 * colinc ) ] = A[ ( row + 2 ) + ( 7 * colinc ) ] = yy;
            //A[ row + ( 2 * colinc ) ] = A[ ( row + 1 ) + ( 5 * colinc ) ] = A[ ( row + 2 ) + ( 8 * colinc ) ] = zz;

            //A[ row + ( 9 * colinc ) ] = A[ ( row + 1 ) + ( 10 * colinc ) ] = A[ ( row + 2 ) + ( 10 * colinc ) ] = 1.0f;

            A.at<float>( row, 0 ) = A.at<float>( row + 1, 3 ) = A.at<float>( row + 2, 6 ) = xx;
            A.at<float>( row, 1 ) = A.at<float>( row + 1, 4 ) = A.at<float>( row + 2, 7 ) = yy;
            A.at<float>( row, 2 ) = A.at<float>( row + 1, 5 ) = A.at<float>( row + 2, 8 ) = zz;
            A.at<float>( row, 9 ) = A.at<float>( row + 1, 10 ) = A.at<float>( row + 2, 11 ) = 1.0f;

            b.at<float>( row, 0 ) = ref_x;
            b.at<float>( row + 1, 0 ) = ref_y;
            b.at<float>( row + 2, 0 ) = ref_z;

            const int row2 = 2 * index;
            A2.at<float>( row2, 0 ) = A2.at<float>( row2 + 1, 4 ) = xx;
            A2.at<float>( row2, 1 ) = A2.at<float>( row2 + 1, 5 ) = yy;
            A2.at<float>( row2, 2 ) = A2.at<float>( row2 + 1, 6 ) = zz;
            A2.at<float>( row2, 3 ) = A2.at<float>( row2 + 1, 7 ) = 1.0f;
            
            b2.at<float>( row2, 0 ) = ref_x;
            b2.at<float>( row2 + 1, 0 ) = ref_y;

            //b[ row ] = ref_x;
            //b[ row + 1 ] = ref_y;
            //b[ row + 2 ] = ref_z;
        }

        drawChessboardCorners( img_display2, patternsize, Mat( corners ), ret );
        imshow( "image", img_display2 );

        //long n, nrhs = 1, ipiv[ 12 ], lda, ldb, info;
        //n = lda = ldb = 12;
        //sgesv_( &n, &nrhs, A, &lda, ipiv, b, &ldb, &info );

        //cout << endl << "parameters calculated" << endl;
        //for( int i = 0; i < 12; ++i ) {
        //    cout << b[ i ] << endl;
        //}

        //{
        //    ofstream ofs_A( "A.txt" ), ofs_b( "b.txt" );
        //    for( int row = 0; row < A.rows; ++row ) {
        //        for( int col = 0; col < A.cols; ++col ) {
        //            ofs_A << setprecision( 4 ) << A.at<float>( row, col ) << ", ";
        //        }
        //        ofs_A << endl;
        //        ofs_b << setprecision( 4 ) << b.at<float>( row, 0 ) << endl;
        //    }

        //    solve( A, b, params, DECOMP_SVD );
        //    cout << endl << "parameters calculated" << endl;
        //    for( int i = 0; i < 12; ++i ) {
        //        cout << setprecision( 4 ) << params.at<float>( i, 0 ) << endl;
        //    }
        //    H.create( 3, 4, CV_32F );
        //    H.at<float>( 0, 0 ) = params.at<float>( 0, 0 ); H.at<float>( 0, 1 ) = params.at<float>( 1, 0 ); H.at<float>( 0, 2 ) = params.at<float>( 2, 0 ); H.at<float>( 0, 3 ) = params.at<float>( 9, 0 );
        //    H.at<float>( 1, 0 ) = params.at<float>( 3, 0 ); H.at<float>( 1, 1 ) = params.at<float>( 4, 0 ); H.at<float>( 1, 2 ) = params.at<float>( 5, 0 ); H.at<float>( 1, 3 ) = params.at<float>( 10, 0 );
        //    H.at<float>( 2, 0 ) = params.at<float>( 6, 0 ); H.at<float>( 2, 1 ) = params.at<float>( 7, 0 ); H.at<float>( 2, 2 ) = params.at<float>( 8, 0 ); H.at<float>( 2, 3 ) = params.at<float>( 11, 0 );

        //    cout << endl << "reprojection" << endl;
        //    int i = 0;
        //    for( vector<Point3f>::iterator it = corners3d.begin(); it != corners3d.end(); ++it, ++i ) {
        //        Mat x( 4, 1, CV_32F ), ans;
        //        x.at<float>( 0, 0 ) = it->x; x.at<float>( 1, 0 ) = it->y; x.at<float>( 2, 0 ) = it->z; x.at<float>( 3, 0 ) = 1.0f;
        //        ans = H * x;
        //        cout << it->x << ", " << it->y << ", " << it->z 
        //             << " -> " << ans.at<float>( 0, 0 ) << ", " << ans.at<float>( 1, 0 ) << ", " << ans.at<float>( 2, 0 ) 
        //             << "(" << corners3d_ref[ i ].x << ", " << corners3d_ref[ i ].y << ", " << corners3d_ref[ i ].z << ")" << endl;
        //    }
        //}

        //{
        //    solve( A2, b2, params2, DECOMP_SVD );
        //    Mat xc( 3, 1, CV_32F ), yc( 3, 1, CV_32F ), zc;
        //    xc.at<float>( 0, 0 ) = params2.at<float>( 0, 0 );
        //    xc.at<float>( 1, 0 ) = params2.at<float>( 1, 0 );
        //    xc.at<float>( 2, 0 ) = params2.at<float>( 2, 0 );
        //    yc.at<float>( 0, 0 ) = params2.at<float>( 4, 0 );
        //    yc.at<float>( 1, 0 ) = params2.at<float>( 5, 0 );
        //    yc.at<float>( 2, 0 ) = params2.at<float>( 6, 0 );
        //    zc = xc.cross( yc );
        //    for( int i = 0; i < 8; ++i ) {
        //        cout << setprecision( 4 ) << params2.at<float>( i, 0 ) << endl;
        //        if( i == 3 ) {
        //            cout << endl;
        //        }
        //    }

        //    cout << zc.at<float>( 0, 0 ) << endl
        //         << zc.at<float>( 1, 0 ) << endl
        //         << zc.at<float>( 2, 0 ) << endl;
        //}

        H.create( 3, 4, CV_32F );
        H.at<float>( 0, 0 ) = 0.556880; H.at<float>( 0, 1 ) = -0.326309; H.at<float>( 0, 2 ) = 0.763811; H.at<float>( 0, 3 ) = -1.818401435;
        H.at<float>( 1, 0 ) = -0.829754; H.at<float>( 1, 1 ) = -0.259884; H.at<float>( 1, 2 ) = 0.493932; H.at<float>( 1, 3 ) = -1.335204279;
        H.at<float>( 2, 0 ) = 0.037327; H.at<float>( 2, 1 ) = -0.908836; H.at<float>( 2, 2 ) = -0.415480; H.at<float>( 2, 3 ) = 2.237207354;

        cout << endl << "reprojection" << endl;
        int i = 0;
        for( vector<Point3f>::iterator it = corners3d.begin(); it != corners3d.end(); ++it, ++i ) {
            Mat x( 4, 1, CV_32F ), ans;
            x.at<float>( 0, 0 ) = it->x; x.at<float>( 1, 0 ) = it->y; x.at<float>( 2, 0 ) = it->z; x.at<float>( 3, 0 ) = 1.0f;
            ans = H * x;
            cout << it->x << ", " << it->y << ", " << it->z 
                    << " -> " << ans.at<float>( 0, 0 ) << ", " << ans.at<float>( 1, 0 ) << ", " << ans.at<float>( 2, 0 ) 
                    << "(" << corners3d_ref[ i ].x - 1.485 << ", " << corners3d_ref[ i ].y - 1.485 * 2 << ", " << corners3d_ref[ i ].z << ")" << endl;
        }

/*
        //estimateAffine3D( pos_mat, ref_mat, H, inliers );
        //InputArray fa = corners3d, ta = corners3d_ref;
        //InputArray fa = pos_mat, ta = ref_mat;
        //InputArray fa = Mat(corners3d), ta = Mat(corners3d_ref);
        OutputArray ia = inliers;
        //Mat from = fa.getMat(), to = ta.getMat();
        Mat mask = ia.getMat();
        //CV_Assert( CV_ARE_SIZES_EQ(&from, &mask) );
        //Mat from = Mat( corners3d );
        //Mat to = Mat( corners3d_ref );
        Mat from, to;
        transpose( Mat( corners3d ), from );
        transpose( Mat( corners3d_ref ), to );
        cout << "from.depth()=" << from.depth() << ", to.depth()=" << to.depth() << endl;
        estimateAffine3D( from, to, H, inliers );
        cout << endl << "parameters calculated" << endl;
        //estimateAffine3D( pos_mat, ref_mat, H, inliers );
        for( int row = 0; row < H.rows; ++row ) {
            for( int col = 0; col < H.cols; ++col ) {
                cout << setprecision( 4 ) << H.at<double>( row, col ) << ", ";
            }
            cout << endl;
        }
        
        cout << endl << "reprojection" << endl;
        i = 0;
        for( vector<Point3f>::iterator it = corners3d.begin(); it != corners3d.end(); ++it, ++i ) {
            Mat x( 4, 1, CV_64F ), ans;
            x.at<double>( 0, 0 ) = it->x; x.at<double>( 1, 0 ) = it->y; x.at<double>( 2, 0 ) = it->z; x.at<double>( 3, 0 ) = 1.0f;
            ans = H * x;
            cout << it->x << ", " << it->y << ", " << it->z 
                 << " -> " << ans.at<double>( 0, 0 ) << ", " << ans.at<double>( 1, 0 ) << ", " << ans.at<double>( 2, 0 ) 
                 << "(" << corners3d_ref[ i ].x << ", " << corners3d_ref[ i ].y << ", " << corners3d_ref[ i ].z << ")" << endl;
        }
*/
    } else {
      cout << endl << "No corners found." << endl;
    }
    
    while( true ) {
        // Exit when ESC is hit.
        char c = cvWaitKey( 1 );
        if ( c == 27 ) {
            break;
        }
    }

    destroyWindow( "Disparity" );
    destroyWindow( "image" );

    SetBumblebeeParameteres( stereo_width, stereo_height );

    delete [] buffer;
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


    Image rawImage;
    unsigned char* buffer = new unsigned char[ iMaxCols * 2 * iMaxRows ];
    Image deinterlacedImage( iMaxRows, iMaxCols * 2, iMaxCols * 2, buffer, iMaxRows * iMaxCols * 2, PIXEL_FORMAT_RAW8, GBRG );
    Mat img_display( height, width, CV_8U );
    TriclopsImage16 depthImage16;
    Mat cnt( height, width, CV_8U );
    int framecnt = 0;

    for( int x = 0; x < img_background.cols; ++x ) {
        for( int y  = 0; y < img_background.rows; ++y ) {
            img_background.at<float>( y, x ) = 0.0;
            cnt.at<unsigned char>( y, x ) = 0;
        }
    }

    for( int frame = 0; frame < nFrame; ++frame ) {

        cout << "# of frames: " << frame + 1 << endl;

        // Retrieve an image
        err = bumblebee.RetrieveBuffer( &rawImage );
        if( err != PGRERROR_OK ) {
            PrintError( err );
            continue;
        }

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

        clock_t t = clock();
        stereo( &depthImage16, triclopsInput );
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

    err = bumblebee.StopCapture();


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

    delete [] buffer;
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

    SetBumblebeeParameteres( width = iMaxCols, height = iMaxRows );

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

    SetBumblebeeParameteres( stereo_width, stereo_height );

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
        }
    }

    // Initialize Bumblebee
    ret = InitializeBumblebee();
    if( ret == false ) {
        cout << "Camera initialization failed." << endl;
        exit( 1 );
    }

    SetBumblebeeParameteres( stereo_width, stereo_height );


    // Load a background image
    cout << endl;
    if( load_background() ) {
        cout << "Background image loaded." << endl;
    } else {
        cout << "No background image loaded." << endl;
    }


    // Load extrinsic parameters
        H.at<float>( 0, 0 ) = 0.556880; H.at<float>( 0, 1 ) = -0.326309; H.at<float>( 0, 2 ) = 0.763811; H.at<float>( 0, 3 ) = -1.818401435;
        H.at<float>( 1, 0 ) = -0.829754; H.at<float>( 1, 1 ) = -0.259884; H.at<float>( 1, 2 ) = 0.493932; H.at<float>( 1, 3 ) = -1.335204279;
        H.at<float>( 2, 0 ) = 0.037327; H.at<float>( 2, 1 ) = -0.908836; H.at<float>( 2, 2 ) = -0.415480; H.at<float>( 2, 3 ) = 2.237207354;

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
        } else if( strCmd == "calibrate" ) {
            calibration();
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

    // Close Bumblebee
    CloseBumblebee();

    return 0;
}
