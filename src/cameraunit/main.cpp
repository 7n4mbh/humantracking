#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>

#include "windows.h"
#include "triclops.h"
#include "pgrflycapture.h"
#include "pgrflycapturestereo.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

using namespace std;
using namespace cv;

#define _HANDLE_TRICLOPS_ERROR( description, error )	\
{ \
   if( error != TriclopsErrorOk ) \
   { \
      printf( \
	 "*** Triclops Error '%s' at line %d :\n\t%s\n", \
	 triclopsErrorToString( error ), \
	 __LINE__, \
	 description );	\
      printf( "Press any key to exit...\n" ); \
      getchar(); \
      exit( 1 ); \
   } \
} \

//
// Macro to check, report on, and handle Digiclops API error codes.
//
#define _HANDLE_FLYCAPTURE_ERROR( description, error )	\
{ \
   if( error != FLYCAPTURE_OK ) \
   { \
      printf( \
	 "*** Flycapture Error '%s' at line %d :\n\t%s\n", \
	 flycaptureErrorToString( error ), \
	 __LINE__, \
	 description );	\
      printf( "Press any key to exit...\n" ); \
      getchar(); \
      exit( 1 ); \
   } \
} \

FlyCaptureContext flycapture;
FlyCaptureInfoEx flycaptureInfo;
TriclopsContext triclops;

char* szCalFile;
unsigned long ulDevice = 0;

unsigned char* rowIntColor = NULL;
unsigned char* rowIntMono = NULL;

FlyCapturePixelFormat pixelFormat = FLYCAPTURE_RAW16;
int iMaxCols = 1280, iMaxRows = 960;
int width = 512, height = 384;
Mat img_background( height, width, CV_32F );

bool InitializeBumblebee()
{
    TriclopsError te;
    FlyCaptureError fe;
    
    // Creating a handle(FlyCapturecontext) to the camera.
    fe = flycaptureCreateContext( &flycapture );
    _HANDLE_FLYCAPTURE_ERROR( "flycaptureCreateContext()", fe );

    // Initialize the Flycapture context
    fe = flycaptureInitialize( flycapture, ulDevice );
    _HANDLE_FLYCAPTURE_ERROR( "flycaptureInitialize()", fe );
    cout << "Camera" << ulDevice << " successfully initialized" << endl;

    // Get camera information
    fe = flycaptureGetCameraInfo( flycapture, &flycaptureInfo );
    _HANDLE_FLYCAPTURE_ERROR( "flycatpureGetCameraInfo()", fe ); 
    const char* strFlyCaptureCameraType[] = { "B/W", "Color" };
    cout << endl;
    cout << " Serial Number: " << flycaptureInfo.SerialNumber << endl;
    cout << " Type of imager: " << strFlyCaptureCameraType[ flycaptureInfo.CameraType ] << endl;
    cout << " Camera model: " << flycaptureInfo.CameraModel << endl;
    cout << " Camera model string: " << flycaptureInfo.pszModelName << endl;
    cout << " Vendor name string: " << flycaptureInfo.pszVendorName << endl;
    cout << " Sensor info string: " << flycaptureInfo.pszSensorInfo << endl;
    cout << " 1394 DCAM compliance level: " << (double)flycaptureInfo.iDCAMVer / 100.0 << endl;
    cout << " Low-level 1394 node number for this device: " << flycaptureInfo.iNodeNum << endl;
    cout << " Low-level 1394 bus number for this device: " << flycaptureInfo.iBusNum << endl;
    cout << " Camera max bus speed: " << flycaptureInfo.CameraMaxBusSpeed << endl;

    // Get the path for camera calibration information embedded in the camera.
    fe = flycaptureGetCalibrationFileFromCamera( flycapture, &szCalFile );
    _HANDLE_FLYCAPTURE_ERROR( "flycaptureGetCalibrationFileFromCamera()", fe );
    cout << endl << "Calibration data fetched.";

    // Store the calibration information to a file
    ostringstream oss;
    oss << "calibration" << flycaptureInfo.SerialNumber << ".txt";
    ::CopyFileA( szCalFile, oss.str().c_str(), FALSE );
    cout << " The data is stored to '" << oss.str() << "'" << endl;

    // Create a Triclops context from the cameras calibration file
    te = triclopsGetDefaultContextFromFile( &triclops, szCalFile );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetDefaultContextFromFile()", te );

    //
    // End the program



    return true;
}

bool CloseBumblebee()
{
    FlyCaptureError fe;

    fe = flycaptureDestroyContext( flycapture );
    _HANDLE_FLYCAPTURE_ERROR( "flycaptureDestroyContext()", fe );
    cout << endl << "Camera" << ulDevice << " closed." << endl;

    return true;
}

bool SetBumblebeeParameteres( int width, int height )
{
    TriclopsError te;
    FlyCaptureError fe;

    cout << endl;
    cout << "Changing camera parameters..." << endl; 

    // Set the bus speed as high as possible.
    FlyCaptureBusSpeed bs1, bs2;
    fe = flycaptureGetBusSpeed( flycapture, &bs1, &bs2 );
    cout << endl;
    cout << " Bus Speed(old): asyncBusSpeed=" << bs1 << ", isochBusSpeed=" << bs2 << endl;
    fe = flycaptureSetBusSpeed( flycapture, FLYCAPTURE_S_FASTEST, FLYCAPTURE_S_FASTEST );
    cout << " Bus Speed(new): asyncBusSpeed=" << bs1 << ", isochBusSpeed=" << bs2 << endl;
    cout << " Make sure both speeds are no less than 2 (FLYCAPTURE_S400)." << endl;

    // Use the center and right cameras for stereo processing.
    long lValueA, lValueB;
    bool bAuto;
    fe = flycaptureGetCameraProperty( flycapture
                                    , FLYCAPTURE_PAN
                                    , &lValueA 
                                    , &lValueB
                                    , &bAuto );
    cout << endl;
    cout << " FLYCAPTURE_PAN: " << "lValueA=" << lValueA << ", lValueB=" << lValueB << ", bAuto=" << bAuto << endl;
    fe = flycaptureSetCameraProperty( flycapture
                                    , FLYCAPTURE_PAN
                                    , 1 
                                    , 0
                                    , false );
    fe = flycaptureGetCameraProperty( flycapture
                                    , FLYCAPTURE_PAN
                                    , &lValueA 
                                    , &lValueB
                                    , &bAuto );
    cout << " FLYCAPTURE_PAN: " << "lValueA=" << lValueA << ", lValueB=" << lValueB << ", bAuto=" << bAuto << endl;
    cout << " Make sure lValueA=1 (Use center and right cameras for stereo)." << endl;

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
    te = triclopsSetDisparity( triclops, 0, 100 );
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

    cout << endl << "Done." << endl;

    return true;
}

void grabcolor( TriclopsInput* pDst )
{
    TriclopsError te;
    FlyCaptureError fe;
    
    FlyCaptureImage flycaptureImage;
   
    // Grab an image from the camera
    fe = flycaptureGrabImage2( flycapture, &flycaptureImage );
    _HANDLE_FLYCAPTURE_ERROR( "flycaptureGrabImage()", fe );

    // Extract information from the FlycaptureImage
    int imageCols = flycaptureImage.iCols;
    int imageRows = flycaptureImage.iRows;
    int imageRowInc = flycaptureImage.iRowInc;
    int iSideBySideImages = flycaptureImage.iNumImages;
    unsigned long timeStampSeconds = flycaptureImage.timeStamp.ulSeconds;
    unsigned long timeStampMicroSeconds = flycaptureImage.timeStamp.ulMicroSeconds;

    // Create buffers for holding the color and mono images
    if( rowIntColor == NULL ) {
        rowIntColor = new unsigned char[ imageCols * imageRows * iSideBySideImages * 4 ];
    }

    // Create a temporary FlyCaptureImage for preparing the stereo image
    FlyCaptureImage tempImage;
    tempImage.pData = rowIntColor;

    // Convert the pixel interleaved raw data to row interleaved format
    fe = flycapturePrepareStereoImage( flycapture, flycaptureImage, NULL, &tempImage );
    _HANDLE_FLYCAPTURE_ERROR( "flycapturePrepareStereoImage()", fe );

    // Pointers to positions in the color buffer that correspond to the beginning
    // of the red, green and blue sections
    unsigned char* redColor = NULL;
    unsigned char* greenColor = NULL;
    unsigned char* blueColor = NULL;

    redColor = rowIntColor;
    if (flycaptureImage.iNumImages == 2)
    {
        greenColor = redColor + ( 4 * imageCols );
        blueColor = redColor + ( 4 * imageCols );
    }
    if (flycaptureImage.iNumImages == 3)
    {
        greenColor = redColor + ( 4 * imageCols );
        blueColor = redColor + ( 2 * 4 * imageCols );
    }

    // Use the row interleaved images to build up a packed TriclopsInput.
    // A packed triclops input will contain a single image with 32 bpp.
    te = triclopsBuildPackedTriclopsInput(
        imageCols,
        imageRows,
        imageRowInc * 4,
        timeStampSeconds,
        timeStampMicroSeconds,
        redColor,
        pDst );
    _HANDLE_TRICLOPS_ERROR( "triclopsBuildPackedTriclopsInput()", te );
}

void grabmono( TriclopsInput* pDst )
{
    TriclopsError te;
    FlyCaptureError fe;
    
    FlyCaptureImage flycaptureImage;

    // Grab an image from the camera
    fe = flycaptureGrabImage2( flycapture, &flycaptureImage );
    _HANDLE_FLYCAPTURE_ERROR( "flycaptureGrabImage()", fe );

    // Extract information from the FlycaptureImage
    int imageCols = flycaptureImage.iCols;
    int imageRows = flycaptureImage.iRows;
    int imageRowInc = flycaptureImage.iRowInc;
    int iSideBySideImages = flycaptureImage.iNumImages;
    unsigned long timeStampSeconds = flycaptureImage.timeStamp.ulSeconds;
    unsigned long timeStampMicroSeconds = flycaptureImage.timeStamp.ulMicroSeconds;

    // Create buffers for holding the mono images
    if( rowIntMono == NULL ) { 
        rowIntMono = new unsigned char[ imageCols * imageRows * iSideBySideImages ];
    }

    // Create a temporary FlyCaptureImage for preparing the stereo image
    FlyCaptureImage tempImage;
    tempImage.pData = rowIntMono;

    // Convert the pixel interleaved raw data to row interleaved format
    fe = flycapturePrepareStereoImage( flycapture, flycaptureImage, &tempImage, NULL);
    _HANDLE_FLYCAPTURE_ERROR( "flycapturePrepareStereoImage()", fe );

    // Pointers to positions in the mono buffer that correspond to the beginning
    // of the red, green and blue sections
    unsigned char* redMono = NULL;
    unsigned char* greenMono = NULL;
    unsigned char* blueMono = NULL;

    redMono = rowIntMono;
    if (flycaptureImage.iNumImages == 2)
    {
       greenMono = redMono + imageCols;
       blueMono = redMono + imageCols;
    }
    if (flycaptureImage.iNumImages == 3)
    {
       greenMono = redMono + imageCols;
       blueMono = redMono + ( 2 * imageCols );
    }

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
      pDst);
    _HANDLE_TRICLOPS_ERROR( "triclopsBuildRGBTriclopsInput()", te );
}

void stereo( TriclopsImage16* pDst, TriclopsInput& triclopsInput )
{
    TriclopsError te;
/*
    TriclopsPackedColorImage  colorImage;

    // rectify the color image
    te = triclopsRectifyPackedColorImage( triclops, 
			       TriCam_REFERENCE, 
			       &colorInput, 
			       &colorImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectifyPackedColorImage()", te );

    //memcpy( pDst->data, colorImage.data, colorImage.rowinc * colorImage.nrows );
*/
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
    FlyCaptureError fe;

    // Start capturing
    // "DCAM Format 7"規格を用いた画像取得プロセスを開始
    fe = flycaptureStartCustomImage( flycapture, 3, 0, 0, iMaxCols, iMaxRows, 100, pixelFormat );
    _HANDLE_FLYCAPTURE_ERROR( "flycaptureStart()", fe );
    cout << endl << "Capturing Started. The image size is " << iMaxCols << "x" << height << endl;

    TriclopsInput triclopsInput;
    TriclopsImage16 depthImage16;
    Mat img( iMaxRows, iMaxCols * 2, CV_8UC4 );
    Mat img_disparity( height, width, CV_32F );
    Mat img_display( height, width, CV_8U );
    DWORD t, tref;
    int framecnt = 0;
    while( 1 ) {
        if( framecnt == 0 ) {
            tref = timeGetTime();
        }

        // Fetch an image
        //grabcolor( &triclopsInput );
        grabmono( &triclopsInput );
        ++framecnt;

        //memcpy( img.data, colorInput.u.rgb32BitPacked.data, colorInput.rowinc * colorInput.nrows );
        //resize( img, img_display, img_display.size(), 0, 0 );
        //imshow( "image", img_display );

        // Stereo Processing
        stereo( &depthImage16, triclopsInput );
        unsigned short disparity;
        float xx, yy, zz;
        
        // （OpenMPなどで高速化の余地あり）
        for( int x = 0; x < depthImage16.ncols; x++ ) {
            for( int y = 0; y < depthImage16.nrows; y++ ) {
                disparity = *(unsigned short*)((BYTE*)depthImage16.data + depthImage16.rowinc * y + x * 2 );
                triclopsRCD16ToXYZ( triclops, y, x, disparity, &xx, &yy, &zz );
                if( disparity >= 0xff00 ) {
                    zz = 0.0f;
                    xx = 0.0f;
                    yy = 0.0f;
                }
                img_disparity.at<float>( y, x ) = zz;
            }
        }

        // background subtraction（ベクトル計算で高速化の余地あり）
        for( int x = 0; x < img_disparity.cols; ++x ) {
            for( int y  = 0; y < img_disparity.rows; ++y ) {
                if( abs( img_disparity.at<float>( y, x ) - img_background.at<float>( y, x ) ) < 0.2f ) {
                    img_disparity.at<float>( y, x ) = 0.0f;
                }
            }
        }
        img_disparity.convertTo( img_display, CV_8U, 25.0, 0.0 );
        imshow( "image", img_display );

        // Calculate FPS.
        t = timeGetTime();
        if( t - tref >= 1000 ) {
            cout << framecnt << endl;
            framecnt = 0;
        }

        // Exit when ESC is hit.
        char c = cvWaitKey( 1 );
        if ( c == 27 ) {
            break;
        }
    }

    destroyWindow( "image" );

    // Stop capturing
    fe = flycaptureStop( flycapture );
    _HANDLE_FLYCAPTURE_ERROR( "flycaptureStop()", fe );
    cout << endl << "Capturing stopped." << endl;
}

void update_background( int nFrame )
{
    FlyCaptureError fe;

    // Start capturing
    // "DCAM Format 7"規格を用いた画像取得プロセスを開始
    fe = flycaptureStartCustomImage( flycapture, 3, 0, 0, iMaxCols, iMaxRows, 100, pixelFormat );
    _HANDLE_FLYCAPTURE_ERROR( "flycaptureStart()", fe );
    cout << endl << "Capturing Started. The image size is " << iMaxCols << "x" << height << endl;

    TriclopsInput triclopsInput;
    TriclopsImage16 depthImage16;
    Mat cnt( height, width, CV_8U );
    Mat img_display( height, width, CV_8U );
    int framecnt = 0;

    for( int x = 0; x < img_background.cols; ++x ) {
        for( int y  = 0; y < img_background.rows; ++y ) {
            img_background.at<float>( y, x ) = 0.0;
            cnt.at<unsigned char>( y, x ) = 0;
        }
    }

    for( int frame = 0; frame < nFrame; ++frame ) {
        // Fetch an image
        //grabcolor( &triclopsInput );
        grabmono( &triclopsInput );
        cout << "# of frames: " << frame + 1 << endl;

        //memcpy( img.data, colorInput.u.rgb32BitPacked.data, colorInput.rowinc * colorInput.nrows );
        //resize( img, img_display, img_display.size(), 0, 0 );
        //imshow( "image", img_display );

        // Stereo Processing
        stereo( &depthImage16, triclopsInput );
        unsigned short disparity;
        float xx, yy, zz;
        
        for( int x = 0; x < depthImage16.ncols; x++ ) {
            for( int y = 0; y < depthImage16.nrows; y++ ) {
                disparity = *(unsigned short*)((BYTE*)depthImage16.data + depthImage16.rowinc * y + x * 2 );
                triclopsRCD16ToXYZ( triclops, y, x, disparity, &xx, &yy, &zz );
                if( disparity >= 0xff00 ) {
                    zz = 0.0f;
                    xx = 0.0f;
                    yy = 0.0f;
                } else {
                    cnt.at<unsigned char>( y, x ) = cnt.at<unsigned char>( y, x ) + 1;
                }
                img_background.at<float>( y, x ) += zz;
            }
        }
    }

    // Stop capturing
    fe = flycaptureStop( flycapture );
    _HANDLE_FLYCAPTURE_ERROR( "flycaptureStop()", fe );
    cout << endl << "Capturing stopped." << endl;

    for( int x = 0; x < img_background.cols; ++x ) {
        for( int y  = 0; y < img_background.rows; ++y ) {
            img_background.at<float>( y, x ) = img_background.at<float>( y, x ) / (float)(cnt.at<unsigned char>( y, x ));
        }
    }

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

int main( int argc, char *argv[] )
{
    bool ret;

    // Initialize Bumblebee
    ret = InitializeBumblebee();
    if( ret == false ) {
        cout << "Camera initialization failed." << endl;
        exit( 1 );
    }

    //
    // Change camera parameters
    ret = SetBumblebeeParameteres( width, height );
    if( ret == false ) {
        cout << "Parameters setting failed." << endl;
        exit( 1 );
    }
    
    //
    // Command Prompt
    string strCmd;
    while( 1 ) {
        cout << ">";
        cin >> strCmd;
        if( strCmd == "run" ) {
            execute();
        } else if( strCmd == "update" ) {
            update_background( 20 );
        } else if( strCmd == "quit" || strCmd == "exit" ) {
            break;
        } else {
            cout << "Unkown command." << endl;
        }
    }

    // Close the camera
    CloseBumblebee();

    if( rowIntColor ) {
        delete [] rowIntColor;
    }
    
    if( rowIntMono ) {
        delete [] rowIntMono;
    }

    return 0;
}