#include <iostream>
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
char* szCalFile;
unsigned long ulDevice = 0;

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

int main( int argc, char *argv[] )
{
    FlyCaptureError fe;
    bool ret;

    // Initialize Bumblebee
    ret = InitializeBumblebee();
    if( ret == false ) {
        cout << "Camera initialization failed." << endl;
        exit( 1 );
    }

    //
    // Change the camera parameters

    cout << endl;
    cout << "Changing the parameters..." << endl; 

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


    // Start capturing
    // "DCAM Format 7"規格を用いた画像取得プロセスを開始
    FlyCapturePixelFormat pixelFormat = FLYCAPTURE_RAW16;
    unsigned int width = 1280, height = 960;
    fe = flycaptureStartCustomImage( flycapture, 3, 0, 0, width, height, 100, pixelFormat );
    _HANDLE_FLYCAPTURE_ERROR( "flycaptureStart()", fe );
    cout << endl << "Capturing Started. The image size is " << width << "x" << height << endl;

    FlyCaptureImage _fly_img;
    cv::Mat img( width * 2, height, CV_8U );
    FlyCaptureImage flycaptureImage;
    while( 1 ) {
        // Grab an image from the camera
        // 最新の画像を取得（この関数は画像取得可能になるまで待機する）
        fe = flycaptureGrabImage2( flycapture, &flycaptureImage );
        _HANDLE_FLYCAPTURE_ERROR( "flycaptureGrabImage()", fe );

        // Convert the pixel interleaved raw data to row interleaved format
        // pixel interleaved形式である生データをrow interleaved形式に変換し，
        // tempImageに格納。
        // pixel interleaved形式:ピクセルごとに「左･右･左･右･･･」となっている。
        // row interleaved形式:左右画像が並べられたもの。行ごとに「左･右･左･右･･･」となっている。
        
        _fly_img.pData = img.data;
        fe = flycapturePrepareStereoImage( flycapture, flycaptureImage, &_fly_img, NULL );
        _HANDLE_FLYCAPTURE_ERROR( "flycapturePrepareStereoImage()", fe );
    }


    // Stop capturing
    fe = flycaptureStop( flycapture );
    _HANDLE_FLYCAPTURE_ERROR( "flycaptureStop()", fe );

    // Close the camera
    CloseBumblebee();

    return 0;
}