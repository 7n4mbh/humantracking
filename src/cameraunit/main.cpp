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
TriclopsContext triclops;

char* szCalFile;
unsigned long ulDevice = 0;

unsigned char* rowIntColor = NULL;

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
    
   
/*
    // rectify the color image
    te = triclopsRectifyPackedColorImage( triclops, 
			       TriCam_REFERENCE, 
			       &colorInput, 
			       &colorImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectifyPackedColorImage()", te );

    memcpy( pDst->data, colorImage.data, colorImage.rowinc * colorImage.nrows );
*/
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
    // Change camera parameters

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
    
    cout << endl << "Done." << endl;
    

    // Start capturing
    // "DCAM Format 7"規格を用いた画像取得プロセスを開始
    FlyCapturePixelFormat pixelFormat = FLYCAPTURE_RAW16;
    unsigned int width = 1280, height = 960;
    fe = flycaptureStartCustomImage( flycapture, 3, 0, 0, width, height, 100, pixelFormat );
    _HANDLE_FLYCAPTURE_ERROR( "flycaptureStart()", fe );
    cout << endl << "Capturing Started. The image size is " << width << "x" << height << endl;

    TriclopsInput colorInput;
    Mat img( height, width * 2, CV_8UC4 ), imgdisp( 240, 320 * 2, CV_8UC4 );
    DWORD t, tref;
    int framecnt = 0;
    while( 1 ) {
        if( framecnt == 0 ) {
            tref = timeGetTime();
        }

        // Fetch an image
        grabcolor( &colorInput );
        memcpy( img.data, colorInput.u.rgb32BitPacked.data, colorInput.rowinc * colorInput.nrows );
        ++framecnt;

        resize( img, imgdisp, imgdisp.size(), 0, 0 );
        imshow( "image", imgdisp );

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


    // Stop capturing
    fe = flycaptureStop( flycapture );
    _HANDLE_FLYCAPTURE_ERROR( "flycaptureStop()", fe );

    // Close the camera
    CloseBumblebee();

    if( rowIntColor ) {
        delete [] rowIntColor;
    }

    return 0;
}