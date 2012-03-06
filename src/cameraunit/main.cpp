#include <iostream>
#include <sstream>

#include "FlyCapture2.h"
#include "triclops.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

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

using namespace std;
using namespace FlyCapture2;
using namespace cv;

Camera bumblebee;
CameraInfo camInfo;
TriclopsContext triclops;

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
    oss << "calibration" << camInfo.serialNumber << ".txt";
	te = triclopsGetDefaultContextFromFile( &triclops, (char*)oss.str().c_str() );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetDefaultContextFromFile()", te );

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

void execute()
{
	Error err;

    // Start capturing images
	err = bumblebee.StartCapture();
    if( err != PGRERROR_OK ) {
        PrintError( err );
        exit( 1 );
    }

	Image rawImage;
	unsigned char* buffer = new unsigned char[ 1280 * 2 * 960 ];
	Image deinterlacedImage( 960, 1280 * 2, 1280 * 2, buffer, 960 * 1280 * 2, PIXEL_FORMAT_RAW8, GBRG );
	Mat img( 960, 1280 * 2, CV_8UC4 ), img_display( 384, 512 * 2, CV_8UC4 );
	Image convertedImage( img.data, 4 * 1280 * 2 * 960 );//img.rows() * img.cols() * 3 );
	while( true ) {
        // Exit when ESC is hit.
        char c = cvWaitKey( 1 );
        if ( c == 27 ) {
            break;
        }

		// Retrieve an image
        err = bumblebee.RetrieveBuffer( &rawImage );
        if( err != PGRERROR_OK ) {
            PrintError( err );
            continue;
        }
		//cout << "Raw image: ";
		//PrintImageInformation( rawImage );
		//cout << endl;

		// de-interlace
		for( int x = 0; x < 1280; ++x ) {
			for( int y = 0; y < 960; ++y ) {
				const unsigned char* data = rawImage.GetData();
				// left
				buffer[ x + 2560 * y ] = data[ 2 * x + 2560 * y ];
				// right
				buffer[ 1280 + x + 2560 * y ] = data[ 2 * x + 1 + 2560 * y ];
			}
		}

        // Create a converted image
        //Image convertedImage;

        // Convert the raw image
		err = deinterlacedImage.Convert( PIXEL_FORMAT_BGRU, &convertedImage );
        if( err != PGRERROR_OK ) {
            PrintError( err );
            exit( 1 );
        }  

		//cout << "Converted image: ";
		//PrintImageInformation( convertedImage );
		//cout << endl;

		resize( img, img_display, img_display.size(), 0, 0 );
		imshow( "image", img_display );

		//cout << endl;
	}

	destroyWindow( "image" );

	delete [] buffer;
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

	FC2Config config; // debug
	bumblebee.GetConfiguration( &config ); // debug

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