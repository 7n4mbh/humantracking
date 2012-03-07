#include <iostream>
#include <sstream>

#include "FlyCapture2.h"
#include "triclops.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "FrameRateCounter.h"

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

int iMaxCols = 1280, iMaxRows = 960;
int width = 512, height = 384;

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

	//TriclopsImage tmp;
	//te = triclopsGetImage( triclops, TriImg_RECTIFIED, TriCam_REFERENCE, &tmp );
	//_HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );

	//memcpy( pDst->data, tmp.data, tmp.nrows * tmp.rowinc );
}

void execute()
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

	FrameRateCounter framerate;
	framerate.SetFrameRate( 0.0 );

	Image rawImage;
	unsigned char* buffer = new unsigned char[ iMaxCols * 2 * iMaxRows ];
	Image deinterlacedImage( iMaxRows, iMaxCols * 2, iMaxCols * 2, buffer, iMaxRows * iMaxCols * 2, PIXEL_FORMAT_RAW8, GBRG );
	Mat img( iMaxRows, iMaxCols * 2, CV_8UC4 );
	Mat img_display( height, width, CV_8U );
	//Mat img_display( height, width * 2, CV_8UC4 );
	Image convertedImage( img.data, 4 * iMaxCols * 2 * iMaxRows );//img.rows() * img.cols() * 3 );
	TriclopsImage16 depthImage16;
	Mat img_disparity( height, width, CV_32F );
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
		framerate.NewFrame();
		cout << "Frame Rate: " << framerate.GetFrameRate() << endl;
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

		//resize( img, img_display, img_display.size(), 0, 0 );
		//imshow( "image", img_display );

		// Stereo Processing
		TriclopsError te;
		TriclopsInput triclopsInput;
		//triclopsInput.ncols = iMaxCols;
		//triclopsInput.nrows = iMaxRows;
		//triclopsInput.rowinc = iMaxCols * 2 * 4;
		//triclopsInput.inputType = TriInp_RGB_32BIT_PACKED;
		//triclopsInput.u.rgb32BitPacked.data = convertedImage.GetData();

		int imageCols = iMaxCols;//convertedImage.GetCols();// flycaptureImage.iCols;
		int imageRows = iMaxRows;//convertedImage.GetRows();// flycaptureImage.iRows;
		int imageRowInc = rawImage.GetStride();// flycaptureImage.iRowInc;
		int iSideBySideImages = 2;//flycaptureImage.iNumImages;
		unsigned long timeStampSeconds = convertedImage.GetTimeStamp().seconds;// flycaptureImage.timeStamp.ulSeconds;
		unsigned long timeStampMicroSeconds = convertedImage.GetTimeStamp().microSeconds;//flycaptureImage.timeStamp.ulMicroSeconds;

		// Pointers to positions in the mono buffer that correspond to the beginning
		// of the red, green and blue sections
		unsigned char* redMono = NULL;
		unsigned char* greenMono = NULL;
		unsigned char* blueMono = NULL;

		redMono = deinterlacedImage.GetData();//rowIntMono;
		//if (flycaptureImage.iNumImages == 2)
		//{
		   greenMono = redMono + imageCols;
		   blueMono = redMono + imageCols;
		//}
		//if (flycaptureImage.iNumImages == 3)
		//{
		//   greenMono = redMono + imageCols;
		//   blueMono = redMono + ( 2 * imageCols );
		//}

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

		//stereo( &img_display, triclopsInput );
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

        img_disparity.convertTo( img_display, CV_8U, 25.0, 0.0 );
        imshow( "image", img_display );
		//cout << endl;

	}

	err = bumblebee.StopCapture();
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

	SetBumblebeeParameteres( width, height );

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