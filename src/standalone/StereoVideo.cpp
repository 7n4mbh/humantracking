#include <iostream>
#include <fstream>
#include <sstream>
#include <map>

#include "FlyCapture2.h"
#include "triclops.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "../humantracking.h"
#include "StereoVideo.h"

extern float roi_width, roi_height;
extern float roi_x, roi_y;
extern float scale_m2px, scale_m2px_silhouette;

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
using namespace cv;

void getfilename( const string src, string* p_str_path, string* p_str_name, string* p_str_noextname );

StereoVideo::StereoVideo()
{
    H.create( 3, 4, CV_32F );
    frame = -1;
    width = 512;
    height = 384;
}

StereoVideo::~StereoVideo()
{
}

bool StereoVideo::load_extrinsic_parameters()
{
    ifstream ifs;

    string strPath, strName, strNoextName;
    
    //if( flgVideoFile ) {
        getfilename( strVideoFile, &strPath, &strName, &strNoextName );
    //}

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

    cout << "Loaded the extrinsic parameters from " << oss.str() << endl;
    cout << H.at<float>( 0, 0 ) << "," << H.at<float>( 0, 1 ) << "," << H.at<float>( 0, 2 ) << "," << H.at<float>( 0, 3 ) << endl;
    cout << H.at<float>( 1, 0 ) << "," << H.at<float>( 1, 1 ) << "," << H.at<float>( 1, 2 ) << "," << H.at<float>( 1, 3 ) << endl;
    cout << H.at<float>( 2, 0 ) << "," << H.at<float>( 2, 1 ) << "," << H.at<float>( 2, 2 ) << "," << H.at<float>( 2, 3 ) << endl;

    return true;

    //H.at<float>( 0, 0 ) = 0.556880; H.at<float>( 0, 1 ) = -0.326309; H.at<float>( 0, 2 ) = 0.763811; H.at<float>( 0, 3 ) = -1.818401435;
    //H.at<float>( 1, 0 ) = -0.829754; H.at<float>( 1, 1 ) = -0.259884; H.at<float>( 1, 2 ) = 0.493932; H.at<float>( 1, 3 ) = -1.335204279;
    //H.at<float>( 2, 0 ) = 0.037327; H.at<float>( 2, 1 ) = -0.908836; H.at<float>( 2, 2 ) = -0.415480; H.at<float>( 2, 3 ) = 2.237207354;
}

bool StereoVideo::SetStereoParameters( int width, int height, bool flg_output_msg ) {

    TriclopsError te;

    this->width = width;
    this->height = height;
    img_background.create( height, width, CV_32F );
    img_background_cam.create( height, width, CV_32F );
    image_rectified.create( height, width, CV_8U );
    image_occupancy.create( (int)( scale_m2px * roi_height ), (int)( scale_m2px * roi_width ), CV_8U );
    image_depth.create( height, width, CV_8U );

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

bool StereoVideo::load_background()
{
    ifstream ifs;
    string strPath, strName, strNoextName;
    
    //if( flgVideoFile ) {
        getfilename( strVideoFile, &strPath, &strName, &strNoextName );
    //}

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

bool StereoVideo::init( std::string strVideoFile )
{
    TriclopsError te;

    this->strVideoFile = strVideoFile;

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

bool StereoVideo::grab()
{
    if( !video.read( image ) ) {
        return false;
    }
    
    ++frame;

    return true;
}

void StereoVideo::stereo( TriclopsImage16* pDst, Mat* pDstRefImg, const Mat& src )
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

void StereoVideo::create_pepmap()
{
    const size_t len_compress_buf = (int)( roi_height * scale_m2px ) * (int)( roi_width * scale_m2px ) * 2;
    unsigned char* compress_buf = new unsigned char[ len_compress_buf ];
    //const size_t len_compress_buf_geometry = width * height * 2;
    //unsigned char* compress_buf_geometry = new unsigned char[ len_compress_buf_geometry ];
    
    //Mat image;
    Mat img_display( height, width, CV_8U );
    //Mat img_occupancy( (int)( scale_m2px * roi_height ), (int)( scale_m2px * roi_width ), CV_8U );
    //Mat img_display2( (int)( roi_height * 80 ), (int)( roi_width * 80 ), CV_8U );
    TriclopsImage16 depthImage16;
    Mat img_depth( height, width, CV_32F );
    Mat img_camera( height, width, CV_8U );
    Mat img_gray( image.size(), CV_8U );

    // Stereo Processing
    //clock_t t = clock();
    cvtColor( image, img_gray, CV_BGR2GRAY );
    stereo( &depthImage16, &img_camera, img_gray );
    img_camera.copyTo( image_rectified );
    //cout << (double)( clock() - t ) / (double)CLOCKS_PER_SEC << "[sec]" << endl;
    //cout << "done." << endl << flush;

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

    // 
    // Create an occupancy map with foreground data
    // （ベクトル計算で高速化の余地あり）
    /*Mat*/ occupancy = Mat::zeros( (int)( roi_height * scale_m2px ), (int)( roi_width * scale_m2px ), CV_16U );
    //Mat occupancy_2 = Mat::zeros( (int)( 3.0 * scale_m2px ), (int)( roi_height * scale_m2px ), CV_16U );
    //Mat occupancy_3 = Mat::zeros( (int)( roi_height * scale_m2px ), (int)( roi_width * scale_m2px ), CV_16U );
    //Mat geometry = Mat::zeros( height, width, CV_16U );
    //Mat geometry_2 = Mat::zeros( height, width, CV_16U );
    Mat disparitymap = Mat::zeros( height, width, CV_16U );
    int row, col;
    if( 1/*!flgNullPEPMap*/ ) {
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
                    //geometry.at<unsigned short>( y, x ) = 0;
                    //geometry_2.at<unsigned short>( y, x ) = 0;
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
                        //geometry.at<unsigned short>( y, x ) = row * occupancy.cols + col + 1;
                            
                        //col = row; // X axis is projected on the horizontal axis of the geometry map2.
                        //row = (int)( occupancy_2.rows - scale_m2px * pv_z ); // Z axis is projeted on the vertical axis of the geometry map2.
                        //col = (int)( scale_m2px_silhouette * ( ( pv_x - roi_x ) + roi_height / 2.0f ) ); // X axis is projected on the horizontal axis of the geometry map2.
                        //row = (int)( ( 3.0 * scale_m2px_silhouette ) - scale_m2px_silhouette * pv_z ); // Z axis is projeted on the vertical axis of the geometry map2.
                        //if( row >= 0 && row < ( 3.0 * scale_m2px_silhouette ) && col >= 0 && col < ( scale_m2px_silhouette * roi_height ) ) {
                        //    //occupancy_2.at<unsigned short>( row, col ) = occupancy_2.at<unsigned short>( row, col ) + 1;
                        //    geometry_2.at<unsigned short>( y, x ) = row * ( scale_m2px_silhouette * roi_height ) + col + 1;
                        //} else {
                        //    geometry_2.at<unsigned short>( y, x ) = 0;
                        //}

                        //col = (int)( scale_m2px * ( ( pv_x - roi_x ) + roi_width / 2.0f ) );
                        //if( row >= 0 && row < occupancy_3.rows && col >= 0 && col < occupancy_3.cols ) {
                        //    occupancy_3.at<unsigned short>( row, col ) = occupancy_3.at<unsigned short>( row, col ) + 1;
                        //}

                    } else {
                        img_depth.at<float>( y, x ) = 0.0f;
                        //geometry.at<unsigned short>( y, x ) = 0;
                        //geometry_2.at<unsigned short>( y, x ) = 0;
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

        GaussianBlur( occupancy, occupancy, Size( 7, 7 ), 1.5 );
        for( int row = 0; row < occupancy.rows; ++row ) {
            for( int col = 0; col < occupancy.cols; ++col ) {
                if( occupancy.at<unsigned short>( row, col ) < /*10*/50 ) {
                    occupancy.at<unsigned short>( row, col ) = 0;
                }
            }
        }
    }

    image_occupancy.create( (int)( scale_m2px * roi_height ), (int)( scale_m2px * roi_width ), CV_8U );
    image_depth.create( height, width, CV_8U );
    occupancy.convertTo( image_occupancy, CV_8U );
    img_depth.convertTo( image_depth, CV_8U, 25.0, 0.0 );
}



