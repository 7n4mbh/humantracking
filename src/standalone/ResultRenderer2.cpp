#include "../humantracking.h"
#include "ResultRenderer2.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "zlib.h"

using namespace std;
using namespace cv;

extern bool flgCompatible;

extern float roi_width, roi_height;
extern float roi_x, roi_y;
extern float scale_m2px, scale_m2px_silhouette;

extern const int stereo_width = 512, stereo_height = 384;

extern bool flgSegmentationComparisonMode;

void getfilename( const string src, string* p_str_path, string* p_str_name, string* p_str_noextname );

ResultRenderer2::ResultRenderer2()
{
}

ResultRenderer2::~ResultRenderer2()
{
}

void ResultRenderer2::init( std::string result_pepmapvideo_filename, std::string result_pepmapvideo_widthout_region_filename, std::string resut_trajectoryvideo_filename, std::string result_cameravideo_filename, double fps/*, std::string result_cameravideo_filename, std::string silhouette_path*/ )
{
    bufPEPMap.clear();
    trackingResult.clear();
    trackingResultExt.clear();
    result_buffer.clear();
    id_to_trace.clear();
    image_silhouette.clear();

    time_video = 0;
    flgFirst = true;

    strResultPEPMapVideoFilename = result_pepmapvideo_filename;
    strResultPEPMapWithoutRegionFilename = result_pepmapvideo_widthout_region_filename;
    strResultTrajectoryVideoFilename = resut_trajectoryvideo_filename;
    strResultCameraVideoFilename = result_cameravideo_filename;
    this->fps = fps;

    string strName, strNoextName;
    getfilename( strResultPEPMapVideoFilename, &strVideoFilePath, &strName, &strNoextName );

    if( !pepmapVideoWriter.open( strResultPEPMapVideoFilename, CV_FOURCC('X','V','I','D'), fps, Size( (int)( roi_width * 80 ), (int)( roi_height * 80 ) ) ) ) {
        cerr << "Couldn't open " << strResultPEPMapVideoFilename << "." <<  endl;
        exit( 1 );
    }
    
    if( !pepmapWithoutRegionVideoWriter.open( strResultPEPMapWithoutRegionFilename, CV_FOURCC('X','V','I','D'), fps, Size( (int)( roi_width * 80 ), (int)( roi_height * 80 ) ) ) ) {
        cerr << "Couldn't open " << strResultPEPMapWithoutRegionFilename << "." <<  endl;
        exit( 1 );
    }

    if( !trajectoryVideoWriter.open( strResultTrajectoryVideoFilename, CV_FOURCC('X','V','I','D'), fps, Size( (int)( roi_width * 80 ), (int)( roi_height * 80 ) ) ) ) {
        cerr << "Couldn't open " << strResultTrajectoryVideoFilename << "." <<  endl;
        exit( 1 );
    }

    if( !cameraVideoWriter.open( strResultCameraVideoFilename, CV_FOURCC('X','V','I','D'), fps, Size( (int)stereo_width * 2, (int)stereo_height * 2 ) ) ) {
        cerr << "Couldn't open " << strResultCameraVideoFilename << "." <<  endl;
        exit( 1 );
    }        

    ostringstream oss;
#ifdef WINDOWS_OS
    oss << strVideoFilePath << "result_silhouette\\";
#endif
#ifdef LINUX_OS
    oss << strVideoFilePath << "result_silhouette/";
#endif
    gesture.init( oss.str() );
}

void ResultRenderer2::AddPEPMapInfo( PEPMapInfoEx& pepmap )
{
    ostringstream oss;
    oss << pepmap.timeStamp << "_" << pepmap.serialNumber;
    bufPEPMap[ oss.str() ] = pepmap;
    bufTimeStamp.insert( oss.str() );
}

void ResultRenderer2::AddCameraImageInfo( CameraImageInfoEx& cam_image )
{
    ostringstream oss;
    oss << cam_image.timeStamp << "_" << cam_image.serialNumber;
    bufCameraImage[ oss.str() ] = cam_image;
    bufTimeStamp.insert( oss.str() );
}

void ResultRenderer2::AddGeometryMapInfo( GeometryMapInfoEx& geometry )
{
    ostringstream oss;
    oss << geometry.timeStamp << "_" << geometry.serialNumber;
    bufGeometry[ oss.str() ] = geometry;
    bufTimeStamp.insert( oss.str() );
}

void ResultRenderer2::AddSilhouetteMapInfo( GeometryMapInfoEx& silhouette )
{
    ostringstream oss;
    oss << silhouette.timeStamp << "_" << silhouette.serialNumber;
    bufSilhouette[ oss.str() ] = silhouette;
    bufTimeStamp.insert( oss.str() );
}

void ResultRenderer2::AddResultTrajectories( const std::map< unsigned long long, std::map<int,cv::Point2d> >& result, const std::map<unsigned long long, std::multimap<int,cv::Point2d> >& ext_result )
{
    {
        map< unsigned long long, map<int,Point2d> >::const_iterator it = result.begin();
        for( ; it != result.end(); ++it ) {
            trackingResult[ it->first ].insert( it->second.begin(), it->second.end() );
        }
    }

    {
        std::map<unsigned long long, std::multimap<int,cv::Point2d> >::const_iterator it = ext_result.begin();
        for( ; it != ext_result.end(); ++it ) {
            trackingResultExt[ it->first ].insert( it->second.begin(), it->second.end() );
        }
    }
}

void ResultRenderer2::Render()
{
    Mat image_camera( (int)stereo_height, (int)stereo_width, CV_8U );
    Mat image_occupancy_gray( (int)( scale_m2px * roi_height ), (int)( scale_m2px * roi_width ), CV_8U );
    Mat image_occupancy_without_region_record;
    Mat image_trajectory_record( (int)( scale_m2px * roi_height ), (int)( scale_m2px * roi_width ), CV_8UC3 );

    const unsigned long long time_render_begin = trackingResult.begin()->first;
    const unsigned long long time_render_end = trackingResult.rbegin()->first;

    if( flgFirst ) {
        time_video = time_render_begin;
        time_start = time_render_begin;
        frame = 0;
    }

    while( time_video <= time_render_end ) {
    cout << "time_video=" << time_video << endl << flush;
    bool flgOccupancyMapUpdated = false;
        for( set<string>::iterator  itTimeStamp_SerialNumber = bufTimeStamp.begin(); itTimeStamp_SerialNumber != bufTimeStamp.end(); ) {
            cout << "itTimeStamp_SerialNumber=" << *itTimeStamp_SerialNumber << endl;
            int idxDelimiter = itTimeStamp_SerialNumber->find( '_' );
            string strTimeStamp = itTimeStamp_SerialNumber->substr( 0, idxDelimiter );
            string strSerialNumber = itTimeStamp_SerialNumber->substr( idxDelimiter + 1, itTimeStamp_SerialNumber->length() - idxDelimiter - 1 );
            unsigned long long serialNumber = (unsigned long long)atoi( strSerialNumber.c_str() );
            unsigned long long timestamp;
            istringstream iss( strTimeStamp );
            iss >> timestamp;

            if( timestamp > time_video ) {
                break;
            }
        
            cout << "timestamp=" << timestamp << ", serialNumber=" << serialNumber << endl << flush;
         

            // 時刻timestamp[usec],シリアルナンバーserialNumberのpepmap, cam_image, geometry, silhouetteのレンダリングを行う
            map<int,Point2d> posHuman = trackingResult.lower_bound( timestamp )->second;
            multimap<int,Point2d> regionHuman = trackingResultExt.lower_bound( timestamp )->second;
            map<string,PEPMapInfoEx>::iterator itPEPMap = bufPEPMap.find( *itTimeStamp_SerialNumber );
            map<string,CameraImageInfoEx>::iterator itCameraImage = bufCameraImage.find( *itTimeStamp_SerialNumber );
            map<string,GeometryMapInfoEx>::iterator itGeometryMap = bufGeometry.find( *itTimeStamp_SerialNumber );
            map<string,GeometryMapInfoEx>::iterator itSilhouetteMap = bufSilhouette.find( *itTimeStamp_SerialNumber );

            // カメラ画像更新
            cout << "Updating a camera image..." << flush;
            if( itCameraImage != bufCameraImage.end() ) {
                image_camera_record[ serialNumber ].create( itCameraImage->second.image.size(), CV_8UC3 );
                cvtColor( itCameraImage->second.image, image_camera_record[ serialNumber ], CV_GRAY2BGR );
            } else {
                image_camera_record[ serialNumber ] = Scalar( 0, 0, 0 );
            }
            cout << "done." << endl << flush;

            // Occupancy Map画像更新
            cout << "Updating an occupancy map..." << flush;
            if( itPEPMap != bufPEPMap.end() ) {
                itPEPMap->second.occupancy.convertTo( image_occupancy_gray, CV_8U );
                cvtColor( image_occupancy_gray, image_occupancy_record, CV_GRAY2BGR );
                flgOccupancyMapUpdated = true;
            } else {
                image_occupancy_record = Scalar( 0, 0, 0 );
            }
            image_occupancy_without_region_record = image_occupancy_record.clone();
            cout << "done." << endl << flush;

            cout << "Drawing human regions..." << flush;
            // Occupancy Map上に人物領域を描画   
            image_trajectory_record = Scalar( 255, 255, 255 );
            map<int,int> geometry_to_ID;
            if( !flgSegmentationComparisonMode ) {
                for( multimap<int,Point2d>::iterator itHuman = regionHuman.begin(); itHuman != regionHuman.end(); ++itHuman ) {
                    int _row_on_pepmap = scale_m2px * ( ( itHuman->second.x - roi_x ) + roi_height / 2.0f );
                    int _col_on_pepmap = scale_m2px * ( ( itHuman->second.y - roi_y ) + roi_width / 2.0f );
                    for( int col_on_pepmap = max( _col_on_pepmap - 2, 0 ); col_on_pepmap < min( _col_on_pepmap + 2, itPEPMap->second.occupancy.cols - 1 ); ++col_on_pepmap ) {
                        for( int row_on_pepmap = max( _row_on_pepmap - 2, 0 ); row_on_pepmap < min( _row_on_pepmap + 2, itPEPMap->second.occupancy.rows - 1 ); ++row_on_pepmap ) {
                            int row = (int)( ( (float)image_occupancy_record.size().height / (float)image_occupancy_record.size().height ) * row_on_pepmap );
                            int col = (int)( ( (float)image_occupancy_record.size().width / (float)image_occupancy_record.size().width ) * col_on_pepmap );
                            int keyval = row_on_pepmap * itPEPMap->second.occupancy.cols + col_on_pepmap + 1;
                            line( image_occupancy_record, Point( col, row ), Point( col, row ), color_table[ itHuman->first % sizeColorTable ] );
                            geometry_to_ID[ keyval ] = itHuman->first;
                        }
                    }
                }
                for( map<int,Point2d>::iterator itHuman = posHuman.begin(); itHuman != posHuman.end(); ++itHuman ) {
                    int row = (int)( ( (float)image_occupancy_record.size().height / (float)image_occupancy_record.size().height ) * scale_m2px * ( ( itHuman->second.x - roi_x ) + roi_height / 2.0f ) );
                    int col = (int)( ( (float)image_occupancy_record.size().width / (float)image_occupancy_record.size().width ) * scale_m2px * ( ( itHuman->second.y - roi_y ) + roi_width / 2.0f ) );
                    circle( image_occupancy_record, Point( col, row ), 1, color_table[ itHuman->first % sizeColorTable ], -1 );
                    circle( image_occupancy_without_region_record, Point( col, row ), 1, color_table[ itHuman->first % sizeColorTable ], -1 );
                    circle( image_trajectory_record, Point( col, row ), 1, color_table[ itHuman->first % sizeColorTable ], -1 );
                    id_to_trace[ itHuman->first ][ time_video ] = Point( col, row );
                }
                for( map< int, std::map<unsigned long long,Point> >::iterator it_id_to_trace = id_to_trace.begin(); it_id_to_trace != id_to_trace.end(); ++it_id_to_trace ) {
                    // 3秒以前の位置情報を削除
                    it_id_to_trace->second.erase( it_id_to_trace->second.begin()
                                                , it_id_to_trace->second.lower_bound( time_video - 3000000 ) );
                    if( !it_id_to_trace->second.empty() ) {
                        vector< vector<Point> > pts( 1 );
                        for( map<unsigned long long,Point>::iterator itPoint = it_id_to_trace->second.begin(); itPoint != it_id_to_trace->second.end(); ++itPoint ) {
                            pts[ 0 ].push_back( itPoint->second );
                        }
			cout << " polylines(): id=" << it_id_to_trace->first << flush << endl;
                        polylines( image_occupancy_without_region_record, pts, false, color_table[ it_id_to_trace->first % sizeColorTable ] );
                        polylines( image_trajectory_record, pts, false, color_table[ it_id_to_trace->first % sizeColorTable ] );
                    }
                }
            } else {
                for( map<int,Point2d>::iterator itHuman = posHuman.begin(); itHuman != posHuman.end(); ++itHuman ) {
                    int row = (int)( ( (float)image_occupancy_record.size().height / (float)image_occupancy_record.size().height ) * scale_m2px * ( ( itHuman->second.x - roi_x ) + roi_height / 2.0f ) );
                    int col = (int)( ( (float)image_occupancy_record.size().width / (float)image_occupancy_record.size().width ) * scale_m2px * ( ( itHuman->second.y - roi_y ) + roi_width / 2.0f ) );
                    circle( image_occupancy_record, Point( col, row ), 7, color_table[ itHuman->first % sizeColorTable ], 1 );
                }
            }

            // カメラ画像上に人物領域を描画。Silhouette作成用のデータも作る。
            map<int,Mat> count_silhouette;
            const int silhouette_width = (int)( scale_m2px_silhouette * roi_height );
            const int silhouette_height = (int)( scale_m2px_silhouette * 3.0 );
            for( int x = 0; x < itGeometryMap->second.geometry.cols; ++x ) {
                for( int y = 0; y < itGeometryMap->second.geometry.rows; ++y ) {
                    const int keyval = itGeometryMap->second.geometry.at<unsigned short>( y, x );
                    int id_assigned = -1;
                    map<int,int>::iterator itID;
                    if( !flgSegmentationComparisonMode && ( itID = geometry_to_ID.find( keyval ) ) != geometry_to_ID.end() ) {
                        line( image_camera_record[ serialNumber ], Point( x, y ), Point( x, y ), color_table[ itID->second % sizeColorTable ], 1 );
                        id_assigned = itID->second;
                    } else if( keyval != 0 ) {
                        const int row_on_pepmap = ( keyval - 1 ) / itPEPMap->second.occupancy.cols;
                        const int col_on_pepmap = ( keyval - 1 ) % itPEPMap->second.occupancy.cols;
                        for( map<int,Point2d>::iterator itHuman = posHuman.begin(); itHuman != posHuman.end(); ++itHuman ) {
                            const int _row_on_pepmap = scale_m2px * ( ( itHuman->second.x - roi_x ) + roi_height / 2.0f );
                            const int _col_on_pepmap = scale_m2px * ( ( itHuman->second.y - roi_y ) + roi_width / 2.0f );
                            const int diff_row = row_on_pepmap - _row_on_pepmap;
                            const int diff_col = col_on_pepmap - _col_on_pepmap;
                            const double dist = sqrt( (double)( diff_row * diff_row + diff_col * diff_col ) );
                            
                            const double adaptive_threshold = ( itHuman->first == 1 ) ? 10 : 7/*6*/; // test code
                            if( dist < ( flgCompatible ? 6 : adaptive_threshold ) ) {
                                line( image_camera_record[ serialNumber ], Point( x, y ), Point( x, y ), color_table[ itHuman->first % sizeColorTable ], 1 );
                                id_assigned = itHuman->first;
                            }
                        }
                    }

                    if( itSilhouetteMap != bufSilhouette.end() && id_assigned >= 1 ) {
                        if( count_silhouette.find( id_assigned ) == count_silhouette.end() ) {
                            count_silhouette[ id_assigned ] = Mat::zeros( silhouette_height, silhouette_width, CV_16U );
                        }
                        const int keyval = itSilhouetteMap->second.geometry.at<unsigned short>( y, x );
                        const int col_on_silhouette = ( keyval - 1 ) % silhouette_width;//p_count_silhouette[ id_assigned ]->cols;
                        const int row_on_silhouette = ( keyval - 1 ) / silhouette_width;//p_count_silhouette[ id_assigned ]->cols;
                        if( col_on_silhouette >= 0 && col_on_silhouette < silhouette_width && row_on_silhouette >= 0 && row_on_silhouette < silhouette_height ) {
                            count_silhouette[ id_assigned ].at<unsigned short>( row_on_silhouette, col_on_silhouette )
                                = count_silhouette[ id_assigned ].at<unsigned short>( row_on_silhouette, col_on_silhouette ) + 1;
                        }
                    }
                }
            }
            cout << "done." << endl << flush;

            // Silhouette作成
            cout << "Slihouette Creation..." << endl << flush;
            if( itSilhouetteMap != bufSilhouette.end() ) {
                for( map<int,Mat>::iterator it_id_to_count = count_silhouette.begin(); it_id_to_count != count_silhouette.end(); ++it_id_to_count ) {
                    const int id = it_id_to_count->first;
                    if( image_silhouette.find( id ) == image_silhouette.end() ) {
                        image_silhouette[ id ];
                        image_silhouette2[ id ];
                        
                        {
                            ostringstream oss;
#ifdef WINDOWS_OS
                            oss << strVideoFilePath << "result_silhouette\\silhouette_" << id << ".avi";
#endif
#ifdef LINUX_OS
                            oss << strVideoFilePath << "result_silhouette/silhouette_" << id << ".avi";
#endif
                            if( !silhouetteVideoWriter[ id ].open( oss.str(), CV_FOURCC('X','V','I','D'), fps, Size( (int)( scale_m2px_silhouette * roi_height ) * 2, (int)( scale_m2px_silhouette * 3.0 ) * 2 ) ) ) {
                                cerr << "Couldn't open " << oss.str() << "." <<  endl;
                                exit( 1 );
                            }   
                        }
			cout << " silhouette_" << id << ".avi prepared." << endl << flush; 
                        {
                            ostringstream oss;
#ifdef WINDOWS_OS
                            oss << strVideoFilePath << "result_silhouette\\silhouette2_" << id << ".avi";
#endif
#ifdef LINUX_OS
                            oss << strVideoFilePath << "result_silhouette/silhouette2_" << id << ".avi";
#endif
                            if( !silhouetteVideoWriter2[ id ].open( oss.str(), CV_FOURCC('X','V','I','D'), fps, Size( (int)( scale_m2px_silhouette * roi_height ) , (int)( scale_m2px_silhouette * 3.0 ) ) ) ) {
                                cerr << "Couldn't open " << oss.str() << "." <<  endl;
                                exit( 1 );
                            }   
			}
			cout << " silhouette2_" << id << ".avi prepared." << endl << flush; 
                        
                    }
		    cout << " image_silhouette[" << id << "] and image_silhouette2[" << id << "]" << flush;
                    if( image_silhouette[ id ].find( serialNumber ) == image_silhouette[ id ].end() ) {
                        image_silhouette[ id ][ serialNumber ].create( silhouette_height, silhouette_width, CV_8U );
                        image_silhouette2[ id ].create( silhouette_height, silhouette_width, CV_8U );
                    }
		    cout << " prepared." << endl << flush;

                    GaussianBlur( count_silhouette[ id ], count_silhouette[ id ], Size( 5, 5 ), 0.75 );
		    cout << " Blur applied." << endl << flush;
		    cout << " Creating image_silhouette[" << id << "]..." << flush;
                    for( int col = 0; col < count_silhouette[ id ].cols; ++col ) {
                        for( int row = 0; row < count_silhouette[ id ].rows; ++row ) {
                            image_silhouette[ id ][ serialNumber ].at<unsigned char>( row, col ) 
                                = ( count_silhouette[ id ].at<unsigned short>( row, col ) > 3 ) ? 255 : 0;
                        }
                    }
		    cout << "done." << flush << endl;
                    //image_silhouette2[ id ] = image_silhouette[ id ][ serialNumber ].clone();
                }
            }
            cout << "done." << endl << flush;

            // 利用済みのデータを削除
            if( itPEPMap != bufPEPMap.end() ) {
                bufPEPMap.erase( itPEPMap );
            }
            if( itCameraImage != bufCameraImage.end() ) {
                bufCameraImage.erase( itCameraImage );
            }
            if( itGeometryMap != bufGeometry.end() ) {
                bufGeometry.erase( itGeometryMap );
            }
            if( itSilhouetteMap != bufSilhouette.end() ) {
                bufSilhouette.erase( itSilhouetteMap );
            }

            bufTimeStamp.erase( itTimeStamp_SerialNumber++ );
        }

        // 動画出力
        if( flgOccupancyMapUpdated && flgCompatible ) {
            Mat tmp( image_occupancy_record.size(), CV_8UC3 );
            Point2d center( image_occupancy_record.cols * 0.5, image_occupancy_record.rows * 0.5 );
            const Mat affine_matrix = getRotationMatrix2D( center, 90.0, 1.0 );
            warpAffine( image_occupancy_record, tmp, affine_matrix, image_occupancy_record.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar::all( 255 ) );
            image_occupancy_record = tmp.clone();
            warpAffine( image_occupancy_without_region_record, tmp, affine_matrix, image_occupancy_without_region_record.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar::all( 255 ) );
            image_occupancy_without_region_record = tmp.clone();
            warpAffine( image_trajectory_record, tmp, affine_matrix, image_trajectory_record.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar::all( 255 ) );
            image_trajectory_record = tmp.clone();
        }
        cout << "pepmapVideoWriter.write()..." << flush;
        pepmapVideoWriter.write( image_occupancy_record );
        pepmapWithoutRegionVideoWriter.write( image_occupancy_without_region_record );
        trajectoryVideoWriter.write( image_trajectory_record );
        cout << "done." << endl << flush;
        Mat image_camera_record_integrated( stereo_height * 2, stereo_width * 2, CV_8UC3 );
        {
            map<unsigned long long,Mat>::iterator it;
            if( ( it = image_camera_record.find( 7420008 ) ) != image_camera_record.end() ) {
                copy( image_camera_record_integrated, 0, 0, it->second, 0, 0, stereo_width, stereo_height );
            }
            if( ( it = image_camera_record.find( 7420015 ) ) != image_camera_record.end() ) {
                copy( image_camera_record_integrated,  stereo_width, 0, it->second, 0, 0, stereo_width, stereo_height );
            }
            if( ( it = image_camera_record.find( 7140019 ) ) != image_camera_record.end() ) {
                copy( image_camera_record_integrated, 0, stereo_height, it->second, 0, 0, stereo_width, stereo_height );
            }
            if( ( it = image_camera_record.find( 7420005 ) ) != image_camera_record.end() ) {
                copy( image_camera_record_integrated, stereo_width, stereo_height, it->second, 0, 0, stereo_width, stereo_height );
            }
            cout << "cameraVideoWriter.write()..." << flush;
            cameraVideoWriter.write( image_camera_record_integrated );
            cout << "done." << endl << flush;
        }
        cout << "silhouettevideoWriter..." << flush;
        Mat image_silhouette_record;
        for( map<int,map<unsigned long long,Mat> >::iterator it_id_serial_silhouette = image_silhouette.begin(); it_id_serial_silhouette != image_silhouette.end(); ++it_id_serial_silhouette ) {
            const int id = it_id_serial_silhouette->first;
            cout << "output silhouette: " << id << endl << flush;
            cout << " initializing..." << flush;
            map<unsigned long long,Mat>& img = it_id_serial_silhouette->second;
            const int w = (int)( scale_m2px_silhouette * roi_height );
            const int h = (int)( scale_m2px_silhouette * 3.0 );
            image_silhouette_record = Mat::zeros( h * 2, w * 2, CV_8UC3 );
            image_silhouette2[ id ] = Mat::zeros( h, w, CV_8U );
            Mat tmp( h, w, CV_8UC3 );
            cout << "done." << flush;
            map<unsigned long long,Mat>::iterator it;
            if( ( it = img.find( 7420008 ) ) != img.end() ) {
                cout << " 7420008...";
                cvtColor( it->second, tmp, CV_GRAY2BGR );
                copy( image_silhouette_record, 0, 0, tmp, 0, 0 ,w, h );
                cout << " adding...";
                image_silhouette2[ id ] += it->second;
                cout << "done." << endl;
            }
            if( ( it = img.find( 7420015 ) ) != img.end() ) {
                cout << " 7420015...";
                cvtColor( it->second, tmp, CV_GRAY2BGR );
                copy( image_silhouette_record,  w, 0, tmp, 0, 0, w, h );
                cout << " adding...";
                image_silhouette2[ id ] += it->second;
                 cout << "done." << endl;
            }
            if( ( it = img.find( 7140019 ) ) != img.end() ) {
                cout << " 7140019...";
                cvtColor( it->second, tmp, CV_GRAY2BGR );
                copy( image_silhouette_record, 0, h, tmp, 0, 0, w, h );
                cout << " adding...";
                image_silhouette2[ id ] += it->second;
                cout << "done." << endl;
            }
            if( ( it = img.find( 7420005 ) ) != img.end() ) {
                cout << " 7420005...";
                cvtColor( it->second, tmp, CV_GRAY2BGR );
                copy( image_silhouette_record, w, h, tmp, 0, 0, w, h );
                cout << " adding...";
                image_silhouette2[ id ] += it->second;
                cout << "done." << endl;
            }
            cout << "silhouetteVideoWriter.write()..." << flush;
            silhouetteVideoWriter[ id ].write( image_silhouette_record );
            cout << "done." << endl << flush;
            cvtColor( image_silhouette2[ id ], tmp, CV_GRAY2BGR );
            cout << "Gesture Recognition..." << flush;
            gesture.clear_silhouette( id );
            gesture.set_silhouette( id, time_video, tmp );
            gesture.recognize( id, time_video );
            if( gesture.status[ id ] ) {
                rectangle( tmp
                         , Point( 0, 0 )
                         , Point( 10, 10 )
                         , color_table[ 1 % sizeColorTable ]
                         , CV_FILLED );
            }
            cout << "done." << endl << flush;
            cout << "silhouetteVideoWriter2.write()..." << flush;
            silhouetteVideoWriter2[ id ].write( tmp );
            cout << "done." << endl << flush;
        }
        cout << "done." << endl;

        cout << "imshow...";
        imshow( "Tracking Result", image_occupancy_record );
        imshow( "Segmentation", image_camera_record_integrated );
        (void)cvWaitKey( 10 );
        cout << "done." << endl;

        cout << "finalizing..." << flush;
        ++frame;
        time_video = time_start + ( ( 1000000ULL  * (unsigned long long)frame ) / (unsigned long long)fps );
        cout << "done."<< endl << flush;
    }

    trackingResult.clear();
    trackingResultExt.clear();
    flgFirst = false;
}
