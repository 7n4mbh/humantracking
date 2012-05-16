#include <fstream>

#include "../humantracking.h"
#include "TrackingResultResources.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "zlib.h"

using namespace std;
using namespace cv;

extern float roi_width, roi_height;
extern float roi_x, roi_y;
extern float scale_m2px;

TrackingResultResources::TrackingResultResources()
{
    bRunThread = false;
    pViewer = NULL;
#ifdef WINDOWS_OS
    hThread = NULL;
    InitializeCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_init( &mutex, NULL );
#endif
}

TrackingResultResources::~TrackingResultResources()
{
#ifdef WINDOWS_OS
    DeleteCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_destroy( &mutex );
#endif
}

void TrackingResultResources::init( std::string result_filename, std::string result_pepmapvideo_filename, std::string result_cameravideo_filename, std::string silhouette_path, Viewer* p_viewer )
{
    bufPEPMap.clear();
    trackingResult.clear();
    trackingResultExt.clear();
    nUpdateViewRequest = 0;
    delayUpdate = 0;
    posHumanStill.clear();
    cntStill.clear();
    strResultFilename = result_filename;
    strResultPEPMapVideoFilename = result_pepmapvideo_filename;
    strResultCameraVideoFilename = result_cameravideo_filename;
    strSilhouettePath = silhouette_path;
    pViewer = p_viewer;
    ofstream ofs( strResultFilename.c_str() );
    if( !pepmapVideoWriter.open( strResultPEPMapVideoFilename, CV_FOURCC('X','V','I','D'), 40, Size( (int)( roi_width * 80 ), (int)( roi_height * 80 ) ) ) ) {
      cerr << "Couldn't open " << strResultPEPMapVideoFilename << "." <<  endl;
      exit( 1 );
    }
    if( !cameraVideoWriter.open( strResultCameraVideoFilename, CV_FOURCC('X','V','I','D'), 10, Size( 512, 384 ) ) ) {
      cerr << "Couldn't open " << strResultCameraVideoFilename << "." <<  endl;
      exit( 1 );
    }
    silhouetteVideoWriter.clear();
}

void TrackingResultResources::clear()
{
    bufPEPMap.clear();
    trackingResult.clear();
    trackingResultExt.clear();
    nUpdateViewRequest = 0;
    posHumanStill.clear();
    cntStill.clear();
    ofstream ofs( strResultFilename.c_str() );
    silhouetteVideoWriter.clear();
}

bool TrackingResultResources::hasDataToDisplay()
{
#ifdef WINDOWS_OS
    EnterCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_lock( &mutex );
#endif

    bool ret = !bufPEPMap.empty() && !trackingResult.empty();

#ifdef WINDOWS_OS
    LeaveCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_unlock( &mutex );
#endif

    return ret;
}

void TrackingResultResources::AddResultTrajectories( const std::map< unsigned long long, std::map<int,cv::Point2d> >& result, const std::map<unsigned long long, std::multimap<int,cv::Point2d> >& ext_result )
{
#ifdef WINDOWS_OS
    EnterCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_lock( &mutex );
#endif
    cout << endl << "Received New Results!" << endl;

    ofstream ofs(  strResultFilename.c_str(), ios::out | ios::app );


    {
        map< unsigned long long, map<int,Point2d> >::const_iterator it = result.begin();
        for( ; it != result.end(); ++it ) {
            trackingResult[ it->first ].insert( it->second.begin(), it->second.end() );
            ofs << it->first // Timestamp
                << ", " << it->second.size(); // # of people
            for( map<int,Point2d>::const_iterator itPosHuman = it->second.begin(); itPosHuman != it->second.end(); ++itPosHuman ) {
                ofs << ", " << itPosHuman->first // ID
                    << ", " << itPosHuman->second.x // X
                    << ", " << itPosHuman->second.y; // Y
	        map<int,Point2d>::iterator itPosHumanStill;
	        if( ( itPosHumanStill = posHumanStill.find( itPosHuman->first ) ) != posHumanStill.end() ) {
		    const float dx = itPosHumanStill->second.x - itPosHuman->second.x;
		    const float dy = itPosHumanStill->second.y - itPosHuman->second.y;
		    const float dist = sqrt( dx * dx + dy * dy );
		    if( dist < 0.5f ) {
		        cntStill[ itPosHuman->first ]++;
		    } else {
		        cntStill[ itPosHuman->first ] = 0;
		        itPosHumanStill->second = itPosHuman->second;
		    }
	        } else {
		    posHumanStill[ itPosHuman->first ] = itPosHuman->second;
	        }
	    }
            ofs << endl;
        }
    }

    {
        std::map<unsigned long long, std::multimap<int,cv::Point2d> >::const_iterator it = ext_result.begin();
        for( ; it != ext_result.end(); ++it ) {
            trackingResultExt[ it->first ].insert( it->second.begin(), it->second.end() );
        }
    }

#ifdef WINDOWS_OS
    LeaveCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_unlock( &mutex );
#endif
}

void TrackingResultResources::AddPEPMapInfo( PEPMapInfo& pepmap )
{
#ifdef WINDOWS_OS
    EnterCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_lock( &mutex );
#endif

    bufPEPMap.push_back( pepmap );

#ifdef WINDOWS_OS
    LeaveCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_unlock( &mutex );
#endif
}

void TrackingResultResources::AddCameraImageInfo( CameraImageInfo& cam_image )
{
#ifdef WINDOWS_OS
    EnterCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_lock( &mutex );
#endif

    bufCameraImage[ cam_image.timeStamp ] = cam_image;

#ifdef WINDOWS_OS
    LeaveCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_unlock( &mutex );
#endif
}

void TrackingResultResources::AddGeometryMapInfo( GeometryMapInfo& geometry )
{
#ifdef WINDOWS_OS
    EnterCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_lock( &mutex );
#endif

    bufGeometry[ geometry.timeStamp ] = geometry;

#ifdef WINDOWS_OS
    LeaveCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_unlock( &mutex );
#endif
}

void TrackingResultResources::AddGeometryMap2Info( GeometryMapInfo& geometry2 )
{
#ifdef WINDOWS_OS
    EnterCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_lock( &mutex );
#endif

    bufGeometry2[ geometry2.timeStamp ] = geometry2;

#ifdef WINDOWS_OS
    LeaveCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_unlock( &mutex );
#endif
}

bool TrackingResultResources::EnableViewWindow()
{
#ifdef WINDOWS_OS
    if( hThread != NULL ) {
        return false;
    }
#endif

    nUpdateViewRequest = 0;
    bRunThread = true;

#ifdef WINDOWS_OS
    hThread = CreateThread( NULL, 0, TrackingResultResources::ViewThread, (LPVOID)this, 0, &ThreadId );
    if (hThread == NULL) {
        exit( 1 );
    }
#endif
#ifdef LINUX_OS
    // Launch the thread that gets the input and sends it to the child.
    pthread_create( &thread
                , NULL
                , TrackingResultResources::ViewThread
                , (void*)this );
#endif

    return true;
}

bool TrackingResultResources::TerminateViewWindow()
{
#ifdef WINDOWS_OS
    if( hThread == NULL ) {
        return false;
    }
#endif

    bRunThread = false;

#ifdef WINDOWS_OS
    WaitForSingleObject( hThread, 0 );
    hThread = NULL;
#endif
#ifdef LINUX_OS
    pthread_join( thread, NULL );
#endif
    
    return true;
}

void TrackingResultResources::UpdateView()
{
#ifdef WINDOWS_OS
    EnterCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_lock( &mutex );
#endif

    ++nUpdateViewRequest;

#ifdef WINDOWS_OS
    LeaveCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_unlock( &mutex );
#endif
}

void TrackingResultResources::SetDelayUpdate( int delay_update )
{
    delayUpdate = delay_update;
}

int TrackingResultResources::GetDelayUpdate()
{
    return delayUpdate;
}

#ifdef WINDOWS_OS
DWORD WINAPI TrackingResultResources::ViewThread( LPVOID p_tracking_result_resources )
#endif
#ifdef LINUX_OS
void* TrackingResultResources::ViewThread( void* p_tracking_result_resources )
#endif
{
    TrackingResultResources* pTrackingResultResources = (TrackingResultResources*)p_tracking_result_resources;
    Mat occupancy = Mat::zeros( (int)( roi_height * scale_m2px ), (int)( roi_width * scale_m2px ), CV_16U );
    Mat geometry, geometry2;
    Mat img_occupancy( (int)( scale_m2px * roi_height ), (int)( scale_m2px * roi_width ), CV_8U );
    Mat img_display_tmp( (int)( roi_height * 80 ), (int)( roi_width * 80 ), CV_8UC3 );
    Mat img_display( (int)( roi_height * 80 ), (int)( roi_width * 80 ), CV_8UC3 );
    Mat img_cam_display;
    Mat img_silhouette_display( (int)( scale_m2px * 3.0 ), (int)( scale_m2px * roi_height ), CV_8UC3 );
    char buf[ SIZE_BUFFER ];

    deque< map<int,Point2d> > result_buffer;
    
    while( pTrackingResultResources->bRunThread ) {
#ifdef WINDOWS_OS
      //Sleep( 50 );
            EnterCriticalSection( &pTrackingResultResources->cs );
#endif
#ifdef LINUX_OS
	    //    usleep( 50000 );
            pthread_mutex_lock( &pTrackingResultResources->mutex );
#endif
            bool data_available = !pTrackingResultResources->bufPEPMap.empty() && !pTrackingResultResources->trackingResult.empty();
#ifdef WINDOWS_OS
            LeaveCriticalSection( &pTrackingResultResources->cs );
#endif
#ifdef LINUX_OS 
            pthread_mutex_unlock( &pTrackingResultResources->mutex );
#endif
        if( data_available && ( pTrackingResultResources->GetDelayUpdate() > 0 || pTrackingResultResources->nUpdateViewRequest ) ) {
#ifdef WINDOWS_OS
            EnterCriticalSection( &pTrackingResultResources->cs );
#endif
#ifdef LINUX_OS
            pthread_mutex_lock( &pTrackingResultResources->mutex );
#endif
            PEPMapInfo pepmap = pTrackingResultResources->bufPEPMap.front();
            unsigned long long timeStamp = pTrackingResultResources->trackingResult.begin()->first;
            map<int,Point2d> posHuman = pTrackingResultResources->trackingResult.begin()->second;
            
#ifdef WINDOWS_OS
            LeaveCriticalSection( &pTrackingResultResources->cs );
#endif
#ifdef LINUX_OS
            pthread_mutex_unlock( &pTrackingResultResources->mutex );
#endif
	    //cout << "pepmap.timeStamp=" << pepmap.timeStamp << ", timeStamp=" << timeStamp;
            if( pepmap.timeStamp <= timeStamp ) {
	      //cout << " -> show" << endl;
#ifdef WINDOWS_OS
                EnterCriticalSection( &pTrackingResultResources->cs );
#endif
#ifdef LINUX_OS
                pthread_mutex_lock( &pTrackingResultResources->mutex );
#endif
	            //--pTrackingResultResources->nUpdateViewRequest;
  	            pTrackingResultResources->nUpdateViewRequest = 0;

                pTrackingResultResources->bufPEPMap.pop_front();
#ifdef WINDOWS_OS
                LeaveCriticalSection( &pTrackingResultResources->cs );
#endif
#ifdef LINUX_OS     
                pthread_mutex_unlock( &pTrackingResultResources->mutex );
#endif
                const int size = pepmap.data.size() / 2;

                char a[ 3 ]; a[ 2 ] = '\0';
                for( int j = 0; j < size; ++j ) {
                    a[ 0 ] = pepmap.data[ j * 2 ];
                    a[ 1 ] = pepmap.data[ j * 2 + 1 ];
                    buf[ j ] = strtol( a, NULL, 16 );
                }
                //cout << "Data Received." << endl;
                time_t _sec = pepmap.timeStamp / 1000000ULL;
                string strTime;
		// sometimes, ctime() returns null string for some reason.
		// The following is for re-trying in that case.
                for( int i = 0; i < 10; ++i ) {
                    strTime = string( ctime( &_sec ) );
                    if( strTime.size() ) {
                        break;
                    }
                }
                //cout << "## debug info ## strTime.size=" << strTime.size() 
                //     << ", pepmap.data.size=" << pepmap.data.size() 
                //     << ", pepmap.timeStamp=" << pepmap.timeStamp 
                //     << ", pepmap.serialNumber=" << pepmap.serialNumber << endl;
                strTime.erase( strTime.size() - 1 );
                cout << "Tracking Result(posHuman.size=" << posHuman.size() << "): Serial #: " << pepmap.serialNumber << ", time: " << pepmap.timeStamp << "(" << strTime << ")" << endl;
//#ifdef WINDOWS_OS
    //            ofs_pepmap << "Serial #: " << pepmap.serialNumber << ", time: " << pepmap.timeStamp << "(" << strTime << ")" << endl;
//#endif
    //            ++cnt;
                //if( pepmap.timeStamp < tmp_timestamp ) {
                //    cout << "## Illegal Time Stamp! ## This pepmap will be rejected. To avoid this, have more size of sort_buffer." << endl;
                //    continue;
                //}
                //tmp_timestamp = pepmap.timeStamp;

                uLongf len_uncompressed = (int)( roi_height * scale_m2px ) * (int)( roi_width * scale_m2px ) * 2;
                uncompress( occupancy.data
                , &len_uncompressed
                , (const Bytef*)buf
                , size );

                result_buffer.push_back( posHuman );
                if( result_buffer.size() > 20 ) {
                    result_buffer.pop_front();
                }

                occupancy.convertTo( img_occupancy, CV_8U );
                cv::cvtColor( img_occupancy, img_display_tmp, CV_GRAY2BGR );
                resize( img_display_tmp, img_display, img_display.size() );
                const float scale_width = (float)img_display.size().width / (float)img_display_tmp.size().width;
                const float scale_height = (float)img_display.size().height / (float)img_display_tmp.size().height;

                map<int, vector<Point2d> > trajectory_to_draw;
                for( deque< map<int,Point2d> >::iterator itPosHuman = result_buffer.begin(); itPosHuman != result_buffer.end(); ++itPosHuman ) {
                    for( map<int,Point2d>::iterator it = itPosHuman->begin(); it != itPosHuman->end(); ++it ) {
                        trajectory_to_draw[ it->first ].push_back( it->second );
                    }
                }

                int old_col, old_row;
                for( map< int, vector<Point2d> >::iterator itHuman = trajectory_to_draw.begin(); itHuman != trajectory_to_draw.end(); ++itHuman ) {
                    for( vector<Point2d>::iterator it = itHuman->second.begin(); it != itHuman->second.end(); ++it ) {
                        int row = (int)( ( (float)img_display.size().height / (float)img_display_tmp.size().height ) * scale_m2px * ( ( it->x - roi_x ) + roi_height / 2.0f ) );
                        int col = (int)( ( (float)img_display.size().width / (float)img_display_tmp.size().width ) * scale_m2px * ( ( it->y - roi_y ) + roi_width / 2.0f ) );
                        circle( img_display, Point( col, row ), 3, color_table[ itHuman->first % sizeColorTable ], -1 );
                        if( it != itHuman->second.begin() ) {
                            line( img_display, Point( col, row ), Point( old_col, old_row ), color_table[ itHuman->first % sizeColorTable ], 6 );
                        }
                        old_col = col;
                        old_row = row;
                    }

		            if( pTrackingResultResources->cntStill.find( itHuman->first ) != pTrackingResultResources->cntStill.end() ) {
                        if( pTrackingResultResources->cntStill[ itHuman->first ] > 30 ) {
	                        int row = (int)( ( (float)img_display.size().height / (float)img_display_tmp.size().height ) * scale_m2px * ( ( pTrackingResultResources->posHumanStill[ itHuman->first ].x - roi_x ) + roi_height / 2.0f ) );
	                        int col = (int)( ( (float)img_display.size().width / (float)img_display_tmp.size().width ) * scale_m2px * ( ( pTrackingResultResources->posHumanStill[ itHuman->first ].y - roi_y ) + roi_width / 2.0f ) );
	                        circle( img_display, Point( col, row ), ( (float)img_display.size().width / (float)img_display_tmp.size().width ) * 0.5 * scale_m2px, color_table[ itHuman->first % sizeColorTable ], 2 );
                        }
		            }
                }

                
                map<unsigned long long,multimap<int,Point2d> >::iterator itRegionHuman = pTrackingResultResources->trackingResultExt.find( pepmap.timeStamp );
                map<int,int> geometry_to_ID;
                if( itRegionHuman != pTrackingResultResources->trackingResultExt.end() ) {
                    for( multimap<int,Point2d>::iterator itHuman = itRegionHuman->second.begin(); itHuman != itRegionHuman->second.end(); ++itHuman ) {
                        int _row_on_pepmap = scale_m2px * ( ( itHuman->second.x - roi_x ) + roi_height / 2.0f );
                        int _col_on_pepmap = scale_m2px * ( ( itHuman->second.y - roi_y ) + roi_width / 2.0f );
                        for( int col_on_pepmap = max( _col_on_pepmap - 2, 0 ); col_on_pepmap < min( _col_on_pepmap + 2, occupancy.cols - 1 ); ++col_on_pepmap ) {
                            for( int row_on_pepmap = max( _row_on_pepmap - 2, 0 ); row_on_pepmap < min( _row_on_pepmap + 2, occupancy.rows - 1 ); ++row_on_pepmap ) {
                                int row = (int)( ( (float)img_display.size().height / (float)img_display_tmp.size().height ) * row_on_pepmap );
                                int col = (int)( ( (float)img_display.size().width / (float)img_display_tmp.size().width ) * col_on_pepmap );
                                //int keyval = col_on_pepmap * occupancy.rows + row_on_pepmap + 1;
                                int keyval = row_on_pepmap * occupancy.cols + col_on_pepmap + 1;
                                rectangle( img_display, Point( col - scale_width / 2, row - scale_height / 2 ), Point( col + scale_width / 2, row + scale_height / 2 ), color_table[ itHuman->first % sizeColorTable ], CV_FILLED );
                                geometry_to_ID[ keyval ] = itHuman->first;
                            }
                        }
                    }
                }

                memset( img_silhouette_display.data, 0, img_silhouette_display.rows * img_silhouette_display.cols * 3 );

                map<unsigned long long,CameraImageInfo>::iterator itCamImageInfo = pTrackingResultResources->bufCameraImage.find( pepmap.timeStamp );
                if( itCamImageInfo != pTrackingResultResources->bufCameraImage.end() ) {
                    //img_cam_display.create( itCamImageInfo->second.height, itCamImageInfo->second.width, CV_8UC3 );
                    vector<uchar> buff( itCamImageInfo->second.data.size() / 2 );
                    char a[ 3 ]; a[ 2 ] = '\0';
                    for( int j = 0; j < buff.size(); ++j ) {
                        a[ 0 ] = itCamImageInfo->second.data[ j * 2 ];
                        a[ 1 ] = itCamImageInfo->second.data[ j * 2 + 1 ];
                        buff[ j ] = strtol( a, NULL, 16 );
                    }
                    img_cam_display = imdecode(Mat(buff),CV_LOAD_IMAGE_COLOR); 

                    map<unsigned long long,GeometryMapInfo>::iterator itGeometryInfo = pTrackingResultResources->bufGeometry.find( pepmap.timeStamp );
                    if( itGeometryInfo != pTrackingResultResources->bufGeometry.end() ) {
                        vector<uchar> buff( itGeometryInfo->second.data.size() / 2 );
                        char a[ 3 ]; a[ 2 ] = '\0';
                        for( int j = 0; j < buff.size(); ++j ) {
                            a[ 0 ] = itGeometryInfo->second.data[ j * 2 ];
                            a[ 1 ] = itGeometryInfo->second.data[ j * 2 + 1 ];
                            buff[ j ] = strtol( a, NULL, 16 );
                        }
                        geometry.create(  itGeometryInfo->second.height, itGeometryInfo->second.width, CV_16U );
                        uLongf len_uncompressed = itGeometryInfo->second.height * itGeometryInfo->second.width * 2;
                        uncompress( geometry.data
                        , &len_uncompressed
                        , (const Bytef*)&buff[ 0 ]
                        , itGeometryInfo->second.data.size() / 2 );
                        
                        map<unsigned long long,GeometryMapInfo>::iterator itGeometry2Info = pTrackingResultResources->bufGeometry2.find( pepmap.timeStamp );
                        bool flgGeometry2Available = false;
                        if( itGeometry2Info != pTrackingResultResources->bufGeometry2.end() ) {
                            vector<uchar> buff2( itGeometry2Info->second.data.size() / 2 );
                            char a[ 3 ]; a[ 2 ] = '\0';
                            for( int j = 0; j < buff2.size(); ++j ) {
                                a[ 0 ] = itGeometry2Info->second.data[ j * 2 ];
                                a[ 1 ] = itGeometry2Info->second.data[ j * 2 + 1 ];
                                buff2[ j ] = strtol( a, NULL, 16 );
                            }
                            geometry2.create(  itGeometry2Info->second.height, itGeometry2Info->second.width, CV_16U );
                            uLongf len_uncompressed = itGeometry2Info->second.height * itGeometry2Info->second.width * 2;
                            uncompress( geometry2.data
                            , &len_uncompressed
                            , (const Bytef*)&buff2[ 0 ]
                            , itGeometry2Info->second.data.size() / 2 );

                            flgGeometry2Available = true;
                        }

                        map<int,Mat> img_silhouette;
                        for( int x = 0; x < geometry.cols; ++x ) {
                            for( int y = 0; y < geometry.rows; ++y ) {
                                const int keyval = geometry.at<unsigned short>( y, x );
                                map<int,int>::iterator itID;
                                if( ( itID = geometry_to_ID.find( keyval ) ) != geometry_to_ID.end() ) {
                                    line( img_cam_display, Point( x, y ), Point( x, y ), color_table[ itID->second % sizeColorTable ], 1 );
                                    if( flgGeometry2Available/* && itID->second == 1*/ ) {
                                        const int keyval2 = geometry2.at<unsigned short>( y, x );
                                        if( keyval2 > 0 ) {
                                            //const int col_on_silhouette = ( keyval2 - 1 ) % img_silhouette_display.cols;
                                            //const int row_on_silhouette = ( keyval2 - 1 ) / img_silhouette_display.cols;
                                            //line( img_silhouette_display
                                            //    , Point( col_on_silhouette, row_on_silhouette )
                                            //    , Point( col_on_silhouette, row_on_silhouette )
                                            //    , color_table[ itID->second % sizeColorTable ]
                                            //    , 1 );
                                            if( img_silhouette.find( itID->second ) == img_silhouette.end() ) {
                                                img_silhouette[ itID->second ].create( (int)( scale_m2px * 3.0 ), (int)( scale_m2px * roi_height ), CV_8UC3 );
                                                memset( img_silhouette[ itID->second ].data, 0, img_silhouette[ itID->second ].cols * img_silhouette[ itID->second ].rows * 3 );
                                            }
                                            const int col_on_silhouette = ( keyval2 - 1 ) % img_silhouette[ itID->second ].cols;
                                            const int row_on_silhouette = ( keyval2 - 1 ) / img_silhouette[ itID->second ].cols;
                                            line( img_silhouette[ itID->second ]
                                                , Point( col_on_silhouette, row_on_silhouette )
                                                , Point( col_on_silhouette, row_on_silhouette )
                                                , color_table[ itID->second % sizeColorTable ]
                                                , 1 );
                                        }
                                    }
                                }
                            }
                        }

                        for( map<int,Mat>::iterator itImgSilhouette = img_silhouette.begin(); itImgSilhouette != img_silhouette.end(); ++itImgSilhouette ) {
                            const int id = itImgSilhouette->first;
                            ostringstream oss;
                            oss << pTrackingResultResources->strSilhouettePath << id << "_" << pepmap.timeStamp << ".avi";
                            //imwrite( oss.str(), itImgSilhouette->second ); 
                            if( pTrackingResultResources->silhouetteVideoWriter.find( id ) == pTrackingResultResources->silhouetteVideoWriter.end() ) {
                                pTrackingResultResources->silhouetteVideoWriter[ id ].open( oss.str()
                                                                                          , CV_FOURCC('X','V','I','D')
                                                                                          , 10
                                                                                          , Size( itImgSilhouette->second.cols, itImgSilhouette->second.rows ) );
                            }
                            pTrackingResultResources->silhouetteVideoWriter[ id ].write( itImgSilhouette->second );
                        }

                        if( img_silhouette.find( 1 ) != img_silhouette.end() ) {
                            img_silhouette_display = img_silhouette[ 1 ];
                        }
                    }

                    imshow( "Camera", img_cam_display );
                    imshow( "Silhouette", img_silhouette_display );
                    pTrackingResultResources->cameraVideoWriter.write( img_cam_display );
                }

                //for( deque< map<int,Point2d> >::iterator itPosHuman = result_buffer.begin(); itPosHuman != result_buffer.end(); ++itPosHuman ) {
                //    for( map<int,Point2d>::iterator it = itPosHuman->begin(); it != itPosHuman->end(); ++it ) {
                //        int col = (int)( ( (float)img_display.size().width / (float)img_display_tmp.size().width ) * scale_m2px * ( ( it->second.x - roi_x ) + roi_width / 2.0f ) );
                //        int row = (int)( ( (float)img_display.size().height / (float)img_display_tmp.size().height ) * scale_m2px * ( ( it->second.y - roi_y ) + roi_height / 2.0f ) );
                //        const int sizeColorTable = sizeof( color_table ) / sizeof( CvScalar );
                //        //cout << "sizeColorTable=" << sizeColorTable << endl;
                //        circle( img_display, Point( row, col ), 3, color_table[ it->first % sizeColorTable ], -1 );
                //    }
                //}

                if( pTrackingResultResources->pViewer ) {
                    pTrackingResultResources->pViewer->SetPos( pepmap.timeStamp );
                }

                imshow( "Tracking Result", img_display );
                pTrackingResultResources->pepmapVideoWriter.write( img_display );
                if( pTrackingResultResources->GetDelayUpdate() ) {
		            (void)cvWaitKey( pTrackingResultResources->GetDelayUpdate() );
                } else {
		            (void)cvWaitKey( 1 );
                }
            } else {
#ifdef WINDOWS_OS
                EnterCriticalSection( &pTrackingResultResources->cs );
#endif
#ifdef LINUX_OS
                pthread_mutex_lock( &pTrackingResultResources->mutex );
#endif
                const unsigned long long time_pepmap = pTrackingResultResources->trackingResult.begin()->first;
                pTrackingResultResources->trackingResultExt.erase( pTrackingResultResources->trackingResultExt.begin()
                                                                 , pTrackingResultResources->trackingResultExt.lower_bound( time_pepmap )  );
                pTrackingResultResources->bufCameraImage.erase( pTrackingResultResources->bufCameraImage.begin()
                                                                 , pTrackingResultResources->bufCameraImage.lower_bound( time_pepmap )  );
                pTrackingResultResources->bufGeometry.erase( pTrackingResultResources->bufGeometry.begin()
                                                                 , pTrackingResultResources->bufGeometry.lower_bound( time_pepmap )  );
                pTrackingResultResources->bufGeometry2.erase( pTrackingResultResources->bufGeometry2.begin()
                                                                 , pTrackingResultResources->bufGeometry2.lower_bound( time_pepmap )  );
                pTrackingResultResources->trackingResult.erase( pTrackingResultResources->trackingResult.begin() );

		//cout << " -> erase()" << endl;
#ifdef WINDOWS_OS
                LeaveCriticalSection( &pTrackingResultResources->cs );
#endif
#ifdef LINUX_OS     
                pthread_mutex_unlock( &pTrackingResultResources->mutex );
#endif
            }
        }
        (void)cvWaitKey( 1 );
    }

#ifdef WINDOWS_OS
    return 1;
#endif
#ifdef LINUX_OS
    cout << endl << "Exiting Thread..." << endl;
    return NULL;
#endif
}
