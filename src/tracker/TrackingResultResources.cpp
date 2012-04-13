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

void TrackingResultResources::init( std::string filename )
{
    bufPEPMap.clear();
    trackingResult.clear();
    nUpdateViewRequest = 0;
    delayUpdate = 0;
    posHumanStill.clear();
    cntStill.clear();
    strResultFilename = filename;
    ofstream ofs( strResultFilename.c_str() );
}

void TrackingResultResources::clear()
{
    bufPEPMap.clear();
    trackingResult.clear();
    nUpdateViewRequest = 0;
    posHumanStill.clear();
    cntStill.clear();
    ofstream ofs( strResultFilename.c_str() );
}

void TrackingResultResources::AddResultTrajectories( const std::map< unsigned long long, std::map<int,cv::Point2d> >& result )
{
#ifdef WINDOWS_OS
    EnterCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_lock( &mutex );
#endif
    cout << endl << "Received New Results!" << endl;

    ofstream ofs(  strResultFilename.c_str(), ios::out | ios::app );

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
    Mat img_occupancy( (int)( scale_m2px * roi_height ), (int)( scale_m2px * roi_width ), CV_8U );
    Mat img_display_tmp( (int)( roi_height * 80 ), (int)( roi_width * 80 ), CV_8UC3 );
    Mat img_display( (int)( roi_height * 80 ), (int)( roi_width * 80 ), CV_8UC3 );
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

                map<int, vector<Point2d> > trajectory_to_draw;
                for( deque< map<int,Point2d> >::iterator itPosHuman = result_buffer.begin(); itPosHuman != result_buffer.end(); ++itPosHuman ) {
                    for( map<int,Point2d>::iterator it = itPosHuman->begin(); it != itPosHuman->end(); ++it ) {
                        trajectory_to_draw[ it->first ].push_back( it->second );
                    }
                }

                int old_col, old_row;
                for( map< int, vector<Point2d> >::iterator itHuman = trajectory_to_draw.begin(); itHuman != trajectory_to_draw.end(); ++itHuman ) {
                    for( vector<Point2d>::iterator it = itHuman->second.begin(); it != itHuman->second.end(); ++it ) {
                        int col = (int)( ( (float)img_display.size().width / (float)img_display_tmp.size().width ) * scale_m2px * ( ( it->x - roi_x ) + roi_width / 2.0f ) );
                        int row = (int)( ( (float)img_display.size().height / (float)img_display_tmp.size().height ) * scale_m2px * ( ( it->y - roi_y ) + roi_height / 2.0f ) );
                        circle( img_display, Point( row, col ), 3, color_table[ itHuman->first % sizeColorTable ], -1 );
                        if( it != itHuman->second.begin() ) {
                            line( img_display, Point( row, col ), Point( old_row, old_col ), color_table[ itHuman->first % sizeColorTable ], 6 );
                        }
                        old_col = col;
                        old_row = row;
                    }

		    if( pTrackingResultResources->cntStill.find( itHuman->first ) != pTrackingResultResources->cntStill.end() ) {
			if( pTrackingResultResources->cntStill[ itHuman->first ] > 30 ) {
			    int col = (int)( ( (float)img_display.size().width / (float)img_display_tmp.size().width ) * scale_m2px * ( ( pTrackingResultResources->posHumanStill[ itHuman->first ].x - roi_x ) + roi_width / 2.0f ) );
			    int row = (int)( ( (float)img_display.size().height / (float)img_display_tmp.size().height ) * scale_m2px * ( ( pTrackingResultResources->posHumanStill[ itHuman->first ].y - roi_y ) + roi_height / 2.0f ) );
			    circle( img_display, Point( row, col ), ( (float)img_display.size().width / (float)img_display_tmp.size().width ) * 0.5 * scale_m2px, color_table[ itHuman->first % sizeColorTable ], 2 );
			}
		    }
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

                imshow( "Tracking Result", img_display );
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
