#include <iostream>
#include <vector>
#include <deque>
#include <map>
#include <string>
#include <sstream>
#include <fstream>
//#include <tchar.h>

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "zlib.h"
//
//#include "GL/glui.h"

#include "../humantracking.h"

#include "CameraUnit.h"
#include "track.h"
#include "TrackingResultResources.h"
#include "TrackingProcessLogger.h"

//#ifdef WINDOWS_OS
//#include <conio.h>
//#endif

using namespace std;
using namespace cv;

bool flgPEPMapFile = false;
string strPEPMapFile;
bool flgOutputTrackingProcessData2Files = false;

float roi_width, roi_height;
float roi_x, roi_y;
float scale_m2px;

map<unsigned int,string> cameraName;
volatile bool flgRunGetPEPMapThread;

TrackingResultResources resTracking;
TrackingProcessLogger logTracking;

#ifdef WINDOWS_OS
CRITICAL_SECTION cs;
#endif
#ifdef LINUX_OS
pthread_mutex_t mutex;
#endif

deque<PEPMapInfo> bufPEPMap;

void getfilename( const string src, string* p_str_path, string* p_str_name, string* p_str_noextname )
{
    int idxExt = src.rfind( ".", src.npos );
#ifdef WINDOWS_OS
    int idxPath = src.rfind( "\\", src.npos );
#else
    int idxPath = src.rfind( "/", src.npos );
#endif
    *p_str_path = src.substr( 0, idxPath + 1 );
    *p_str_name = src.substr( idxPath + 1 );
    *p_str_noextname = src.substr( idxPath + 1, idxExt - idxPath - 1 );
}

bool load_pepmap_config()
{
    ifstream ifs;

    string strPath, strName, strNoextName;
    
    if( flgPEPMapFile ) {
        getfilename( strPEPMapFile, &strPath, &strName, &strNoextName );
    }

    ostringstream oss;
    oss << strPath << "pepmap.cfg";
    ifs.open( oss.str().c_str() );

    if( !ifs.is_open() ) {
        return false;
    }

    char buf[ 1000 ];
    vector<float> value;
    while( !ifs.eof() ) {
        ifs.getline( buf, sizeof( buf ) );
        string str( buf );
        if( str[ 0 ] == '#' ) {
            continue;
        }
        float v = atof( str.c_str() );
        value.push_back( v );
    }

    if( value.size() != 5 ) {
        return false;
    }

    roi_width = value[ 0 ];
    roi_height = value[ 1 ];
    roi_x = value[ 2 ];
    roi_y = value[ 3 ];
    scale_m2px = value[ 4 ];

    return true;
}

int load_cameraunit_list( vector<string>* p_addrCameraUnit )
{
    ifstream ifs;

    ifs.open( "cameraunit_list.txt" );

    if( !ifs.is_open() ) {
        return false;
    }

    p_addrCameraUnit->clear();
    string str;
    while( !ifs.eof() ) {
        ifs >> str;
	cout << "CameraUnit to be connected:" << str << endl;
        p_addrCameraUnit->push_back( str );
    }

    sort( p_addrCameraUnit->begin(), p_addrCameraUnit->end() );
    p_addrCameraUnit->erase( unique( p_addrCameraUnit->begin(), p_addrCameraUnit->end() )
			     , p_addrCameraUnit->end() );

    return true;
}

#ifdef WINDOWS_OS
DWORD WINAPI GetPEPMapThread( LPVOID p_cameraunit )
#endif
#ifdef LINUX_OS
void* GetPEPMapThread( void* p_cameraunit )
#endif
{
    CameraUnit* pCameraUnit = (CameraUnit*)p_cameraunit;

    pCameraUnit->send( "run\n", 4 );

    PEPMapInfo pepmap;
    char buf[ SIZE_BUFFER ];
	while( flgRunGetPEPMapThread ) {
		pCameraUnit->readline( buf, SIZE_BUFFER );

		string str( buf );
		if( str.find( "<PEPMap>" ) != str.npos ) {
			//cout << "Detected a PEP-map." << endl;
            {
                pCameraUnit->readline( buf, SIZE_BUFFER );
                string strtmp( buf );
                istringstream iss( strtmp );
                iss >> pepmap.serialNumber;
            }
            {
                pCameraUnit->readline( buf, SIZE_BUFFER );
                string strtmp( buf );
                istringstream iss( strtmp );
                iss >> pepmap.timeStamp;
            }            
            //cout << "Serial #: " << pepmap.serialNumber << ", time: " << pepmap.timeStamp;
            time_t _sec = pepmap.timeStamp / 1000000ULL;
            //cout << "Serial #: " << pepmap.serialNumber << ", time: " << pepmap.timeStamp << "(" << ctime( &_sec ) << ")";
            pCameraUnit->readline( buf, SIZE_BUFFER );
            int size = atoi( buf );
            //cout << " size = " << size << "...";
            pCameraUnit->read( buf, size * 2 );
            buf[ size * 2 ] = '\0';
            pepmap.data = string( buf );
            //cout << "Data Received." << endl;

#ifdef WINDOWS_OS
            EnterCriticalSection( &cs );
#endif
#ifdef LINUX_OS
	    pthread_mutex_lock( &mutex );
#endif
            bufPEPMap.push_back( pepmap );
            logTracking.receive_pepmap( pepmap );
            resTracking.UpdateView();
#ifdef WINDOWS_OS
            LeaveCriticalSection( &cs );
#endif
#ifdef LINUX_OS
	    pthread_mutex_unlock( &mutex );
#endif
		}
	}

    buf[ 0 ] = 'q'; buf[ 1 ] = '\n';
    pCameraUnit->send( buf, 2 );

    // skip all the unread PEP-maps.
    while( pCameraUnit->hasData() ) {
        pCameraUnit->ClearBuffer();
        cout << "buffer clear." << endl;
#ifdef WINDOWS_OS
      	Sleep( 100 );
#endif
#ifdef LINUX_OS
	usleep( 100000 );
#endif
    }

#ifdef WINDOWS_OS
    return 1;
#endif
#ifdef LINUX_OS
    //cout << "(from thread)exitting..." << endl;
    return NULL;
#endif
}

int bumblebee_mode()
{

	CameraUnit* cameraunit;
    string strSerial;

    resTracking.SetDelayUpdate( 40/*0*/ );

    //
    // Load camera unit list
    vector<string> addrCameraUnit;
    load_cameraunit_list( &addrCameraUnit );
    const int nCameraUnit = addrCameraUnit.size();
    cameraunit = new CameraUnit[ nCameraUnit ];


    //
    // Initialize the camera units
    for( size_t i = 0; i < nCameraUnit; ++i ) {
	    cameraunit[ i ].connect( addrCameraUnit[ i ] );

	    char buf[ SIZE_BUFFER ];
	    for( ; ; ) {
		    cameraunit[ i ].readline( buf, SIZE_BUFFER );
		    //cout << buf << endl;

		    string str( buf );
		    if( str.find( "<Initialized>" ) != str.npos ) {
                cout << str << endl;
                cameraunit[ i ].readline( buf, SIZE_BUFFER );
                strSerial = string( buf );
                cout << strSerial << endl;
            } else if( str.find( "<Camera Parameters Changed>" ) != str.npos ) {
                cout << str << endl;
		    } else if( str.find( "<Stereo Parameters Changed>" ) != str.npos ) {
                cout << str << endl;
                break;
		    }
	    }

	    cout << endl << "Camera " << strSerial << " is ready." << endl;
    }


    cout << "Request to send PEPMaps in two seconds." << endl;
#ifdef WINDOWS_OS
	Sleep( 2000 );
#endif
#ifdef LINUX_OS
	sleep( 2 );
#endif


    //
    // Craete thread for each cameraunit
    flgRunGetPEPMapThread = true;
#ifdef WINDOWS_OS
    InitializeCriticalSection( &cs );
    HANDLE* hThread = new HANDLE[ nCameraUnit ];
    DWORD* ThreadID = new DWORD[ nCameraUnit ];
    for( int i = 0; i < nCameraUnit; ++i ) {
        hThread[ i ] = CreateThread( NULL
                                    , 0
                                    , GetPEPMapThread
                                    , (LPVOID)&cameraunit[ i ]
                                    ,0
                                    ,&ThreadID[ i ] );
    }
#endif
#ifdef LINUX_OS
    pthread_mutex_init( &mutex, NULL );
    pthread_t* thread = new pthread_t[ nCameraUnit ];
    for( int i = 0; i < nCameraUnit; ++i ) {
      pthread_create( &thread[ i ]
		      , NULL
                      , GetPEPMapThread
                      , (void*)&cameraunit[ i ] );
    }
#endif


    //
    // Tracking
    Mat occupancy = Mat::zeros( (int)( roi_height * scale_m2px ), (int)( roi_width * scale_m2px ), CV_16U );
    Mat img_occupancy( (int)( scale_m2px * roi_height ), (int)( scale_m2px * roi_width ), CV_8U );
    Mat img_display2( (int)( roi_height * 80 ), (int)( roi_width * 80 ), CV_8U );
    resTracking.clear();
    resTracking.EnableViewWindow();
    //unsigned long long timeStamp;
    //unsigned int serialNumber;
    char buf[ SIZE_BUFFER ];
    //cameraunit.send( "run\n", 4 );
    int cnt = 0;
    PEPMapInfo pepmap;
    unsigned long long tmp_timestamp = 0;
    map<unsigned long long,PEPMapInfo> sort_buffer;
#ifdef WINDOWS_OS    
    ofstream ofs_pepmap( "received_pepmaps.log" );
#endif
	do {
#ifdef WINDOWS_OS
        EnterCriticalSection( &cs );
#endif
#ifdef LINUX_OS
	pthread_mutex_lock( &mutex );
#endif
        if( !bufPEPMap.empty() ) {
            //if( bufPEPMap.empty() ) {
            //    int a = 0;
            //}
            pepmap = bufPEPMap.front();
            bufPEPMap.pop_front();
        } else {
#ifdef WINDOWS_OS
            LeaveCriticalSection( &cs );
#endif
#ifdef LINUX_OS
            pthread_mutex_unlock( &mutex );
#endif
            continue;
        }
#ifdef WINDOWS_OS
        LeaveCriticalSection( &cs );
#endif
#ifdef LINUX_OS
        pthread_mutex_unlock( &mutex );
#endif

        sort_buffer[ pepmap.timeStamp ] = pepmap;

        for( ; ; ) {
            if( sort_buffer.size() < 2 ) {
                break;
            }

            unsigned long long diff = sort_buffer.rbegin()->first - sort_buffer.begin()->first;
            if( diff < 300000/*1000000*/ ) {
                break;
            }
            
            pepmap = sort_buffer.begin()->second;
            sort_buffer.erase( sort_buffer.begin() );

            const int size = pepmap.data.size() / 2;

            char a[ 3 ]; a[ 2 ] = '\0';
            for( int j = 0; j < size; ++j ) {
                a[ 0 ] = pepmap.data[ j * 2 ];
                a[ 1 ] = pepmap.data[ j * 2 + 1 ];
                buf[ j ] = strtol( a, NULL, 16 );
            }
            //cout << "Data Received." << endl;
            time_t _sec = pepmap.timeStamp / 1000000ULL;
            string strTime( ctime( &_sec ) );
            if( !strTime.empty() ) {
		strTime.erase( strTime.size() - 1 );
	    }
            //cout << "Serial #: " << pepmap.serialNumber << ", time: " << pepmap.timeStamp << "(" << strTime << ")" << endl;
#ifdef WINDOWS_OS
            ofs_pepmap << "Serial #: " << pepmap.serialNumber << ", time: " << pepmap.timeStamp << "(" << strTime << ")" << endl;
#endif
            ++cnt;
            if( pepmap.timeStamp < tmp_timestamp ) {
	        cout << "## Illegal Time Stamp! ## This pepmap from " << pepmap.serialNumber << " will be rejected. To avoid this, have more size of sort_buffer." << endl;
                continue;
            }
            tmp_timestamp = pepmap.timeStamp;

            uLongf len_uncompressed = (int)( roi_height * scale_m2px ) * (int)( roi_width * scale_m2px ) * 2;
            uncompress( occupancy.data
            , &len_uncompressed
            , (const Bytef*)buf
            , size );

            //occupancy.convertTo( img_occupancy, CV_8U );
            //resize( img_occupancy, img_display2, img_display2.size() );
            //imshow( "Occupancy Map", img_display2 );
            //(void)cvWaitKey( 10 );

            // Tracking
            //map<int,CTrajectory> result;
            map< unsigned long long, map<int,Point2d> > result;
            if( track( &result, occupancy, pepmap.timeStamp ) ) {
                // Store result view resources
                resTracking.AddResultTrajectories( result );
            }
	    //cout << "Adding PEP-map info...";
            resTracking.AddPEPMapInfo( pepmap );
	    //cout << "Done." << endl;
        }
	} while( cnt < 2000 );

    resTracking.TerminateViewWindow();

    //
    // Exit all the threads
    flgRunGetPEPMapThread = false;
#ifdef WINDOWS_OS
    WaitForMultipleObjects( nCameraUnit, hThread, true, INFINITE );
    
    delete [] hThread;
    delete [] ThreadID;
    
    hThread = NULL;
    ThreadID = NULL;
#endif
#ifdef LINUX_OS
    void* thread_result;
    //cout << "Waiting for all threads to quit...(nCameraUnit=" << nCameraUnit << ")";
    for( int i = 0; i < nCameraUnit; ++i ) {
      pthread_join( thread[ i ], NULL  );
    }    
    //cout << "done." << endl;

    delete [] thread;
    thread = NULL;
#endif
    ////buf[ 0 ] = 27; buf[ 1 ] = '\n';
    //buf[ 0 ] = 'q'; buf[ 1 ] = '\n';
    //cameraunit.send( buf, 2 );

    //// skip all the unread PEP-maps.
    //while( cameraunit.hasData() ) {
    //    cameraunit.ClearBuffer();
    //    cout << "buffer clear." << endl;
    //  	Sleep( 100 );
    //}

	//cameraunit.send( "hoge\n", 5 );
	//for( ; ; ) {
	//	cameraunit.readline( buf, 1000 );
	//	cout << buf << endl;

	//	string str( buf );
	//	if( str.find( "Line99" ) != str.npos ) {
	//		break;
	//	}
	//}

	//Sleep( 2000 );

    // 
    // Disconnect all the camera unit.
    //cout << "Disconnecting camera units...";
    for( size_t i = 0; i < nCameraUnit; ++i ) {
	    cameraunit[ i ].disconnect();
    }
    //cout << endl;

    delete [] cameraunit;
    cameraunit = NULL;

#ifdef WINDOWS_OS
    DeleteCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_destroy( &mutex );
#endif

    cout << "Succeeded." << endl;

	return 0;
/*
    // Load camera unit list
    // test code
    vector<string> addrCameraUnit;
    addrCameraUnit.push_back( "192.168.1.22" );

    
    // Connect to camera units.
    HANDLE hRead, hWrite;
    cout << "Try to connect to " << addrCameraUnit[ 0 ] << "...";
    {
        ostringstream oss;
        oss << "C:\\Windows\\SUA\\bin\\rsh " << addrCameraUnit[ 0 ] << "-l kumalab /home/kumalab/project/HumanTracking/bin/cameraunit --nowindow";

        //HANDLE _hRead, _hWrite;
        SECURITY_ATTRIBUTES sa;
        PROCESS_INFORMATION pi;
        STARTUPINFO si;

        ZeroMemory( &sa, sizeof(sa) );
        sa.lpSecurityDescriptor = NULL;
        sa.bInheritHandle = TRUE;

        bool ret;

        ret = CreatePipe( &hRead, &hWrite, &sa, 0 );
        //DuplicateHandle( GetCurrentProcess()
        //               , _hRead
        //               , GetCurrentProcess()
        //               , 0
        //               , FALSE
        //               , DUPLICATE_SAME_ACCESS );


        ZeroMemory( &si, sizeof(si) );
        si.cb = sizeof(si);
        si.hStdInput = hRead;
        si.hStdOutput = hWrite;
        si.hStdError = GetStdHandle( STD_ERROR_HANDLE );
        ret = CreateProcess( NULL
                           , _T( "C:\\Windows\\SUA\\bin\\rsh 192.168.1.22 -l kumalab /home/kumalab/project/HumanTracking/bin/cameraunit --nowindow" )
                           , NULL
                           , NULL
                           , TRUE
                           , 0//NORMAL_PRIORITY_CLASS
                           , NULL
                           , NULL
                           , &si
                           , &pi);

        if( ret == false ) {
            DWORD id =  GetLastError();
            OutputMessage( id );
            cout << endl << " CreateProcess() failed." << endl;
            CloseHandle( hWrite );
            CloseHandle( hRead );
            exit( 1 );
        }
    }
    cout << "Succeeded." << endl;

    CloseHandle( hWrite );
    CloseHandle( hRead );
*/
}

int pepmapfile_mode( string strVideoFile )
{
    ifstream ifs( strVideoFile.c_str() );

    if( !ifs.is_open() ) {
        cout << "Error occured in opening the PEP-map file.";
        return false;
    }

    resTracking.SetDelayUpdate( 25 );

	char buf[ SIZE_BUFFER ];
    string str;
    Mat occupancy = Mat::zeros( (int)( roi_height * scale_m2px ), (int)( roi_width * scale_m2px ), CV_16U );
    Mat img_occupancy( (int)( scale_m2px * roi_height ), (int)( scale_m2px * roi_width ), CV_8U );
    Mat img_display2( (int)( roi_height * 80 ), (int)( roi_width * 80 ), CV_8U );
    resTracking.clear();
    resTracking.EnableViewWindow();
    PEPMapInfo pepmap;
    //unsigned long long timeStamp;
    //unsigned int serialNumber;
    int size;
    while( !ifs.eof() ) {
        //ifs.getline( buf, SIZE_BUFFER );
        ifs >> str;

		//string str( buf );
		if( str.find( "<PEPMap>" ) != str.npos ) {
			cout << "Detected a PEP-map." << endl;
            //ifs.getline( buf, SIZE_BUFFER );
            ifs >> pepmap.serialNumber >> pepmap.timeStamp >> size;
            //timeStamp /= 10;
            //int size = atoi( buf );
            time_t _sec = pepmap.timeStamp / 1000000ULL;
            cout << "Serial #: " << pepmap.serialNumber << ", time: " << pepmap.timeStamp << ", " << ctime( &_sec );
            //cout << ", size = " << size << "...";
            //ifs.read( buf, size * 2 );
            //str = string( buf );
            ifs >> pepmap.data;
            char a[ 3 ]; a[ 2 ] = '\0';
            for( int j = 0; j < size; ++j ) {
                a[ 0 ] = pepmap.data[ j * 2 ];
                a[ 1 ] = pepmap.data[ j * 2 + 1 ];
                buf[ j ] = strtol( a, NULL, 16 );
            }
            cout << "Data Received." << endl;

            uLongf len_uncompressed = (int)( roi_height * scale_m2px ) * (int)( roi_width * scale_m2px ) * 2;
            uncompress( occupancy.data
            , &len_uncompressed
            , (const Bytef*)buf
            , size );

            for( int row = 0; row < occupancy.rows; ++row ) {
                for( int col = 0; col < occupancy.cols; ++col ) {
                    if( occupancy.at<unsigned short>( row, col ) < 50 ) {
                        occupancy.at<unsigned short>( row, col ) = 0;
                    }
                }
            }
            //occupancy.convertTo( img_occupancy, CV_8U );
            //resize( img_occupancy, img_display2, img_display2.size() );
            //imshow( "Occupancy Map", img_display2 );
            //(void)cvWaitKey( 1 );
            
            // Tracking
            map< unsigned long long, map<int,Point2d> > result;
            if( track( &result, occupancy, pepmap.timeStamp ) ) {
                // Store result view resources
                resTracking.AddResultTrajectories( result );
            }
            resTracking.AddPEPMapInfo( pepmap );
        }

    }
    return 0;
}


int main( int argc, char *argv[] )
{
    string strPath, strName, strNoextName;
    string strTrackingConfigFile = "tracking.cfg";

    for( int i = 0; i < argc; ++i ) {
        string strOpt = argv[ i ];
        if( strOpt == "-f" ) {
            flgPEPMapFile = true;
            strPEPMapFile = string( argv[ ++i ] );
            getfilename( strPEPMapFile, &strPath, &strName, &strNoextName );
        } else if( strOpt == "--output-trackingprocess-files" ) {
            flgOutputTrackingProcessData2Files = true;
        } else if( strOpt == "-c" ) {
	    strTrackingConfigFile = string( argv[ ++i ] );
	}
    }

    initialize_tracker();

    load_pepmap_config();
    load_track_parameters( strPath, strTrackingConfigFile );

    resTracking.init( strPath + "result.txt" );

    if( !flgPEPMapFile ) {
        return bumblebee_mode();
    } else {
        return pepmapfile_mode( strPEPMapFile );
    }
}
