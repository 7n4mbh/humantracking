#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
//#include <tchar.h>

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "zlib.h"
//
//#include "GL/glui.h"

#include "CameraUnit.h"
#include "track.h"

#if defined(WINDOWS) || defined(_WIN32)
#define WINDOWS_OS
#elif defined(linux) || defined(__linux__)
#define LINUX_OS
#endif

//#ifdef WINDOWS_OS
//#include <conio.h>
//#endif

using namespace std;
using namespace cv;

#define SIZE_BUFFER ( 10000 )

bool flgPEPMapFile = false;
string strPEPMapFile;

float roi_width, roi_height;
float roi_x, roi_y;
float scale_m2px;

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

int bumblebee_mode()
{

	CameraUnit cameraunit;
    string strSerial;

	cameraunit.connect( "192.168.1.100" );

	char buf[ SIZE_BUFFER ];
	for( ; ; ) {
		cameraunit.readline( buf, SIZE_BUFFER );
		//cout << buf << endl;

		string str( buf );
		if( str.find( "<Initialized>" ) != str.npos ) {
            cout << str << endl;
            cameraunit.readline( buf, SIZE_BUFFER );
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
    cout << "Request to send PEPMaps in two seconds." << endl;

	Sleep( 2000 );

    Mat occupancy = Mat::zeros( (int)( roi_height * scale_m2px ), (int)( roi_width * scale_m2px ), CV_16U );
    Mat img_occupancy( (int)( scale_m2px * roi_height ), (int)( scale_m2px * roi_width ), CV_8U );
    Mat img_display2( (int)( roi_height * 80 ), (int)( roi_width * 80 ), CV_8U );
    unsigned long long timeStamp;
    unsigned int serialNumber;
    cameraunit.send( "run\n", 4 );
    int cnt = 0;
	do {
		cameraunit.readline( buf, SIZE_BUFFER );

		string str( buf );
		if( str.find( "<PEPMap>" ) != str.npos ) {
			cout << "Detected a PEP-map." << endl;
            {
                cameraunit.readline( buf, SIZE_BUFFER );
                string strtmp( buf );
                istringstream iss( strtmp );
                iss >> serialNumber;
            }
            {
                cameraunit.readline( buf, SIZE_BUFFER );
                string strtmp( buf );
                istringstream iss( strtmp );
                iss >> timeStamp;
            }            
            //cout << "Serial #: " << serialNumber << ", time: " << timeStamp;
            time_t _sec = timeStamp / 1000000ULL;
            cout << "Serial #: " << serialNumber << ", time: " << timeStamp << "(" << ctime( &_sec ) << ")";
            cameraunit.readline( buf, SIZE_BUFFER );
            int size = atoi( buf );
            cout << " size = " << size << "...";
            cameraunit.read( buf, size * 2 );
            str = string( buf );
            char a[ 3 ]; a[ 2 ] = '\0';
            for( int j = 0; j < size; ++j ) {
                a[ 0 ] = str[ j * 2 ];
                a[ 1 ] = str[ j * 2 + 1 ];
                buf[ j ] = strtol( a, NULL, 16 );
            }
            cout << "Data Received." << endl;
            ++cnt;

            uLongf len_uncompressed = (int)( roi_height * scale_m2px ) * (int)( roi_width * scale_m2px ) * 2;
            uncompress( occupancy.data
            , &len_uncompressed
            , (const Bytef*)buf
            , size );

            occupancy.convertTo( img_occupancy, CV_8U );
            resize( img_occupancy, img_display2, img_display2.size() );
            imshow( "Occupancy Map", img_display2 );
            (void)cvWaitKey( 1 );

            // Tracking
            track( occupancy, timeStamp );
		}
	} while( cnt < 100 );
    //buf[ 0 ] = 27; buf[ 1 ] = '\n';
    buf[ 0 ] = 'q'; buf[ 1 ] = '\n';
    cameraunit.send( buf, 2 );

    // skip all the unread PEP-maps.
    while( cameraunit.hasData() ) {
        cameraunit.ClearBuffer();
        cout << "buffer clear." << endl;
      	Sleep( 100 );
    }

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

	cameraunit.disconnect();

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
    ifstream ifs( strVideoFile );

    if( !ifs.is_open() ) {
        cout << "Error occured in opening the PEP-map file.";
        return false;
    }

	char buf[ SIZE_BUFFER ];
    string str;
    Mat occupancy = Mat::zeros( (int)( roi_height * scale_m2px ), (int)( roi_width * scale_m2px ), CV_16U );
    Mat img_occupancy( (int)( scale_m2px * roi_height ), (int)( scale_m2px * roi_width ), CV_8U );
    Mat img_display2( (int)( roi_height * 80 ), (int)( roi_width * 80 ), CV_8U );
    unsigned long long timeStamp;
    unsigned int serialNumber;
    int size;
    while( !ifs.eof() ) {
        //ifs.getline( buf, SIZE_BUFFER );
        ifs >> str;

		//string str( buf );
		if( str.find( "<PEPMap>" ) != str.npos ) {
			cout << "Detected a PEP-map." << endl;
            //ifs.getline( buf, SIZE_BUFFER );
            ifs >> serialNumber >> timeStamp >> size;
            //timeStamp /= 10;
            //int size = atoi( buf );
            time_t _sec = timeStamp / 1000000ULL;
            cout << "Serial #: " << serialNumber << ", time: " << timeStamp << ", " << ctime( &_sec );
            //cout << ", size = " << size << "...";
            //ifs.read( buf, size * 2 );
            //str = string( buf );
            ifs >> str;
            char a[ 3 ]; a[ 2 ] = '\0';
            for( int j = 0; j < size; ++j ) {
                a[ 0 ] = str[ j * 2 ];
                a[ 1 ] = str[ j * 2 + 1 ];
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
            occupancy.convertTo( img_occupancy, CV_8U );
            resize( img_occupancy, img_display2, img_display2.size() );
            imshow( "Occupancy Map", img_display2 );
            (void)cvWaitKey( 1 );
            
            // Tracking
            track( occupancy, timeStamp );
        }

    }
    return 0;
}


int main( int argc, char *argv[] )
{
    string strPath, strName, strNoextName;

    for( int i = 0; i < argc; ++i ) {
        string strOpt = argv[ i ];
        if( strOpt == "-f" ) {
            flgPEPMapFile = true;
            strPEPMapFile = string( argv[ ++i ] );
            getfilename( strPEPMapFile, &strPath, &strName, &strNoextName );
        }
    }

    initialize_tracker();

    load_pepmap_config();
    load_track_parameters( strPath );

    if( !flgPEPMapFile ) {
        return bumblebee_mode();
    } else {
        return pepmapfile_mode( strPEPMapFile );
    }
}