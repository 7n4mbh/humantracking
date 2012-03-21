#include <iostream>
#include <string>
#include <sstream>
#include <tchar.h>

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "GL/glui.h"

#include "CameraUnit.h"

#if defined(WINDOWS) || defined(_WIN32)
#define WINDOWS_OS
#elif defined(linux) || defined(__linux__)
#define LINUX_OS
#endif

#ifdef WINDOWS_OS
#include <conio.h>
#endif

using namespace std;
using namespace cv;

void DisplayError(char *pszAPI);
void ReadAndHandleOutput(HANDLE hPipeRead);
void PrepAndLaunchRedirectedChild(HANDLE hChildStdOut,
                                    HANDLE hChildStdIn,
                                    HANDLE hChildStdErr);
DWORD WINAPI GetAndSendInputThread(LPVOID lpvThreadParam);

HANDLE hChildProcess = NULL;
HANDLE hStdIn = NULL; // Handle to parents std input.
BOOL bRunThread = TRUE;

static inline void OutputMessage( DWORD dwMessageId )  
{  
    LPTSTR lpBuffer = NULL;  
    FormatMessage( FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM
                 , NULL
                 , dwMessageId
                 , LANG_USER_DEFAULT
                 , (LPTSTR)&lpBuffer
                 , 0
                 , NULL );  
    cout << " " << lpBuffer << endl;
    LocalFree( lpBuffer );  
}  

int main( int argc, char *argv[] )
{

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
	CameraUnit cameraunit;
    string strSerial;

	cameraunit.connect();

	char buf[ 1000 ];
	for( ; ; ) {
		cameraunit.readline( buf, 1000 );
		//cout << buf << endl;

		string str( buf );
		if( str.find( "<Camera Initialized>" ) != str.npos ) {
            cout << str << endl;
            cameraunit.readline( buf, 1000 );
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

    cameraunit.send( "run\n", 4 );
    int cnt = 0;
	do {
		cameraunit.readline( buf, 1000 );

		string str( buf );
		if( str.find( "<PEPMap>" ) != str.npos ) {
			cout << "Detect a PEP-map.";
            cameraunit.readline( buf, 1000 );
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
		}
	} while( cnt < 30 );
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
}