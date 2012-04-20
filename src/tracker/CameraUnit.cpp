#include "CameraUnit.h"

#include <iostream>
#include <sstream>
#ifdef WINDOWS_OS
#include <tchar.h>
#endif
#ifdef LINUX_OS
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#endif

using namespace std;

//#define SIZE_BUFFER ( 1000 )
#define R (0)
#define W (1)

CameraUnit::CameraUnit()
{
#ifdef WINDOWS_OS
   hChildProcess = NULL;
   //hStdIn = NULL; // Handle to parents std input.
   bRunThread = TRUE;
    hThread = NULL;
#endif
#ifdef LINUX_OS
    bRunThread = true;
#endif
}

void CameraUnit::connect( const std::string& addr )
{
#ifdef WINDOWS_OS
      HANDLE hOutputReadTmp,hOutputWrite;
      HANDLE hInputWriteTmp,hInputRead;
      HANDLE hErrorWrite;

      SECURITY_ATTRIBUTES sa;

      strAddr = addr;

      InitializeCriticalSection( &cs );

      // Set up the security attributes struct.
      sa.nLength= sizeof(SECURITY_ATTRIBUTES);
      sa.lpSecurityDescriptor = NULL;
      sa.bInheritHandle = TRUE;


      // Create the child output pipe.
      if (!CreatePipe(&hOutputReadTmp,&hOutputWrite,&sa,0))
         DisplayError("CreatePipe");


      // Create a duplicate of the output write handle for the std error
      // write handle. This is necessary in case the child application
      // closes one of its std output handles.
      if (!DuplicateHandle(GetCurrentProcess(),hOutputWrite,
                           GetCurrentProcess(),&hErrorWrite,0,
                           TRUE,DUPLICATE_SAME_ACCESS))
         DisplayError("DuplicateHandle");


      // Create the child input pipe.
      if (!CreatePipe(&hInputRead,&hInputWriteTmp,&sa,0))
         DisplayError("CreatePipe");


      // Create new output read handle and the input write handles. Set
      // the Properties to FALSE. Otherwise, the child inherits the
      // properties and, as a result, non-closeable handles to the pipes
      // are created.
      if (!DuplicateHandle(GetCurrentProcess(),hOutputReadTmp,
                           GetCurrentProcess(),
                           &hOutputRead, // Address of new handle.
                           0,FALSE, // Make it uninheritable.
                           DUPLICATE_SAME_ACCESS))
         DisplayError("DupliateHandle");

      if (!DuplicateHandle(GetCurrentProcess(),hInputWriteTmp,
                           GetCurrentProcess(),
                           &hInputWrite, // Address of new handle.
                           0,FALSE, // Make it uninheritable.
                           DUPLICATE_SAME_ACCESS))
      DisplayError("DupliateHandle");


      // Close inheritable copies of the handles you do not want to be
      // inherited.
      if (!CloseHandle(hOutputReadTmp)) DisplayError("CloseHandle");
      if (!CloseHandle(hInputWriteTmp)) DisplayError("CloseHandle");


      // Get std input handle so you can close it and force the ReadFile to
      // fail when you want the input thread to exit.
      //if ( (hStdIn = GetStdHandle(STD_INPUT_HANDLE)) ==
      //                                          INVALID_HANDLE_VALUE )
      //   DisplayError("GetStdHandle");

      PrepAndLaunchRedirectedChild(hOutputWrite,hInputRead,hErrorWrite);


      // Close pipe handles (do not continue to modify the parent).
      // You need to make sure that no handles to the write end of the
      // output pipe are maintained in this process or else the pipe will
      // not close when the child process exits and the ReadFile will hang.
      if (!CloseHandle(hOutputWrite)) DisplayError("CloseHandle");
      if (!CloseHandle(hInputRead )) DisplayError("CloseHandle");
      if (!CloseHandle(hErrorWrite)) DisplayError("CloseHandle");


      // Launch the thread that gets the input and sends it to the child.
      //hThread = CreateThread(NULL,0,GetAndSendInputThread,
      //                        (LPVOID)hInputWrite,0,&ThreadId);
      //if (hThread == NULL) DisplayError("CreateThread");
	  hThread = CreateThread(NULL,0,CameraUnit::ReadFromCameraUnitThread,
                              (LPVOID)this,0,&ThreadId);
      if (hThread == NULL) DisplayError("CreateThread");

      // Read the child's output.
      //ReadAndHandleOutput(hOutputRead);
      // Redirection is complete
#endif
#ifdef LINUX_OS
      strAddr = addr;

      pthread_mutex_init( &mutex, NULL );

      int pid;

      // Create two pipes.
      if( pipe( pipe_c2p ) < 0 ) {
	perror( "popen2" );
	return;// -1;
      }

      if( pipe( pipe_p2c ) < 0 ) {
	perror( "popen2" );
	close( pipe_c2p[R] );
	close( pipe_c2p[W] );
	return;// -1;
      }

      // Launch the child process
      if( ( pid = fork() ) < 0 ) {
          perror( "fork" );
	  close( pipe_c2p[R] );
	  close( pipe_c2p[W] );
	  close( pipe_p2c[R] );
	  close( pipe_p2c[W] );
      }

      if( pid == 0 ) {
	  // When child process
	//cout << "Child proness successfully launched." << endl;
	close( pipe_p2c[W] );
	close( pipe_c2p[R] );
	dup2( pipe_p2c[R], 0 );
	dup2( pipe_c2p[W], 1 );
	close( pipe_p2c[R] );
	close( pipe_c2p[W] );
	int ret;
        if( strAddr == "192.168.1.100" ) {
            ret = execlp( "rsh", "rsh", strAddr.c_str(), "-l", "kumalab", "/home/kumalab/project/HumanTracking/bin/cameraunit", "--nowindow", NULL );
	} else {
            ret = execlp( "rsh", "rsh", strAddr.c_str(), "-l", "kumalab", "/home/kumalab/project/HumanTracking/bin/cameraunit", "--nowindow", "--no-stdout-camimage", "--no-stdout-geometry", NULL ); // debug code
	}
	if( ret < 0 ) {
	//if( execlp( "./child", "child", NULL ) < 0 ) {
  	    perror( "execlp" );
	    close( pipe_p2c[R] );
	    close( pipe_c2p[W] );
	    exit( 1 );
	}
      }

      // When parent process
      close( pipe_p2c[R] );
      close( pipe_c2p[W] );

      // Launch the thread that gets the input and sends it to the child.
      pthread_create( &thread
                    , NULL
                    , CameraUnit::ReadFromCameraUnitThread
                    , (void*)this );

#endif
}

void CameraUnit::disconnect()
{
#ifdef WINDOWS_OS
      // Force the read on the input to return by closing the stdin handle.
      //if (!CloseHandle(hStdIn)) DisplayError("CloseHandle");


	  const char* strCmd = "quit\n";
	  send( strCmd, 5 ); // Send exit command to the camera unit process
	  send( strCmd, 5 ); // Send exit command to the camera unit process
	  WaitForSingleObject( hChildProcess, INFINITE ); // Wait until the camera unit process ends

      // Tell the thread to exit and wait for thread to die.
      bRunThread = FALSE;

      if (WaitForSingleObject(hThread,INFINITE) == WAIT_FAILED)
         DisplayError("WaitForSingleObject");

      if (!CloseHandle(hOutputRead)) DisplayError("CloseHandle");
      if (!CloseHandle(hInputWrite)) DisplayError("CloseHandle");

      DeleteCriticalSection( &cs );
#endif
#ifdef LINUX_OS
      const char* strCmd = "quit\n";
      send( strCmd, 5 ); // Send exit command to the camera unit process
      send( strCmd, 5 ); // Send exit command to the camera unit process
      //WaitForSingleObject( hChildProcess, INFINITE ); // Wait until the camera unit process ends

      bRunThread = false;
      void* thread_result;
      pthread_join( thread, &thread_result );

      close( pipe_p2c[W] );
      close( pipe_c2p[R] );

      pthread_mutex_destroy( &mutex );
#endif
}

void CameraUnit::read( char* buf, size_t size )
{
	int i = 0;
	for( ; ; ) {
#ifdef WINDOWS_OS
		EnterCriticalSection( &cs );
#endif
#ifdef LINUX_OS
		pthread_mutex_lock( &mutex );
#endif
		if( !buffer.empty() ) {
			buf[ i ] = buffer[ 0 ];
			buffer.pop_front();

			++i;
			if( i >= size ) {
#ifdef WINDOWS_OS
		            LeaveCriticalSection( &cs );
#endif
#ifdef LINUX_OS
                            pthread_mutex_unlock( &mutex );
#endif
				break;
			}
        }
#ifdef WINDOWS_OS
            LeaveCriticalSection( &cs );
#endif
#ifdef LINUX_OS
            pthread_mutex_unlock( &mutex );
#endif
	}
}

void CameraUnit::readline( char* buf, size_t size )
{
	int i = 0;
	for( ; ; ) {
#ifdef WINDOWS_OS
		EnterCriticalSection( &cs );
#endif
#ifdef LINUX_OS
		pthread_mutex_lock( &mutex );
#endif
		if( !buffer.empty() ) {
            if( buffer[ 0 ] != '\r' ) {
			    buf[ i ] = buffer[ 0 ];
			    buffer.pop_front();
			    if( buf[ i ] == '\n' ) {
				    buf[ i ] = '\0';
#ifdef WINDOWS_OS
                                    LeaveCriticalSection( &cs );
#endif
#ifdef LINUX_OS
                                    pthread_mutex_unlock( &mutex );
#endif
				    break;
			    }

			    ++i;
			    if( i >= size ) {
#ifdef WINDOWS_OS
                                LeaveCriticalSection( &cs );
#endif
#ifdef LINUX_OS
                                pthread_mutex_unlock( &mutex );
#endif
				    break;
			    }
            } else {
                buffer.pop_front();
            }
		}
#ifdef WINDOWS_OS
            LeaveCriticalSection( &cs );
#endif
#ifdef LINUX_OS
            pthread_mutex_unlock( &mutex );
#endif
	}
}

void CameraUnit::send( const char* buf, size_t size )
{
#ifdef WINDOWS_OS
	DWORD nBytesWrote;
	if (!WriteFile(hInputWrite,buf,size,&nBytesWrote,NULL))
    {
    if (GetLastError() == ERROR_NO_DATA)
        return; // Pipe was closed.
    else
    DisplayError("WriteFile");
    }
#endif
#ifdef LINUX_OS
    size_t nBytesWrote;

    nBytesWrote = write( pipe_p2c[W], (void*)buf, size );
    if( nBytesWrote != size ) {
      cerr << "fwrite() failed." << endl;
    }
#endif
}

bool CameraUnit::hasData()
{ 
    bool ret;
#ifdef WINDOWS_OS
    EnterCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_lock( &mutex );
#endif
    ret = !buffer.empty();
#ifdef WINDOWS_OS
    LeaveCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_unlock( &mutex );
#endif
    return ret; 
}

void CameraUnit::ClearBuffer()
{ 
    bool ret;
#ifdef WINDOWS_OS
    EnterCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_lock( &mutex );
#endif
    buffer.clear();
#ifdef WINDOWS_OS
    LeaveCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    pthread_mutex_unlock( &mutex );
#endif
}

#ifdef WINDOWS_OS
DWORD WINAPI CameraUnit::ReadFromCameraUnitThread( LPVOID p_cameraunit )
{
      CHAR lpBuffer[256];
      DWORD nBytesRead;
      DWORD nCharsWritten;
      CameraUnit* pCameraUnit = (CameraUnit*)p_cameraunit;

      // Get input from our console and send it to child through the pipe.
      while (pCameraUnit->bRunThread)
      {
          EnterCriticalSection( &pCameraUnit->cs );
          size_t bufSize = pCameraUnit->buffer.size();
          LeaveCriticalSection( &pCameraUnit->cs );

        if( bufSize < SIZE_BUFFER ) {
	         if (!ReadFile(pCameraUnit->hOutputRead,lpBuffer,1,&nBytesRead,NULL) || !nBytesRead) {
                if (GetLastError() == ERROR_BROKEN_PIPE)
                   break; // pipe done - normal exit path.
                else
                   DisplayError("ReadFile"); // Something bad happened.
             }

		    EnterCriticalSection( &pCameraUnit->cs );
		    for( int i = 0; i < nBytesRead; ++i ) {
			    pCameraUnit->buffer.push_back( lpBuffer[ i ] );
		    }
		    LeaveCriticalSection( &pCameraUnit->cs );
        }
      }

      return 1;
}
#endif
#ifdef LINUX_OS
void* CameraUnit::ReadFromCameraUnitThread( void* p_cameraunit )
{
    char buffer[256];
    int nBytesRead;
    CameraUnit* pCameraUnit = (CameraUnit*)p_cameraunit;
    
    //cout << "ReadFromCameraUnitThread() is running..." << endl;

    while( pCameraUnit->bRunThread ) {
        pthread_mutex_lock( &pCameraUnit->mutex );
        ssize_t bufSize = pCameraUnit->buffer.size();
        pthread_mutex_unlock( &pCameraUnit->mutex );

	//cout << "bufSize=" << bufSize << endl;

	if( bufSize < SIZE_BUFFER ) {
	  //cout << "waiting for data from child process..." << pCameraUnit->pipe_c2p[R];
	  nBytesRead = ::read( pCameraUnit->pipe_c2p[R], buffer, 1 );
	  if( nBytesRead == 0 /*feof( pCameraUnit->pipe_c2p[R]*/ ) {
	      break;
	  } else if( nBytesRead == -1 ) {
	    perror( "pipetest" );
	    break;
	  }
	  //cout << "data received" << endl;
          pthread_mutex_lock( &pCameraUnit->mutex );
          for( int i = 0; i < nBytesRead; ++i ) {
              pCameraUnit->buffer.push_back( buffer[ i ] );
	  }
          pthread_mutex_unlock( &pCameraUnit->mutex );
	}	    
    }
    
    //cout << "ReadFromCameraUnitThrad is exitting..." << endl;

    return NULL;
}
#endif


#ifdef WINDOWS_OS
/////////////////////////////////////////////////////////////////////// 
// PrepAndLaunchRedirectedChild
// Sets up STARTUPINFO structure, and launches redirected child.
/////////////////////////////////////////////////////////////////////// 
void CameraUnit::PrepAndLaunchRedirectedChild(HANDLE hChildStdOut,
                                    HANDLE hChildStdIn,
                                    HANDLE hChildStdErr)
{
    PROCESS_INFORMATION pi;
    STARTUPINFO si;

    // Set up the start up info struct.
    ZeroMemory(&si,sizeof(STARTUPINFO));
    si.cb = sizeof(STARTUPINFO);
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdOutput = hChildStdOut;
    si.hStdInput  = hChildStdIn;
    si.hStdError  = hChildStdErr;
    // Use this if you want to hide the child:
    //     si.wShowWindow = SW_HIDE;
    // Note that dwFlags must include STARTF_USESHOWWINDOW if you want to
    // use the wShowWindow flags.


    // Launch the process that you want to redirect (in this case,
    // Child.exe). Make sure Child.exe is in the same directory as
    // redirect.c launch redirect from a command line to prevent location
    // confusion.
    ostringstream oss;
    if( strAddr != "192.168.1.18" ) { 
        oss << "C:\\Windows\\SUA\\bin\\rsh " << strAddr << " -l kumalab /home/kumalab/project/HumanTracking/bin/cameraunit --nowindow";
    } else {
        oss << ".\\cameraunit.exe --nowindow"; // debug code
    }
    if (!CreateProcess( NULL
                      , (LPSTR)oss.str().c_str()//"C:\\Windows\\SUA\\bin\\rsh 192.168.1.100 -l kumalab /home/kumalab/project/HumanTracking/bin/cameraunit --nowindow"
                      , NULL
                      , NULL
                      , TRUE
                      , CREATE_NEW_CONSOLE
                      , NULL
                      , NULL
                      , &si
                      , &pi) )
        DisplayError("CreateProcess");


    // Set global child process handle to cause threads to exit.
    hChildProcess = pi.hProcess;


    // Close any unnecessary handles.
    if (!CloseHandle(pi.hThread)) DisplayError("CloseHandle");
} 
#endif

#ifdef WINDOWS_OS
/////////////////////////////////////////////////////////////////////// 
// DisplayError
// Displays the error number and corresponding message.
/////////////////////////////////////////////////////////////////////// 
void CameraUnit::DisplayError(char *pszAPI)
{
    LPVOID lpvMessageBuffer;
    CHAR szPrintBuffer[512];
    DWORD nCharsWritten;

    FormatMessage(
            FORMAT_MESSAGE_ALLOCATE_BUFFER|FORMAT_MESSAGE_FROM_SYSTEM,
            NULL, GetLastError(),
            MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
            (LPTSTR)&lpvMessageBuffer, 0, NULL);

    wsprintf(szPrintBuffer,
        "ERROR: API    = %s.\n   error code = %d.\n   message    = %s.\n",
            pszAPI, GetLastError(), (char *)lpvMessageBuffer);

    WriteConsole(GetStdHandle(STD_OUTPUT_HANDLE),szPrintBuffer,
                    lstrlen(szPrintBuffer),&nCharsWritten,NULL);

    LocalFree(lpvMessageBuffer);
    ExitProcess(GetLastError());
}
#endif
