// http://support.microsoft.com/kb/190351/ja

#ifndef __HEADER_CAMERAUNIT__
#define __HEADER_CAMERAUNIT__

#include <deque>
#include <string>

#include "../humantracking.h"

#ifdef WINDOWS_OS
#include <windows.h>
#endif

class CameraUnit{
public:

private:
#ifdef WINDOWS_OS
    HANDLE hChildProcess;
    //HANDLE hStdIn; // Handle to parents std input.
    BOOL bRunThread;
    HANDLE hOutputRead, hInputWrite;
    HANDLE hThread;
    DWORD ThreadId;
    CRITICAL_SECTION cs;
#endif
#ifdef LINUX_OS
    int pipe_c2p[2]; // Pipe: Parent(pipe_c2p[R]) <- Child(pipe_c2p[W])
    int pipe_p2c[2]; // Pipe: Parent(pipe_p2c[W]) -> Child(pip2_p2c[R])
    bool bRunThread;
    pthread_t thread;
    pthread_mutex_t mutex;
#endif
    std::deque<char> buffer;
    std::string strAddr;

public:
    CameraUnit();

    void connect( const std::string& addr );
    void disconnect();
    void read( char* buf, size_t size );
    void readline( char* buf, size_t size );
    void send( const char* buf, size_t size );
    bool hasData();
    void ClearBuffer();

private:
#ifdef WINDOWS_OS
   static void DisplayError(char *pszAPI);
   void PrepAndLaunchRedirectedChild(HANDLE hChildStdOut,
                                     HANDLE hChildStdIn,
                                     HANDLE hChildStdErr);
#endif
#ifdef WINDOWS_OS
   friend DWORD WINAPI ReadFromCameraUnitThread( LPVOID p_cameraunit );
   static DWORD WINAPI ReadFromCameraUnitThread( LPVOID p_cameraunit );
#endif
#ifdef LINUX_OS
   friend void* ReadFromCameraUnitThread( void* p_cameraunit );
   static void* ReadFromCameraUnitThread( void* p_cameraunit );
#endif

};

#endif
