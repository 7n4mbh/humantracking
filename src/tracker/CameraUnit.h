// http://support.microsoft.com/kb/190351/ja

#ifndef __HEADER_CAMERAUNIT__
#define __HEADER_CAMERAUNIT__

#include <deque>
#include <string>

#if defined(WINDOWS) || defined(_WIN32)
#define WINDOWS_OS
#elif defined(linux) || defined(__linux__)
#define LINUX_OS
#endif

#ifdef WINDOWS_OS
#include <windows.h>
#endif

#define SIZE_BUFFER 1000

class CameraUnit{
public:

private:
   HANDLE hChildProcess;
   HANDLE hStdIn; // Handle to parents std input.
   BOOL bRunThread;
	HANDLE hOutputRead, hInputWrite;
      HANDLE hThread;
      DWORD ThreadId;
	  std::deque<char> buffer;
	  CRITICAL_SECTION cs;

public:
	CameraUnit();

	void connect();
	void disconnect();
    void read( char* buf, size_t size );
	void readline( char* buf, size_t size );
	void send( const char* buf, size_t size );
    bool hasData();
    void ClearBuffer();

private:
   static void DisplayError(char *pszAPI);
   void PrepAndLaunchRedirectedChild(HANDLE hChildStdOut,
                                     HANDLE hChildStdIn,
                                     HANDLE hChildStdErr);
   friend DWORD WINAPI ReadFromCameraUnitThread( LPVOID p_cameraunit );
   static DWORD WINAPI ReadFromCameraUnitThread( LPVOID p_cameraunit );

};

#endif