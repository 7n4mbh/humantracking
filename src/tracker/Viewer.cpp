#include <sstream>

#include "../humantracking.h"
#include "Viewer.h"

using namespace std;

Viewer::Viewer()
{
#ifdef WINDOWS_OS
    hProcess = NULL;
    hWrite = NULL;
    InitializeCriticalSection( &cs );
#endif

#ifdef LINUX_OS
    pthread_mutex_init( &mutex, NULL );
#endif
}

Viewer::~Viewer()
{
#ifdef WINDOWS_OS
    hWrite = hProcess = NULL;
    DeleteCriticalSection( &cs );
#endif
#ifdef LINUX_OS
    fp = NULL;
    pthread_mutex_destroy( &mutex );
#endif
}

void Viewer::exec()
{
#ifdef WINDOWS_OS
    HANDLE hRead; 
    SECURITY_ATTRIBUTES sa; 
    STARTUPINFO si; 
    PROCESS_INFORMATION pi; 
    
    sa.nLength = sizeof(SECURITY_ATTRIBUTES); 
    sa.lpSecurityDescriptor = NULL; 
    sa.bInheritHandle = TRUE; 
    if ( !CreatePipe( &hRead, &hWrite, &sa, 0 ) ) { 
        cerr << "Faild in creating a pipe" << endl;
        ::exit( 1 );
    } 
    
    if ( !DuplicateHandle( GetCurrentProcess(), //ソースプロセス 
                           hWrite, //duplicateするハンドル(オリジナルハンドル) 
                           GetCurrentProcess(), //ターゲットプロセス(行先) 
                           NULL, //複製ハンドルへのポインタ(コピーハンドル) 
                           0, //アクセス権 
                           FALSE, //子供がハンドルを継承するかどうか 
                           DUPLICATE_SAME_ACCESS)) { //オプション 
        cerr << "Error occured in DuplicateHandle()." << endl;
        CloseHandle(hWrite); 
        CloseHandle(hRead); 
        hWrite = NULL; 
        ::exit( 1 ); 
    } 
    
    memset(&si, 0, sizeof(STARTUPINFO)); 
    si.cb = sizeof(STARTUPINFO); 
    si.dwFlags = STARTF_USESTDHANDLES; 
    si.hStdInput = hRead; 
    si.hStdOutput = GetStdHandle( STD_OUTPUT_HANDLE ); 
    si.hStdError = GetStdHandle( STD_ERROR_HANDLE ); 
    if (!CreateProcess(NULL, "viewer.exe", NULL, NULL, TRUE, 0, NULL, NULL, &si, &pi)) { 
        cerr << "Error occured in Create Process." << endl;
        CloseHandle(hWrite); 
        hWrite = NULL; 
        ::exit( 1 );
    } 
    
    hProcess = pi.hProcess; 
    CloseHandle(pi.hThread); 
    CloseHandle(hRead); 
    
    return;
#endif

#ifdef LINUX_OS
    if((fp=popen("/home/kenichiro/project/HumanTracking/bin/viewer","w"))==NULL){
        fprintf(stderr,"error!!!\n");
        ::exit(-1);
    }
#endif
}

void Viewer::exit()
{
#ifdef WINDOWS_OS
    if(hProcess) {
        CloseHandle(hProcess);
        hProcess = NULL;
    } 
    
    if (hWrite) {
        CloseHandle(hWrite);    
        hWrite = NULL;
    }
#endif
#ifdef LINUX_OS
    pclose( fp );
    fp = NULL;
#endif
}

void Viewer::send( string msg )
{
#ifdef WINDOWS_OS
    EnterCriticalSection( &cs );

    DWORD dwResult, dwError;
    if( !WriteFile(hWrite, msg.c_str(), msg.size(), &dwResult, NULL) ) { 
        dwError = GetLastError(); 
        if (dwError == ERROR_BROKEN_PIPE || dwError == ERROR_NO_DATA) { 
            cerr << "No pipe available." << endl;
            ::exit( 1 );
        } else { 
            cerr << "Error occured in WriteFile()" << endl;
            ::exit( 1 );
        } 
        
        CloseHandle(hWrite); 
        hWrite = NULL;
        CloseHandle(hProcess); 
        hProcess = NULL; 
    }

    LeaveCriticalSection( &cs );
#endif

#ifdef LINUX_OS
    pthread_mutex_lock( &mutex );
    cout << "## debug code! ## Viewer::send()" << endl;
    msg += "\n";
    if( fputs( msg.c_str(), fp ) == -1 ) {
      cerr << "Error occured in fputs()." << endl;
      ::exit( -1 );
    }
    fflush( fp );
    pthread_mutex_unlock( &mutex );
#endif
}

void Viewer::SetStartTime( unsigned long long time )
{
    ostringstream oss;
    oss << "StartTime=" << time << "\r";
    send( oss.str() );
}

void Viewer::SetTrackingBlock( unsigned long long start, unsigned long long end )
{
    ostringstream oss;
    oss << "TrackingBlock=" << start << "," << end << "\r";
    send( oss.str() );
}

void Viewer::SetPEPMap( const PEPMapInfo& pepmap )
{
    ostringstream oss;
    oss << "PEPMap=" << pepmap.timeStamp << "\r";
    send( oss.str() );
}

void Viewer::SetTrackingStatus( int status )
{
    ostringstream oss;
    oss << "TrackingStatus=" << status << "\r";
    send( oss.str() );
}

void Viewer::SetPos( unsigned long long time )
{
    ostringstream oss;
    oss << "Time=" << time << "\r";
    send( oss.str() );
}
