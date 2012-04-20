#ifndef __HEADER_VIEWER__
#define __HEADER_VIEWER__

#include "../humantracking.h"

#ifdef WINDOWS_OS
#include <windows.h>
#endif

#include <string>

class Viewer{
public:

private:
#ifdef WINDOWS_OS
    HANDLE hWrite, hProcess;
    CRITICAL_SECTION cs;
#endif
#ifdef LINUX_OS
    FILE* fp;
    pthread_mutex_t mutex;
#endif

public:
    Viewer();
    virtual ~Viewer();
    void exec();
    void exit();
    void send( std::string msg );
    void SetStartTime( unsigned long long time );
    void SetTrackingBlock( unsigned long long start, unsigned long long end );
    void SetPEPMap( const PEPMapInfo& pepmap );
    void SetCameraImage( const CameraImageInfo& cam_image );
    void SetTrackingStatus( int status );
    void SetPos( unsigned long long time );
    void SetResult( const std::map< unsigned long long, std::map<int,cv::Point2d> >& result );
};

#endif