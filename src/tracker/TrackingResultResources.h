#ifndef __HEADER_TRACKINGRESULTRESOURCES__
#define __HEADER_TRACKINGRESULTRESOURCES__

#include "../humantracking.h"

#include <deque>
#include <map>

#ifdef WINDOWS_OS
#include <windows.h>
#endif

#include "opencv/cv.h"

#include "Trajectory.h"

class TrackingResultResources {
private:
    bool bRunThread;
    int nUpdateViewRequest;
    int delayUpdate;
    std::deque<PEPMapInfo> bufPEPMap;
    std::map<unsigned long long, std::map<int,cv::Point2d> > trackingResult;
#ifdef WINDOWS_OS
    HANDLE hThread;
    DWORD ThreadId;
    CRITICAL_SECTION cs;
#endif
#ifdef LINUX_OS
    pthread_t thread;
    pthread_mutex_t mutex;
#endif

public:
    TrackingResultResources();
    virtual ~TrackingResultResources();
    void AddResultTrajectories( const std::map< unsigned long long, std::map<int,cv::Point2d> >& result );
    void AddPEPMapInfo( PEPMapInfo& pepmap );
    bool EnableViewWindow();
    bool TerminateViewWindow();
    void UpdateView();
    void clear();
    void SetDelayUpdate( int delay_update );
    int GetDelayUpdate();

private:
#ifdef WINDOWS_OS
   friend DWORD WINAPI ViewThread( LPVOID p_tracking_result_resources );
   static DWORD WINAPI ViewThread( LPVOID p_tracking_result_resources );
#endif
#ifdef LINUX_OS
   friend void* ViewThread( void* p_tracking_result_resources );
   static void* ViewThread( void* p_tracking_result_resources );
#endif
};

#endif
