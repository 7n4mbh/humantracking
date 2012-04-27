#ifndef __HEADER_TRACKINGRESULTRESOURCES__
#define __HEADER_TRACKINGRESULTRESOURCES__

#include "../humantracking.h"

#include <deque>
#include <map>
#include <string>

#ifdef WINDOWS_OS
#include <windows.h>
#endif

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "Trajectory.h"
#include "Viewer.h"

class TrackingResultResources {
private:
    bool bRunThread;
    int nUpdateViewRequest;
    int delayUpdate;
    std::deque<PEPMapInfo> bufPEPMap;
    std::map<unsigned long long, CameraImageInfo> bufCameraImage;
    std::map<unsigned long long, GeometryMapInfo> bufGeometry;
    std::map<unsigned long long, std::map<int,cv::Point2d> > trackingResult;
    std::map<unsigned long long, std::multimap<int,cv::Point2d> > trackingResultExt;
    std::string strResultFilename, strResultPEPMapVideoFilename, strResultCameraVideoFilename;
    std::map<int,cv::Point2d> posHumanStill;
    std::map<int,unsigned long> cntStill;
    Viewer* pViewer;
    cv::VideoWriter pepmapVideoWriter, cameraVideoWriter;
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
    void AddResultTrajectories( const std::map< unsigned long long, std::map<int,cv::Point2d> >& result, const std::map<unsigned long long, std::multimap<int,cv::Point2d> >& ext_result );
    void AddPEPMapInfo( PEPMapInfo& pepmap );
    void AddCameraImageInfo( CameraImageInfo& cam_image );
    void AddGeometryMapInfo( GeometryMapInfo& geometry );
    bool EnableViewWindow();
    bool TerminateViewWindow();
    void UpdateView();
    void init( std::string result_filename, std::string result_pepmapvideo_filename, std::string result_cameravideo_filename, Viewer* p_viewer );
    void clear();
    void SetDelayUpdate( int delay_update );
    int GetDelayUpdate();
    bool hasDataToDisplay();

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
