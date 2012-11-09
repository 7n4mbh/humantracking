#ifndef __HEADER_RESULTRENDERER__
#define __HEADER_RESULTRENDERER__

#include <map>
#include <deque>

#include "../humantracking.h"

#ifdef WINDOWS_OS
#include <windows.h>
#endif

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "Trajectory.h"

struct _PEPMapInfoEx : public PEPMapInfo
{
    cv::Mat occupancy;
};
typedef struct _PEPMapInfoEx PEPMapInfoEx;

class ResultRenderer {
private:
    std::deque<PEPMapInfoEx> bufPEPMap;
    std::string strResultPEPMapVideoFilename;
    cv::VideoWriter pepmapVideoWriter;
    std::map<unsigned long long, std::map<int,cv::Point2d> > trackingResult;
    std::map<unsigned long long, std::multimap<int,cv::Point2d> > trackingResultExt;
    std::deque< std::map<int,cv::Point2d> > result_buffer;

public:
    ResultRenderer();
    virtual ~ResultRenderer();
    void init( std::string result_pepmapvideo_filename/*, std::string result_cameravideo_filename, std::string silhouette_path*/ );
    void AddPEPMapInfo( PEPMapInfoEx& pepmap );
    void AddResultTrajectories( const std::map< unsigned long long, std::map<int,cv::Point2d> >& result, const std::map<unsigned long long, std::multimap<int,cv::Point2d> >& ext_result );
    void Render();
};

#endif