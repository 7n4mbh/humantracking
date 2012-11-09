#ifndef __HEADER_RESULTRENDERER__
#define __HEADER_RESULTRENDERER__

#include <map>
#include <deque>
#include <string>

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

struct _GeometryMapInfoEx : public GeometryMapInfo
{
    cv::Mat geometry;
};

struct _CameraImageInfoEx : public CameraImageInfo
{
    cv::Mat image;
};

typedef struct _PEPMapInfoEx PEPMapInfoEx;
typedef struct _GeometryMapInfoEx GeometryMapInfoEx;
typedef struct _CameraImageInfoEx CameraImageInfoEx;

class ResultRenderer {
private:
    std::deque<PEPMapInfoEx> bufPEPMap;
    std::map<std::string, CameraImageInfoEx> bufCameraImage;
    std::map<std::string, GeometryMapInfoEx> bufGeometry;

    std::map<unsigned long long, std::map<int,cv::Point2d> > trackingResult;
    std::map<unsigned long long, std::multimap<int,cv::Point2d> > trackingResultExt;
    std::deque< std::map<int,cv::Point2d> > result_buffer;
    
    std::string strResultPEPMapVideoFilename;
    cv::VideoWriter pepmapVideoWriter;
    std::map<unsigned long long,cv::VideoWriter> cameraVideoWriter;

public:
    ResultRenderer();
    virtual ~ResultRenderer();
    void init( std::string result_pepmapvideo_filename, std::map<unsigned long long,std::string> result_cameravideo_filename/*, std::string result_cameravideo_filename, std::string silhouette_path*/ );
    void AddPEPMapInfo( PEPMapInfoEx& pepmap );
    void AddCameraImageInfo( CameraImageInfoEx& cam_image );
    void AddGeometryMapInfo( GeometryMapInfoEx& geometry );
    void AddResultTrajectories( const std::map< unsigned long long, std::map<int,cv::Point2d> >& result, const std::map<unsigned long long, std::multimap<int,cv::Point2d> >& ext_result );
    void Render();
};

#endif