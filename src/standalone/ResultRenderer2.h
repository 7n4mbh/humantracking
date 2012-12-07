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

class ResultRenderer2 {
private:
    std::map<std::string, PEPMapInfoEx> bufPEPMap;
    std::map<std::string, CameraImageInfoEx> bufCameraImage;
    std::map<std::string, GeometryMapInfoEx> bufGeometry;
    std::map<std::string, GeometryMapInfoEx> bufSilhouette;
    
    std::set<std::string> bufTimeStamp;
    unsigned long long time_video, time_start;
    int frame;
    bool flgFirst;

    std::map<unsigned long long, std::map<int,cv::Point2d> > trackingResult;
    std::map<unsigned long long, std::multimap<int,cv::Point2d> > trackingResultExt;
    std::deque< std::map<int,cv::Point2d> > result_buffer;
    std::map< int, std::map<unsigned long long,cv::Point> > id_to_trace;
    
    std::string strResultPEPMapVideoFilename, strResultPEPMapWithoutRegionFilename;
    std::string strResultTrajectoryVideoFilename;
    std::string strResultCameraVideoFilename;
    std::string strVideoFilePath;
    double fps;
    cv::Mat image_occupancy_record;
    std::map<unsigned long long,cv::Mat> image_camera_record;
    std::map<int, std::map<unsigned long long,cv::Mat> > image_silhouette;
    std::map<int,cv::Mat> image_silhouette2;
    cv::VideoWriter pepmapVideoWriter, pepmapWithoutRegionVideoWriter;
    cv::VideoWriter trajectoryVideoWriter;
    cv::VideoWriter cameraVideoWriter;
    std::map<int,cv::VideoWriter> silhouetteVideoWriter;
    std::map<int,cv::VideoWriter> silhouetteVideoWriter2;

public:
    ResultRenderer2();
    virtual ~ResultRenderer2();
    void init( std::string result_pepmapvideo_filename, std::string result_pepmapvideo_widthout_region_filename, std::string resut_trajectoryvideo_filename, std::string result_cameravideo_filename, double fps/*, std::string result_cameravideo_filename, std::string silhouette_path*/ );
    void AddPEPMapInfo( PEPMapInfoEx& pepmap );
    void AddCameraImageInfo( CameraImageInfoEx& cam_image );
    void AddGeometryMapInfo( GeometryMapInfoEx& geometry );
    void AddSilhouetteMapInfo( GeometryMapInfoEx& silhouette );
    void AddResultTrajectories( const std::map< unsigned long long, std::map<int,cv::Point2d> >& result, const std::map<unsigned long long, std::multimap<int,cv::Point2d> >& ext_result );
    void Render();
};

#endif