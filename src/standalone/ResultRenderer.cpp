#include "../humantracking.h"
#include "ResultRenderer.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "zlib.h"

using namespace std;
using namespace cv;

extern float roi_width, roi_height;
extern float roi_x, roi_y;
extern float scale_m2px, scale_m2px_silhouette;

ResultRenderer::ResultRenderer()
{
}

ResultRenderer::~ResultRenderer()
{
}

void ResultRenderer::init( std::string result_pepmapvideo_filename/*, std::string result_cameravideo_filename, std::string silhouette_path*/ )
{
    bufPEPMap.clear();
    trackingResult.clear();
    trackingResultExt.clear();
    result_buffer.clear();

    strResultPEPMapVideoFilename = result_pepmapvideo_filename;

    if( !pepmapVideoWriter.open( strResultPEPMapVideoFilename, CV_FOURCC('X','V','I','D'), 40, Size( (int)( roi_width * 80 ), (int)( roi_height * 80 ) ) ) ) {
      cerr << "Couldn't open " << strResultPEPMapVideoFilename << "." <<  endl;
      exit( 1 );
    }
}

void ResultRenderer::AddPEPMapInfo( PEPMapInfoEx& pepmap )
{
    bufPEPMap.push_back( pepmap );
}

void ResultRenderer::AddResultTrajectories( const std::map< unsigned long long, std::map<int,cv::Point2d> >& result, const std::map<unsigned long long, std::multimap<int,cv::Point2d> >& ext_result )
{
    {
        map< unsigned long long, map<int,Point2d> >::const_iterator it = result.begin();
        for( ; it != result.end(); ++it ) {
            trackingResult[ it->first ].insert( it->second.begin(), it->second.end() );
        }
    }

    {
        std::map<unsigned long long, std::multimap<int,cv::Point2d> >::const_iterator it = ext_result.begin();
        for( ; it != ext_result.end(); ++it ) {
            trackingResultExt[ it->first ].insert( it->second.begin(), it->second.end() );
        }
    }
}

void ResultRenderer::Render()
{
    Mat img_display( (int)( roi_height * scale_m2px ), (int)( roi_width * scale_m2px ), CV_8UC3 );
    Mat img_occupancy( (int)( scale_m2px * roi_height ), (int)( scale_m2px * roi_width ), CV_8U );

    while( !bufPEPMap.empty() && !trackingResult.empty() ) {
        PEPMapInfoEx pepmap = bufPEPMap.front();
        unsigned long long timeStamp = trackingResult.begin()->first;
        map<int,Point2d> posHuman = trackingResult.begin()->second;

        pepmap.occupancy.convertTo( img_occupancy, CV_8U );
        cvtColor( img_occupancy, img_display, CV_GRAY2BGR );

        if( pepmap.timeStamp <= timeStamp ) {
            
            bufPEPMap.pop_front();

            //
            // Create a time info string
            time_t _sec = pepmap.timeStamp / 1000000ULL;
            string strTime;
            for( int i = 0; i < 10; ++i ) {
                strTime = string( ctime( &_sec ) );
                if( strTime.size() ) {
                    break;
                }
            }
            strTime.erase( strTime.size() - 1 );
            
            
            //
            // Store the position of the people for drawing trajectories
            result_buffer.push_back( posHuman );
            if( result_buffer.size() > 20 ) {
                result_buffer.pop_front();
            }


            //
            // Draw trajectories
            map<int, vector<Point2d> > trajectory_to_draw;
            for( deque< map<int,Point2d> >::iterator itPosHuman = result_buffer.begin(); itPosHuman != result_buffer.end(); ++itPosHuman ) {
                for( map<int,Point2d>::iterator it = itPosHuman->begin(); it != itPosHuman->end(); ++it ) {
                    trajectory_to_draw[ it->first ].push_back( it->second );
                }
            }

            int old_col, old_row;
            for( map< int, vector<Point2d> >::iterator itHuman = trajectory_to_draw.begin(); itHuman != trajectory_to_draw.end(); ++itHuman ) {
                for( vector<Point2d>::iterator it = itHuman->second.begin(); it != itHuman->second.end(); ++it ) {
                    int row = (int)( scale_m2px * ( ( it->x - roi_x ) + roi_height / 2.0f ) );
                    int col = (int)( scale_m2px * ( ( it->y - roi_y ) + roi_width / 2.0f ) );
                    circle( img_display, Point( col, row ), 3, color_table[ itHuman->first % sizeColorTable ], -1 );
                    if( it != itHuman->second.begin() ) {
                        line( img_display, Point( col, row ), Point( old_col, old_row ), color_table[ itHuman->first % sizeColorTable ], 6 );
                    }
                    old_col = col;
                    old_row = row;
                }
            }


            //
            // Draw Human Regions
            map<unsigned long long,multimap<int,Point2d> >::iterator itRegionHuman = trackingResultExt.find( pepmap.timeStamp );
            map<int,int> geometry_to_ID;
            if( itRegionHuman != trackingResultExt.end() ) {
                for( multimap<int,Point2d>::iterator itHuman = itRegionHuman->second.begin(); itHuman != itRegionHuman->second.end(); ++itHuman ) {
                    int _row_on_pepmap = scale_m2px * ( ( itHuman->second.x - roi_x ) + roi_height / 2.0f );
                    int _col_on_pepmap = scale_m2px * ( ( itHuman->second.y - roi_y ) + roi_width / 2.0f );
                    for( int col_on_pepmap = max( _col_on_pepmap - 2, 0 ); col_on_pepmap < min( _col_on_pepmap + 2, pepmap.occupancy.cols - 1 ); ++col_on_pepmap ) {
                        for( int row_on_pepmap = max( _row_on_pepmap - 2, 0 ); row_on_pepmap < min( _row_on_pepmap + 2, pepmap.occupancy.rows - 1 ); ++row_on_pepmap ) {
                            int row = (int)( ( (float)img_display.size().height / (float)img_display.size().height ) * row_on_pepmap );
                            int col = (int)( ( (float)img_display.size().width / (float)img_display.size().width ) * col_on_pepmap );
                            //int keyval = col_on_pepmap * occupancy.rows + row_on_pepmap + 1;
                            int keyval = row_on_pepmap * pepmap.occupancy.cols + col_on_pepmap + 1;
                            line( img_display, Point( col, row ), Point( col, row ), color_table[ itHuman->first % sizeColorTable ] );
                            //rectangle( img_display, Point( col - scale_width / 2, row - scale_height / 2 ), Point( col + scale_width / 2, row + scale_height / 2 ), color_table[ itHuman->first % sizeColorTable ], CV_FILLED );
                            geometry_to_ID[ keyval ] = itHuman->first;
                        }
                    }
                }
            }


            //
            // Show/Record the rendered image
            imshow( "Tracking Result", img_display );
            pepmapVideoWriter.write( img_display );
            (void)cvWaitKey( 10 );
        } else {
                const unsigned long long time_pepmap = trackingResult.begin()->first;
                trackingResultExt.erase( trackingResultExt.begin()
                                       , trackingResultExt.lower_bound( time_pepmap )  );
                trackingResult.erase( trackingResult.begin() );
        }
    }
}