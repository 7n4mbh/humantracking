#include <fstream>
#include <numeric>

#include "../humantracking.h"
#include "GestureRecognition.h"

using namespace std;
using namespace cv;

extern float scale_m2px;

GestureRecognition::GestureRecognition()
{
}

GestureRecognition::~GestureRecognition()
{
}

void GestureRecognition::init( std::string recognition_result_path )
{
    strRecognitionResultPath = recognition_result_path;
}


void GestureRecognition::SetSilhouette( int id, unsigned long long timeStamp, const cv::Mat& silhouette )
{
    if( ofs.find( id ) == ofs.end() ) {
        ostringstream oss;
        oss << strRecognitionResultPath << "id" << "_gesture.txt";
        ofs[ id ].open( oss.str().c_str() );
        if( !ofs[ id ].is_open() ) {
              cerr << "Couldn't open " << oss.str() <<  endl;
              exit( 1 );
        }
        ofs[ id ] << "timeStamp, mean_x, mean_y, row_highest, nAllPixels, nUpperPixels, nLowerPixels" << endl; 
    }

    vector<int> nPixels( silhouette.rows );
    int row_highest = 0;
    int mean_x = 0, mean_y = 0;

    for( int y = 0; y < silhouette.rows; ++y ) {
        nPixels[ y ] = 0;
        for( int x = 0; x < silhouette.cols; ++x ) {
            if( ( silhouette.at<Vec3b>( y, x )[ 0 ] != 0 ) || ( silhouette.at<Vec3b>( y, x )[ 1 ] != 0 ) || ( silhouette.at<Vec3b>( y, x )[ 2 ] != 0 ) ) {
                ++nPixels[ y ];
                mean_x += x;
                mean_y += y;
            }
        }
        if( row_highest != 0 && nPixels[ y ] > 0 ) {
            row_highest = y;
        }
    }

    const int nAllPixels = std::accumulate( nPixels.begin(), nPixels.end(), 0 );
    mean_x /= nAllPixels;
    mean_y /= nAllPixels;
    mean_y = silhouette.rows - mean_y;

    const int nUpperPixels = std::accumulate( nPixels.begin(), nPixels.begin() + ( silhouette.rows + row_highest ) / 2, 0 );
    const int nLowerPixels = nAllPixels - nUpperPixels;


    ofs[ id ] << timeStamp << ", "
              << mean_x << ", "
              << mean_y << ", "
              << row_highest << ", "
              << nAllPixels << ", "
              << nUpperPixels << ", "
              << nLowerPixels << endl;

    ofs[ id ] << flush;
}