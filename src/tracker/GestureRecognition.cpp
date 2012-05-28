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


void GestureRecognition::set_silhouette( int id, unsigned long long timeStamp, const cv::Mat& silhouette )
{
    if( this->silhouette[ id ].size() != silhouette.size() ) {
        this->silhouette[ id ] = Mat::zeros( silhouette.size(), CV_8U );
    }

    for( int y = 0; y < silhouette.rows; ++y ) {
        for( int x = 0; x < silhouette.cols; ++x ) {
            if( ( silhouette.at<Vec3b>( y, x )[ 0 ] != 0 ) || ( silhouette.at<Vec3b>( y, x )[ 1 ] != 0 ) || ( silhouette.at<Vec3b>( y, x )[ 2 ] != 0 ) ) {
                this->silhouette[ id ].at<unsigned char>( y, x ) |= 255;
            }
        }
    }
}

Mat GestureRecognition::get_silhouette( int id )
{
    return silhouette[ id ];
}

void GestureRecognition::clear_silhouette( int id )
{
    memset( silhouette[ id ].data, 0, silhouette[ id ].rows * silhouette[ id ].cols );
}



void GestureRecognition::recognize( int id, unsigned long long timeStamp )
{
    if( silhouette.find( id ) == silhouette.end() ) {
        return;
    }

    if( ofs.find( id ) == ofs.end() ) {
        ostringstream oss;
        oss << strRecognitionResultPath << id << "_gesture.txt";
	ofs[ id ] = new ofstream( oss.str().c_str() );
        //ofs[ id ].open( oss.str().c_str() );
        if( !ofs[ id ]->is_open() ) {
              cerr << "Couldn't open " << oss.str() <<  endl;
              exit( 1 );
        }
        *ofs[ id ] << "timeStamp, mean_x, mean_y, row_highest, nAllPixels, nUpperPixels, nLowerPixels, nUpperLeftPixels, nUpperRightPixels, balance_LR" << endl; 
    }

    vector<int> nPixels_row( silhouette[ id ].rows );
    int row_highest = 0;
    int col_left = silhouette[ id ].cols - 1, col_right = 0;
    int mean_x = 0, mean_y = 0;

    for( int y = 0; y < silhouette[ id ].rows; ++y ) {
        nPixels_row[ y ] = 0;
        for( int x = 0; x < silhouette[ id ].cols; ++x ) {
            //if( ( silhouette[ id ].at<Vec3b>( y, x )[ 0 ] != 0 ) || ( silhouette[ id ].at<Vec3b>( y, x )[ 1 ] != 0 ) || ( silhouette[ id ].at<Vec3b>( y, x )[ 2 ] != 0 ) ) {
            if( silhouette[ id ].at<unsigned char>( y, x ) != 0 ) {
                ++nPixels_row[ y ];
                mean_x += x;
                mean_y += y;

                if( x < col_left ) {
                    col_left = x;
                }

                if( x > col_right ) {
                    col_right = x;
                }
            }
        }
        if( row_highest == 0 && nPixels_row[ y ] > 0 ) {
            row_highest = y;
        }
    }

    int nPixels_leftupper = 0;
    for( int x = col_left; x < ( col_left + col_right ) / 2; ++x ) {
        for( int y = row_highest; y < row_highest + 15/*( silhouette[ id ].rows + row_highest ) / 2*/; ++y ) {
            //if( ( silhouette.at<Vec3b>( y, x )[ 0 ] != 0 ) || ( silhouette.at<Vec3b>( y, x )[ 1 ] != 0 ) || ( silhouette.at<Vec3b>( y, x )[ 2 ] != 0 ) ) {
            if( silhouette[ id ].at<unsigned char>( y, x ) != 0 ) {
                ++nPixels_leftupper;
            }
        }
    }

    int nPixels_rightupper = 0;
    for( int x = ( col_left + col_right ) / 2; x <= col_right; ++x ) {
        for( int y = row_highest; y < row_highest + 15/*( silhouette[ id ].rows + row_highest ) / 2*/; ++y ) {
            //if( ( silhouette.at<Vec3b>( y, x )[ 0 ] != 0 ) || ( silhouette.at<Vec3b>( y, x )[ 1 ] != 0 ) || ( silhouette.at<Vec3b>( y, x )[ 2 ] != 0 ) ) {
            if( silhouette[ id ].at<unsigned char>( y, x ) != 0 ) {
                ++nPixels_rightupper;
            }
        }
    }

    const int nAllPixels = std::accumulate( nPixels_row.begin(), nPixels_row.end(), 0 );
    mean_x /= nAllPixels;
    mean_y /= nAllPixels;
    mean_y = silhouette[ id ].rows - mean_y;

    const int nUpperPixels = std::accumulate( nPixels_row.begin(), nPixels_row.begin() + ( silhouette[ id ].rows + 2 * row_highest ) / 3, 0 );
    const int nLowerPixels = nAllPixels - nUpperPixels;

    float balance_LR;
    if( nPixels_leftupper + nPixels_rightupper == 0 ) {
        balance_LR = 0.5f;
    } else {
        balance_LR = (float)nPixels_rightupper / (float)( nPixels_leftupper + nPixels_rightupper );
    }

    *ofs[ id ] << timeStamp << ", "
              << mean_x << ", "
              << mean_y << ", "
              << row_highest << ", "
              << nAllPixels << ", "
              << nUpperPixels << ", "
              << nLowerPixels << ", "
              << nPixels_leftupper << ", "
              << nPixels_rightupper << ", "
              << balance_LR << endl;

    *ofs[ id ] << flush;

    status[ id ] = ( balance_LR > 0.8f );
}
