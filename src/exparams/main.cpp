#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <map>

#include "../humantracking.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

using namespace std;
using namespace cv;

int main( int argc, char *argv[] )
{
    string strExParamsFileForBaseCam;
    string strBaseCamCornersFile, strCornersFile;

    if( argc != 4 ) {
        cerr << "Invalid Parameters." << endl;
        exit( 1 );
    }

    strExParamsFileForBaseCam = argv[ 1 ];
    strBaseCamCornersFile = argv[ 2 ];
    strCornersFile = argv[ 3 ];

    Mat w_H_base( 4, 4, CV_32F ); 
    // load extrinsic parameters of the base camera
    {
        ifstream ifs( strExParamsFileForBaseCam );
        if( !ifs ) {
            cerr << "Error occured in loading " << strExParamsFileForBaseCam << "." << endl;
            exit( 1 );
        }
        for( int i = 0; i < 12; ++i ) {
            //ifs >> a;
            ifs >> w_H_base.at<float>( i / 4, i % 4 );
        }
        w_H_base.at<float>( 0, 3 ) = w_H_base.at<float>( 0, 3 ) / 1000.0f;
        w_H_base.at<float>( 1, 3 ) = w_H_base.at<float>( 1, 3 ) / 1000.0f;
        w_H_base.at<float>( 2, 3 ) = w_H_base.at<float>( 2, 3 ) / 1000.0f;
    }
    w_H_base.at<float>( 3, 0 ) = w_H_base.at<float>( 3, 1 ) = w_H_base.at<float>( 3, 2 ) = 0.0f;
    w_H_base.at<float>( 3, 3 ) = 1.0f;

    char buf[ 1000 ];

    // load corner positons of the base camera;
    vector<Point3f> base;
    {
        ifstream ifs( strBaseCamCornersFile );
        if( !ifs ) {
            cerr << "Error occured in loading " << strBaseCamCornersFile << "." << endl;
            exit( 1 );
        }
        while( !ifs.eof() ) {
            float u, v;
            float x, y, z;
            ifs.getline( buf, sizeof( buf ) );
            if( buf[ 0 ] == '\0' ) {
                continue;
            }

            string str_tmp;
            int i = 0;
            while( buf[ i ] != '\0' ) {
                if( buf[ i ] != ',' ) {
                    str_tmp.push_back( buf[ i ] );
                }
                ++i;
            }

            istringstream iss( str_tmp );
            iss >> u >> v >> x >> y >> z;
            base.push_back( Point3f( x, y, z ) );
        }
    }

    // load corner position of the camera
    vector<Point3f> camera;
    {
        ifstream ifs( strCornersFile );
        if( !ifs ) {
            cerr << "Error occured in loading " << strCornersFile << "." << endl;
            exit( 1 );
        }
        while( !ifs.eof() ) {
            float u, v;
            float x, y, z;
            ifs.getline( buf, sizeof( buf ) );
            if( buf[ 0 ] == '\0' ) {
                continue;
            }

            string str_tmp;
            int i = 0;
            while( buf[ i ] != '\0' ) {
                if( buf[ i ] != ',' ) {
                    str_tmp.push_back( buf[ i ] );
                }
                ++i;
            }

            istringstream iss( str_tmp );
            iss >> u >> v >> x >> y >> z;
            camera.push_back( Point3f( x, y, z ) );
        }
    }

    if( base.size() != camera.size() ) {
        cerr << "# of coners in the files should be equal. (base=" << base.size() << ",camera=" << camera.size() << ")" << endl;
        exit( 1 );
    }

    const int nCorners = base.size();

    // Calculate the transform matrix H
    Mat H = Mat::zeros( 4, 4, CV_32F );
    Mat base_H_cam = Mat::zeros( 4, 4, CV_32F );
    Mat A = Mat::zeros( 3 * nCorners, 12, CV_32F );
    Mat b = Mat::zeros( 3 * nCorners, 1, CV_32F );
    Mat p;
    {
        for( int i = 0; i < nCorners; ++i ) {
            const int row = 3 * i;
            A.at<float>( row, 0 ) = A.at<float>( row + 1, 3 ) = A.at<float>( row + 2, 6 ) = camera[ i ].x;
            A.at<float>( row, 1 ) = A.at<float>( row + 1, 4 ) = A.at<float>( row + 2, 7 ) = camera[ i ].y;
            A.at<float>( row, 2 ) = A.at<float>( row + 1, 5 ) = A.at<float>( row + 2, 8 ) = camera[ i ].z;
            A.at<float>( row, 9 ) = A.at<float>( row + 1, 10 ) = A.at<float>( row + 2, 11 ) = 1.0f;

            b.at<float>( row, 0 ) = base[ i ].x;
            b.at<float>( row + 1, 0 ) = base[ i ].y;
            b.at<float>( row + 2, 0 ) = base[ i ].z;
        }
    }

    solve( A, b, p, DECOMP_SVD );

    base_H_cam.at<float>( 0, 0 ) = p.at<float>( 0,  0 );
    base_H_cam.at<float>( 0, 1 ) = p.at<float>( 1,  0 );
    base_H_cam.at<float>( 0, 2 ) = p.at<float>( 2,  0 );
    base_H_cam.at<float>( 0, 3 ) = p.at<float>( 9,  0 );
    base_H_cam.at<float>( 1, 0 ) = p.at<float>( 3,  0 );
    base_H_cam.at<float>( 1, 1 ) = p.at<float>( 4,  0 );
    base_H_cam.at<float>( 1, 2 ) = p.at<float>( 5,  0 );
    base_H_cam.at<float>( 1, 3 ) = p.at<float>( 10, 0 );
    base_H_cam.at<float>( 2, 0 ) = p.at<float>( 6,  0 );
    base_H_cam.at<float>( 2, 1 ) = p.at<float>( 7,  0 );
    base_H_cam.at<float>( 2, 2 ) = p.at<float>( 8,  0 );
    base_H_cam.at<float>( 2, 3 ) = p.at<float>( 11, 0 );
    base_H_cam.at<float>( 3, 0 ) = 0.0f;
    base_H_cam.at<float>( 3, 1 ) = 0.0f;
    base_H_cam.at<float>( 3, 2 ) = 0.0f;
    base_H_cam.at<float>( 3, 3 ) = 1.0f;

    cout << "p:" << endl;
    cout << p.at<float>( 0, 0 ) << ", "
         << p.at<float>( 1, 0 ) << ", "
         << p.at<float>( 2, 0 ) << ", "
         << p.at<float>( 9, 0 ) << endl
         << p.at<float>( 3, 0 ) << ", "
         << p.at<float>( 4, 0 ) << ", "
         << p.at<float>( 5, 0 ) << ", "
         << p.at<float>( 10, 0 ) << endl
         << p.at<float>( 6, 0 ) << ", "
         << p.at<float>( 7, 0 ) << ", "
         << p.at<float>( 8, 0 ) << ", "
         << p.at<float>( 11, 0 ) << endl;

    H = w_H_base * base_H_cam;

    cout << "Done." << endl;

    ofstream ofs("Extrinsic.txt" );
    ofs <<  H.at<float>( 0, 0 ) << " "
        <<  H.at<float>( 0, 1 ) << " "
        <<  H.at<float>( 0, 2 ) << " "
        <<  H.at<float>( 0, 3 ) * 1.0e3 << endl
        <<  H.at<float>( 1, 0 ) << " "
        <<  H.at<float>( 1, 1 ) << " "
        <<  H.at<float>( 1, 2 ) << " "
        <<  H.at<float>( 1, 3 ) * 1.0e3 << endl
        <<  H.at<float>( 2, 0 ) << " "
        <<  H.at<float>( 2, 1 ) << " "
        <<  H.at<float>( 2, 2 ) << " "
        <<  H.at<float>( 2, 3 ) * 1.0e3 << endl;

    cout <<  H.at<float>( 0, 0 ) << " "
         <<  H.at<float>( 0, 1 ) << " "
         <<  H.at<float>( 0, 2 ) << " "
         <<  H.at<float>( 0, 3 ) * 1.0e3 << endl
         <<  H.at<float>( 1, 0 ) << " "
         <<  H.at<float>( 1, 1 ) << " "
         <<  H.at<float>( 1, 2 ) << " "
         <<  H.at<float>( 1, 3 ) * 1.0e3 << endl
         <<  H.at<float>( 2, 0 ) << " "
         <<  H.at<float>( 2, 1 ) << " "
         <<  H.at<float>( 2, 2 ) << " "
         <<  H.at<float>( 2, 3 ) * 1.0e3 << endl;

    return 0;
}