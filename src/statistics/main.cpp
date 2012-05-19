#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include "../humantracking.h"

using namespace std;

void getfilename( const string src, string* p_str_path, string* p_str_name, string* p_str_noextname )
{
    int idxExt = src.rfind( ".", src.npos );
#ifdef WINDOWS_OS
    int idxPath = src.rfind( "\\", src.npos );
#else
    int idxPath = src.rfind( "/", src.npos );
#endif
    *p_str_path = src.substr( 0, idxPath + 1 );
    *p_str_name = src.substr( idxPath + 1 );
    *p_str_noextname = src.substr( idxPath + 1, idxExt - idxPath - 1 );
}

int main( int argc, char *argv[] )
{
    string strPEPMapFile;
    strPEPMapFile = string( argv[ 1 ] );

    ifstream ifs( strPEPMapFile );
    unsigned long long timeStamp;
    unsigned long long time_start, time_until;
    bool flgFirst = true;

    if( !ifs.is_open() )  {
        cerr << "Couldn't open " << strPEPMapFile << "." << endl;
        exit( 1 );
    }

    string strPath, strName, strNoextName;
    getfilename( strPEPMapFile, &strPath, &strName, &strNoextName );

    ostringstream oss;
    oss << strPath << "statistics_" << strNoextName << ".csv";
    ofstream ofs( oss.str().c_str() );
    if( !ofs.is_open() )  {
        cerr << "Couldn't open " << oss.str() << "." << endl;
        exit( 1 );
    }
    ofs << "total_size, pepmap_size, disparity_size, camimage_size, geometry_size, geometry2_size" << endl;

    int total_size, pepmap_size, disparity_size, camimage_size, geometry_size, geometry2_size;
    string str, str_serialNumber, str_timeStamp, str_width, str_height, str_datasize, str_data;

    while( !ifs.eof() ) {
        int _pepmap_size = 0, _disparity_size = 0, _camimage_size = 0, _geometry_size = 0, _geometry2_size = 0;
        ifs >> str;
		if( str.find( "<PEPMap>" ) != str.npos ) {
            ifs >> str_serialNumber 
                >> str_timeStamp 
                >> str_datasize 
                >> str_data;
            _pepmap_size = str.length() 
                 + str_serialNumber.length()
                 + str_timeStamp.length()
                 + str_datasize.length()
                 + str_data.length();
        } else if( str.find( "<DisparityMap>" ) != str.npos ) {
            ifs >> str_serialNumber 
                >> str_timeStamp 
                >> str_width
                >> str_height
                >> str_datasize 
                >> str_data;
            _disparity_size = str.length() 
                 + str_serialNumber.length()
                 + str_timeStamp.length()
                 + str_width.length()
                 + str_height.length()
                 + str_datasize.length()
                 + str_data.length();
        } else if( str.find( "<CameraImage>" ) != str.npos ) {
            ifs >> str_serialNumber
                >> str_timeStamp
                >> str_width
                >> str_height
                >> str_datasize
                >> str_data;            
            _camimage_size = str.length() 
                 + str_serialNumber.length()
                 + str_timeStamp.length()
                 + str_width.length()
                 + str_height.length()
                 + str_datasize.length()
                 + str_data.length();
        } else if( str.find( "<Geometry>" ) != str.npos ) {
            ifs >> str_serialNumber 
                >> str_timeStamp 
                >> str_width
                >> str_height
                >> str_datasize 
                >> str_data;
            _geometry_size = str.length() 
                 + str_serialNumber.length()
                 + str_timeStamp.length()
                 + str_width.length()
                 + str_height.length()
                 + str_datasize.length()
                 + str_data.length();
        } else if( str.find( "<Geometry2>" ) != str.npos ) {
            int size;
            ifs >> str_serialNumber 
                >> str_timeStamp 
                >> str_width
                >> str_height
                >> str_datasize 
                >> str_data;
            _geometry2_size = str.length() 
                 + str_serialNumber.length()
                 + str_timeStamp.length()
                 + str_width.length()
                 + str_height.length()
                 + str_datasize.length()
                 + str_data.length();
        }

        istringstream iss( str_timeStamp );
        iss >> timeStamp;

        if( flgFirst ) {
            time_start = timeStamp;
            time_until = timeStamp + 1000000ULL;
            total_size = pepmap_size = disparity_size = camimage_size = geometry_size = geometry2_size = 0;
            flgFirst = false;
        }

        if( timeStamp >= time_until ) {
            ofs << total_size << ", "
                << pepmap_size << ", "
                << disparity_size << ", "
                << camimage_size << ", "
                << geometry_size << ", "
                << geometry2_size << endl;
            total_size = pepmap_size = disparity_size = camimage_size = geometry_size = geometry2_size = 0;
            time_until += 1000000ULL;
        }

        total_size += _pepmap_size + _disparity_size + _camimage_size + _geometry_size + _geometry2_size;
        pepmap_size += _pepmap_size;
        disparity_size += _disparity_size;
        camimage_size += _camimage_size;
        geometry_size += _geometry_size;
        geometry2_size += _geometry2_size;
    }

    unsigned long long time_length = timeStamp - time_start;
    cout << "Done. Data length: " << time_length / 1000000 << "sec." << endl;
}