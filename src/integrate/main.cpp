#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <map>

#include "../humantracking.h"

using namespace std;

string strPEPMapFilePath;

int main( int argc, char *argv[] )
{
    //map<unsigned long long,PEPMapInfo> pepmap;
    vector<unsigned int> serialNumber;
    vector<ifstream> ifs;
    vector<PEPMapInfo> pepmap;
    vector<CameraImageInfo> cam_image;
    vector<GeometryMapInfo> geometry;
    string str;
    unsigned long long time_start = 0;

    for( int i = 1; i < argc; ++i ) {
        string strOpt = argv[ i ];
        if( strOpt == "-d" ) {
            strPEPMapFilePath = string( argv[ ++i ] );
        } else if( strOpt == "-t" ) {
            istringstream iss( string( argv[ ++i ] ).c_str() );
            iss >> time_start;
        } else {
            serialNumber.push_back( atoi( argv[ i ]) );
        }
    }

    pepmap.resize( serialNumber.size() );
    cam_image.resize( serialNumber.size() );
    geometry.resize( serialNumber.size() );
    ifs.resize( serialNumber.size() );

    for( int i = 0; i < serialNumber.size(); ++i ) {
        ostringstream oss;
        oss << strPEPMapFilePath << "pepmap" << serialNumber[ i ] << ".dat";
        ifs[ i ].open( oss.str() );
        if( !ifs[ i ].is_open() ) {
            cerr << oss.str() << "could not be opened." << endl;
            exit( 1 );
        }
    }

    vector<unsigned long long> time;
    for( int i = 0; i < serialNumber.size(); ++i ) {
        while( !ifs[ i ].eof() ) {
            ifs[ i ] >> str;
		    if( str.find( "<PEPMap>" ) != str.npos ) {
                int size;
                ifs[ i ] >> pepmap[ i ].serialNumber >> pepmap[ i ].timeStamp >> size >> pepmap[ i ].data;
                if( /*pepmap[ i ].serialNumber != 7420008 ||*/ pepmap[ i ].timeStamp >= time_start/*1236097834238000ULL*/ ) {
                    break;
                }
            }/* else if( str.find( "<CameraImage>" ) != str.npos ) {
                int size;
                ifs[ i ] >> cam_image[ i ].serialNumber
                         >> cam_image[ i ].timeStamp
                         >> cam_image[ i ].width
                         >> cam_image[ i ].height
                         >> size
                         >> cam_image[ i ].data;            
            } else if( str.find( "<Geometry>" ) != str.npos ) {
                int size;
                ifs[ i ] >> geometry[ i ].serialNumber >> geometry[ i ].timeStamp >> size >> geometry[ i ].data;
            }
            break;*/
        }
    }

    ostringstream oss;
    oss << strPEPMapFilePath << "pepmap.dat";
    ofstream ofs( oss.str() );
    if( !ofs.is_open() ) {
        cerr << oss.str() << "could not be opened." << endl;
        exit( 1 );
    }    

    bool flgLoop, flgFirst = true;
    PEPMapInfo prev_pepmap;
    do {
        int idx_min = 0;
        for( int i = 1; i < pepmap.size(); ++i ) {
            if( ifs[ idx_min ].eof() || ( pepmap[ idx_min ].timeStamp > pepmap[ i ].timeStamp ) ) {
                idx_min = i;
            }
        }

        if( !flgFirst && pepmap[ idx_min ].serialNumber == prev_pepmap.serialNumber && pepmap[ idx_min ].timeStamp == prev_pepmap.timeStamp ) {
            break;
        } else {
            prev_pepmap = pepmap[ idx_min ];
            flgFirst = false;
        }

        ofs << "<PEPMap>" << endl
            << pepmap[ idx_min ].serialNumber << endl
            << pepmap[ idx_min ].timeStamp << endl
            << pepmap[ idx_min ].data.size() / 2 << endl
            << pepmap[ idx_min ].data << endl;

        while( !ifs[ idx_min ].eof() ) {
            ifs[ idx_min ] >> str;
		    if( str.find( "<PEPMap>" ) != str.npos ) {
                int size;
                ifs[ idx_min ] >> pepmap[ idx_min ].serialNumber >> pepmap[ idx_min ].timeStamp >> size >> pepmap[ idx_min ].data;
                break;
            } else if( str.find( "<CameraImage>" ) != str.npos ) {
                int size;
                ifs[ idx_min ] >> cam_image[ idx_min ].serialNumber
                         >> cam_image[ idx_min ].timeStamp
                         >> cam_image[ idx_min ].width
                         >> cam_image[ idx_min ].height
                         >> size
                         >> cam_image[ idx_min ].data;            
                ofs << "<CameraImage>" << endl
                    << cam_image[ idx_min ].serialNumber << endl
                    << cam_image[ idx_min ].timeStamp << endl
                    << cam_image[ idx_min ].width << endl
                    << cam_image[ idx_min ].height << endl
                    << size << endl
                    << cam_image[ idx_min ].data << endl;   
            } else if( str.find( "<Geometry>" ) != str.npos ) {
                int size;
                ifs[ idx_min ] >> geometry[ idx_min ].serialNumber 
                               >> geometry[ idx_min ].timeStamp 
                               >> geometry[ idx_min ].width
                               >> geometry[ idx_min ].height
                               >> size 
                               >> geometry[ idx_min ].data;
                ofs << "<Geometry>" << endl
                    << geometry[ idx_min ].serialNumber << endl
                    << geometry[ idx_min ].timeStamp << endl
                    << geometry[ idx_min ].width << endl
                    << geometry[ idx_min ].height << endl
                    << size << endl
                    << geometry[ idx_min ].data << endl;
            } else if( str.find( "<Geometry2>" ) != str.npos ) {
                int size;
                ifs[ idx_min ] >> geometry[ idx_min ].serialNumber 
                               >> geometry[ idx_min ].timeStamp 
                               >> geometry[ idx_min ].width
                               >> geometry[ idx_min ].height
                               >> size 
                               >> geometry[ idx_min ].data;
                ofs << "<Geometry2>" << endl
                    << geometry[ idx_min ].serialNumber << endl
                    << geometry[ idx_min ].timeStamp << endl
                    << geometry[ idx_min ].width << endl
                    << geometry[ idx_min ].height << endl
                    << size << endl
                    << geometry[ idx_min ].data << endl;
            }
        }

        flgLoop = false;
        for( int i = 0; i < ifs.size(); ++i ) {
            cout << "    " << serialNumber[ i ];
            if( !ifs[ i ].eof() ) {
                cout << ":Data Available";
            } else {
                cout << ":No Data Available";
            }
            flgLoop = flgLoop || !ifs[ i ].eof();
        }
        cout << endl;
    } while( flgLoop );
/*


        size_t nPEPMaps = 0;
        while( !ifs.eof() ) {
            ifs >> str;
		    if( str.find( "<PEPMap>" ) != str.npos ) {
                PEPMapInfo _pepmap;
                int size;
                ifs >> _pepmap.serialNumber >> _pepmap.timeStamp >> size >> _pepmap.data;
                pepmap[ _pepmap.timeStamp ] = _pepmap;
                ++nPEPMaps;
            }
        }

        cout << nPEPMaps << " PEP-map(s) loaded from pepmap" << serialNumber[ i ] << ".dat" << endl;
    }


    ostringstream oss;
    oss << strPEPMapFilePath << "pepmap.dat";
    ofstream ofs( oss.str() );
    if( !ofs.is_open() ) {
        cerr << oss.str() << "could not be opened." << endl;
        exit( 1 );
    }    
    for( map<unsigned long long,PEPMapInfo>::iterator it = pepmap.begin(); it != pepmap.end(); ++it ) {
        ofs << "<PEPMap>" << endl;
        ofs << it->second.serialNumber << endl;
        ofs << it->second.timeStamp << endl;
        ofs << it->second.data.size() / 2 << endl;
        ofs << it->second.data << endl;
    }
*/
    cout << "Done." << endl;

    return 0;
}