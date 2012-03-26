#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <map>

using namespace std;

string strPEPMapFilePath;

typedef struct {
    unsigned int serialNumber;
    unsigned long long timeStamp;
    string data;
} PEPMapInfo;

int main( int argc, char *argv[] )
{
    map<unsigned long long,PEPMapInfo> pepmap;
    vector<unsigned int> serialNumber;
    string str;

    for( int i = 1; i < argc; ++i ) {
        string strOpt = argv[ i ];
        if( strOpt == "-d" ) {
            strPEPMapFilePath = string( argv[ ++i ] );
        } else {
            serialNumber.push_back( atoi( argv[ i ]) );
        }
    }

    for( int i = 0; i < serialNumber.size(); ++i ) {
        ostringstream oss;
        oss << strPEPMapFilePath << "pepmap" << serialNumber[ i ] << ".dat";
        ifstream ifs( oss.str() );
        if( !ifs.is_open() ) {
            cerr << oss.str() << "could not be opened." << endl;
            exit( 1 );
        }

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

    cout << "Done." << endl;

    return 0;
}