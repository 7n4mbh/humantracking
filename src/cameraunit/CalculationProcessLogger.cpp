#include <fstream>

#include "../humantracking.h"
#include "CalculationProcessLogger.h"

using namespace std;

CalculationProcessLogger::CalculationProcessLogger()
{
}

CalculationProcessLogger::~CalculationProcessLogger()
{
}

void CalculationProcessLogger::init( const std::string& filename )
{
    strFilename = filename;
    ofstream ofs( strFilename.c_str() );
    ofs << "Time Stamp"
        << ", Process Time[usec]"
        << ", Process Time of Stereo Processing"
        << ", Process Time of Making Maps"
        << ", Process Time of Sending an Occupancy Map"
        << ", Process Time of Sending a Camera image"
        << ", Process Time of Sending a Geometry Map"
        << ", Process Time of Sending a Geometry Map2"
        << endl;
}

void CalculationProcessLogger::start()
{
    t_process_of_stereo_processing = t_start_of_stereo_processing = t_end_of_stereo_processing = 0;
    t_process_of_makingmaps = t_start_of_makingmaps = t_end_of_makingmaps = 0;
    t_process_of_send_occupancy = t_start_of_send_occupancy = t_end_of_send_occupancy = 0;
    t_process_of_send_camimage = t_start_of_send_camimage = t_end_of_send_camimage = 0;
    t_process_of_send_geometry = t_start_of_send_geometry = t_end_of_send_geometry = 0;
    t_process_of_send_geometry2 = t_start_of_send_geometry2 = t_end_of_send_geometry2 = 0;

    t_start = getTimeStamp();
}

void CalculationProcessLogger::end_and_output2file()
{
    t_end = getTimeStamp();
    const unsigned long long t_process =  t_end - t_start;

    ofstream ofs( strFilename.c_str(), ios::out | ios::app );

    ofs << timeStamp << ", "
        << t_process << ", "
        << t_process_of_stereo_processing << ", "
        << t_process_of_makingmaps << ", "
        << t_process_of_send_occupancy << ", "
        << t_process_of_send_camimage << ", "
        << t_process_of_send_geometry << ", "
        << t_process_of_send_geometry2 << endl;
}

void CalculationProcessLogger::set_timestamp( unsigned long long timestamp )
{
    timeStamp = timestamp;
}

void CalculationProcessLogger::stereo_processing( CalculationProcessLogger::Event evt )
{
    if( evt == CalculationProcessLogger::Start ) {
        t_start_of_stereo_processing = getTimeStamp();
    } else if( evt == CalculationProcessLogger::End ) {
        t_end_of_stereo_processing = getTimeStamp();
        t_process_of_stereo_processing += t_end_of_stereo_processing - t_start_of_stereo_processing;
    }
}

void CalculationProcessLogger::makingmaps( CalculationProcessLogger::Event evt )
{
    if( evt == CalculationProcessLogger::Start ) {
        t_start_of_makingmaps = getTimeStamp();
    } else if( evt == CalculationProcessLogger::End ) {
        t_end_of_makingmaps = getTimeStamp();
        t_process_of_makingmaps += t_end_of_makingmaps - t_start_of_makingmaps;
    }
}

void CalculationProcessLogger::send_occupancy( CalculationProcessLogger::Event evt )
{
    if( evt == CalculationProcessLogger::Start ) {
        t_start_of_send_occupancy = getTimeStamp();
    } else if( evt == CalculationProcessLogger::End ) {
        t_end_of_send_occupancy = getTimeStamp();
        t_process_of_send_occupancy += t_end_of_send_occupancy - t_start_of_send_occupancy;
    }
}

void CalculationProcessLogger::send_camimage( CalculationProcessLogger::Event evt )
{
    if( evt == CalculationProcessLogger::Start ) {
        t_start_of_send_camimage = getTimeStamp();
    } else if( evt == CalculationProcessLogger::End ) {
        t_end_of_send_camimage = getTimeStamp();
        t_process_of_send_camimage += t_end_of_send_camimage - t_start_of_send_camimage;
    }
}

void CalculationProcessLogger::send_geometry( CalculationProcessLogger::Event evt )
{
    if( evt == CalculationProcessLogger::Start ) {
        t_start_of_send_geometry = getTimeStamp();
    } else if( evt == CalculationProcessLogger::End ) {
        t_end_of_send_geometry = getTimeStamp();
        t_process_of_send_geometry += t_end_of_send_geometry - t_start_of_send_geometry;
    }
}

void CalculationProcessLogger::send_geometry2( CalculationProcessLogger::Event evt )
{
    if( evt == CalculationProcessLogger::Start ) {
        t_start_of_send_geometry2 = getTimeStamp();
    } else if( evt == CalculationProcessLogger::End ) {
        t_end_of_send_geometry2 = getTimeStamp();
        t_process_of_send_geometry2 += t_end_of_send_geometry2 - t_start_of_send_geometry2;
    }
}
