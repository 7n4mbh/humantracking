#ifndef __HEADER_CALCULATIONPROCESSLOGGER__
#define __HEADER_CALCULATIONPROCESSLOGGER__

#include <string>
#include "../humantracking.h"

class CalculationProcessLogger {
public:
    typedef enum { Start, End } Event;

private:
    unsigned long long t_start, t_end;
    unsigned long long timeStamp;
    unsigned long long t_process_of_stereo_processing, t_start_of_stereo_processing, t_end_of_stereo_processing;
    unsigned long long t_process_of_makingmaps, t_start_of_makingmaps, t_end_of_makingmaps;
    unsigned long long t_process_of_send_occupancy, t_start_of_send_occupancy, t_end_of_send_occupancy;
    unsigned long long t_process_of_send_camimage, t_start_of_send_camimage, t_end_of_send_camimage;
    unsigned long long t_process_of_send_geometry, t_start_of_send_geometry, t_end_of_send_geometry;
    unsigned long long t_process_of_send_geometry2, t_start_of_send_geometry2, t_end_of_send_geometry2;
    std::string strFilename;

public:
    CalculationProcessLogger();
    virtual ~CalculationProcessLogger();

    void init( const std::string& filename );
    void start();
    void end_and_output2file();
    void set_timestamp( unsigned long long timestamp );
    void stereo_processing( CalculationProcessLogger::Event evt );
    void makingmaps( CalculationProcessLogger::Event evt );
    void send_occupancy( CalculationProcessLogger::Event evt );
    void send_camimage( CalculationProcessLogger::Event evt );
    void send_geometry( CalculationProcessLogger::Event evt );
    void send_geometry2( CalculationProcessLogger::Event evt );
};

#endif