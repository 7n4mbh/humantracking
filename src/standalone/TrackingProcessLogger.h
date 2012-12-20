#ifndef __HEADER_TRACKINGPROCESSLOGGER__
#define __HEADER_TRACKINGPROCESSLOGGER__

#include <string>
#include "../humantracking.h"

class TrackingProcessLogger {
public:
    typedef enum { Start, End } Event;

private:
    unsigned long long t_start, t_end;
    unsigned long long t_start_of_tracking_block, t_end_of_tracking_block;
    unsigned long long t_process_of_making_trajectory, t_start_of_making_trajectory, t_end_of_making_trajectory;
    unsigned long long t_process_of_clustering, t_start_of_clustering, t_end_of_clustering;
    unsigned long long t_process_of_renovation, t_start_of_renovation, t_end_of_renovation;
    unsigned long long t_process_of_finishing, t_start_of_finishing, t_end_of_finishing;
    unsigned long long t_process_of_finishing_a, t_start_of_finishing_a, t_end_of_finishing_a;
    unsigned long long t_process_of_finishing_b, t_start_of_finishing_b, t_end_of_finishing_b;
    unsigned long long t_process_of_finishing_c, t_start_of_finishing_c, t_end_of_finishing_c;
    unsigned long long t_process_of_finishing_d, t_start_of_finishing_d, t_end_of_finishing_d;
    std::string strFilename;

public:
    TrackingProcessLogger();
    virtual ~TrackingProcessLogger();

    void init( const std::string& filename );
    void start();
    void end_and_output2file();
    void set_tracking_block( unsigned long long start, unsigned long long end );

    void receive_pepmap( const PEPMapInfo& pepmap );
    void making_trajectory( TrackingProcessLogger::Event evt );
    void clustering( TrackingProcessLogger::Event evt );
    void renovation( TrackingProcessLogger::Event evt );
    void finishing( TrackingProcessLogger::Event evt );
    void finishing_a( TrackingProcessLogger::Event evt );
    void finishing_b( TrackingProcessLogger::Event evt );
    void finishing_c( TrackingProcessLogger::Event evt );
    void finishing_d( TrackingProcessLogger::Event evt );
};

#endif