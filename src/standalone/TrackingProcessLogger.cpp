#include <fstream>

#include "../humantracking.h"
#include "TrackingProcessLogger.h"

using namespace std;

TrackingProcessLogger::TrackingProcessLogger()
{
}

TrackingProcessLogger::~TrackingProcessLogger()
{
}

void TrackingProcessLogger::init( const std::string& filename )
{
    strFilename = filename;
    ofstream ofs( strFilename.c_str() );
    ofs << "Start Time of Tracking Block"
        << ", End Time of Tracking Block"
        << ", Process Time[usec]"
        << ", Process Time of Making Trajectories[usec]"
        << ", Process Time of Clustering[usec]"
        << ", Process Time of Clustering (Hash Map) [usec]"
        << ", Process Time of Clustering (CCL) [usec]"
        << ", Process Time of Clustering (Finishing) [usec]"
        << ", Process Time of Renovation[usec]"
        << ", Process Timee of Renovation (Hash Map) [usec]"
        << ", Process Time of Renovation (Score) [usec]"
        << ", Finishing Process Time[usec]"
        << ", Percentage of Making Trajectories"
        << ", Percentage of Clustering"
        << ", Percentage of Renovation"
        << ", Percentage of Finishing"
        << ", Percentage of Finishing_A"
        << ", Percentage of Finishing_B"
        << ", Percentage of Finishing_C"
        << ", Percentage of Finishing_D"
        << endl;
}

void TrackingProcessLogger::start()
{
    t_process_of_making_trajectory = t_start_of_making_trajectory = t_end_of_making_trajectory = 0;
    t_process_of_clustering = t_start_of_clustering = t_end_of_clustering = 0;
    t_process_of_clustering_hashmap = t_start_of_clustering_hashmap = t_end_of_clustering_hashmap = 0;
    t_process_of_clustering_ccl = t_start_of_clustering_ccl = t_end_of_clustering_ccl = 0;
    t_process_of_clustering_finishing = t_start_of_clustering_finishing = t_end_of_clustering_finishing = 0;
    t_process_of_renovation = t_start_of_renovation = t_end_of_renovation = 0;
    t_process_of_renovation_hashmap = t_start_of_renovation_hashmap = t_end_of_renovation_hashmap = 0;
    t_process_of_renovation_score = t_start_of_renovation_score = t_end_of_renovation_score = 0;
    t_process_of_finishing = t_start_of_finishing = t_end_of_finishing = 0;
    t_process_of_finishing_a = t_start_of_finishing_a = t_end_of_finishing_a = 0;
    t_process_of_finishing_b = t_start_of_finishing_b = t_end_of_finishing_b = 0;
    t_process_of_finishing_c = t_start_of_finishing_c = t_end_of_finishing_c = 0;
    t_process_of_finishing_d = t_start_of_finishing_d = t_end_of_finishing_d = 0;
    t_start = getTimeStamp();
}

void TrackingProcessLogger::end_and_output2file()
{
    t_end = getTimeStamp();
    const unsigned long long t_process =  t_end - t_start;

    ofstream ofs( strFilename.c_str(), ios::out | ios::app );

    ofs << t_start_of_tracking_block << ", "
        << t_end_of_tracking_block << ", "
        << t_process << ", "
        << t_process_of_making_trajectory << ", "
        << t_process_of_clustering << ", "
        << t_process_of_clustering_hashmap << ", "
        << t_process_of_clustering_ccl << ", "
        << t_process_of_clustering_finishing << ", "
        << t_process_of_renovation << ", "
        << t_process_of_renovation_hashmap << ", "
        << t_process_of_renovation_score << ", "
        << t_process_of_finishing << ", "
        << t_process_of_finishing_a << ", "
        << t_process_of_finishing_b << ", "
        << t_process_of_finishing_c << ", "
        << t_process_of_finishing_d << ", "
        << 100.0 * (double)t_process_of_making_trajectory / (double)t_process << ", "
        << 100.0 * (double)t_process_of_clustering / (double)t_process << ", "
        << 100.0 * (double)t_process_of_renovation / (double)t_process << ", "
        << 100.0 * (double)t_process_of_finishing / (double)t_process << endl;
}

void TrackingProcessLogger::set_tracking_block( unsigned long long start, unsigned long long end )
{
    t_start_of_tracking_block = start;
    t_end_of_tracking_block = end;
}

void TrackingProcessLogger::receive_pepmap( const PEPMapInfo& pepmap )
{
}

void TrackingProcessLogger::making_trajectory( TrackingProcessLogger::Event evt )
{
    if( evt == TrackingProcessLogger::Start ) {
        t_start_of_making_trajectory = getTimeStamp();
    } else if( evt == TrackingProcessLogger::End ) {
        t_end_of_making_trajectory = getTimeStamp();
        t_process_of_making_trajectory += t_end_of_making_trajectory - t_start_of_making_trajectory;
    }
}

void TrackingProcessLogger::clustering( TrackingProcessLogger::Event evt )
{
    if( evt == TrackingProcessLogger::Start ) {
        t_start_of_clustering = getTimeStamp();
    } else if( evt == TrackingProcessLogger::End ) {
        t_end_of_clustering = getTimeStamp();
        t_process_of_clustering += t_end_of_clustering - t_start_of_clustering;
    }
}

void TrackingProcessLogger::clustering_hashmap( TrackingProcessLogger::Event evt )
{
    if( evt == TrackingProcessLogger::Start ) {
        t_start_of_clustering_hashmap = getTimeStamp();
    } else if( evt == TrackingProcessLogger::End ) {
        t_end_of_clustering_hashmap = getTimeStamp();
        t_process_of_clustering_hashmap += t_end_of_clustering_hashmap - t_start_of_clustering_hashmap;
    }
}

void TrackingProcessLogger::clustering_ccl( TrackingProcessLogger::Event evt )
{
    if( evt == TrackingProcessLogger::Start ) {
        t_start_of_clustering_ccl = getTimeStamp();
    } else if( evt == TrackingProcessLogger::End ) {
        t_end_of_clustering_ccl = getTimeStamp();
        t_process_of_clustering_ccl += t_end_of_clustering_ccl - t_start_of_clustering_ccl;
    }
}

void TrackingProcessLogger::clustering_finishing( TrackingProcessLogger::Event evt )
{
    if( evt == TrackingProcessLogger::Start ) {
        t_start_of_clustering_finishing = getTimeStamp();
    } else if( evt == TrackingProcessLogger::End ) {
        t_end_of_clustering_finishing = getTimeStamp();
        t_process_of_clustering_finishing += t_end_of_clustering_finishing - t_start_of_clustering_finishing;
    }
}

void TrackingProcessLogger::renovation( TrackingProcessLogger::Event evt )
{
    if( evt == TrackingProcessLogger::Start ) {
        t_start_of_renovation = getTimeStamp();
    } else if( evt == TrackingProcessLogger::End ) {
        t_end_of_renovation = getTimeStamp();
        t_process_of_renovation += t_end_of_renovation - t_start_of_renovation;
    }
}

void TrackingProcessLogger::renovation_hashmap( TrackingProcessLogger::Event evt )
{
    if( evt == TrackingProcessLogger::Start ) {
        t_start_of_renovation_hashmap = getTimeStamp();
    } else if( evt == TrackingProcessLogger::End ) {
        t_end_of_renovation_hashmap = getTimeStamp();
        t_process_of_renovation_hashmap += t_end_of_renovation_hashmap - t_start_of_renovation_hashmap;
    }
}

void TrackingProcessLogger::renovation_score( TrackingProcessLogger::Event evt )
{
    if( evt == TrackingProcessLogger::Start ) {
        t_start_of_renovation_score = getTimeStamp();
    } else if( evt == TrackingProcessLogger::End ) {
        t_end_of_renovation_score = getTimeStamp();
        t_process_of_renovation_score += t_end_of_renovation_score - t_start_of_renovation_score;
    }
}

void TrackingProcessLogger::finishing( TrackingProcessLogger::Event evt )
{
    if( evt == TrackingProcessLogger::Start ) {
        t_start_of_finishing = getTimeStamp();
    } else if( evt == TrackingProcessLogger::End ) {
        t_end_of_finishing = getTimeStamp();
        t_process_of_finishing += t_end_of_finishing - t_start_of_finishing;
    }
}

void TrackingProcessLogger::finishing_a( TrackingProcessLogger::Event evt )
{
    if( evt == TrackingProcessLogger::Start ) {
        t_start_of_finishing_a = getTimeStamp();
    } else if( evt == TrackingProcessLogger::End ) {
        t_end_of_finishing_a = getTimeStamp();
        t_process_of_finishing_a += t_end_of_finishing_a - t_start_of_finishing_a;
    }
}

void TrackingProcessLogger::finishing_b( TrackingProcessLogger::Event evt )
{
    if( evt == TrackingProcessLogger::Start ) {
        t_start_of_finishing_b = getTimeStamp();
    } else if( evt == TrackingProcessLogger::End ) {
        t_end_of_finishing_b = getTimeStamp();
        t_process_of_finishing_b += t_end_of_finishing_b - t_start_of_finishing_b;
    }
}

void TrackingProcessLogger::finishing_c( TrackingProcessLogger::Event evt )
{
    if( evt == TrackingProcessLogger::Start ) {
        t_start_of_finishing_c = getTimeStamp();
    } else if( evt == TrackingProcessLogger::End ) {
        t_end_of_finishing_c = getTimeStamp();
        t_process_of_finishing_c += t_end_of_finishing_c - t_start_of_finishing_c;
    }
}

void TrackingProcessLogger::finishing_d( TrackingProcessLogger::Event evt )
{
    if( evt == TrackingProcessLogger::Start ) {
        t_start_of_finishing_d = getTimeStamp();
    } else if( evt == TrackingProcessLogger::End ) {
        t_end_of_finishing_d = getTimeStamp();
        t_process_of_finishing_d += t_end_of_finishing_d - t_start_of_finishing_d;
    }
}


