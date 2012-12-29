#include <numeric>
#include <stdlib.h>

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "../humantracking.h"
#include "track.h"
#include "TrackingProcessLogger.h"
//#include "Viewer.h"

using namespace std;
using namespace cv;

TrackingProcessLogger logTracking;
//extern Viewer viewer;

PARAM_COMMON commonParam; // ã§í ÉpÉâÉÅÅ[É^
PARAM_EXTRACTLUM extractlumParam; // ìôë¨íºê¸â^ìÆíäèoÉpÉâÉÅÅ[É^
PARAM_MKTRAJECTORY mktrajectoryParam; // ãOê’çÏê¨ÉpÉâÉÅÅ[É^
PARAM_CLUSTERING clusteringParam; // ÉNÉâÉXÉ^ÉäÉìÉOÉpÉâÉÅÅ[É^
//PARAM_MAKERESULT makeresultParam; // í«ê’åãâ çÏê¨ÉpÉâÉÅÅ[É^
PARAM_RENOVATE_TRAJECTORY rnvtrjParam; // ãOê’èCïúÉpÉâÉÅÅ[É^
PARAM_PLOT plotParam; // åvéZâﬂíˆÉvÉçÉbÉgÉpÉâÉÅÅ[É^

bool flgFirst;

extern float roi_width, roi_height;
extern float roi_x, roi_y;
extern float scale_m2px, scale_m2px_silhouette;

extern bool flgOutputTrackingProcessData2Files;

double dist_trajectory_element( TrajectoryElement& trj1, TrajectoryElement& trj2 )
{
    TIME_MICRO_SEC timeBegin; // ãóó£Çï]âøÇ∑ÇÈäJénéûä‘
    TIME_MICRO_SEC timeEnd; // ãóó£Çï]âøÇ∑ÇÈèIóπéûä‘

    // ã§í Ç∑ÇÈéûä‘Åiãóó£Çï]âøÇ∑ÇÈéûä‘îÕàÕÅjÇí≤Ç◊ÇÈ
    timeBegin = max( trj1.begin()->t, trj2.begin()->t );
    timeEnd = min( trj1.rbegin()->t, trj2.rbegin()->t );

    // ã§í éûä‘Ç™Ç»Ç¢èÍçáÇÕñ≥å¿ëÂÇï‘Ç∑
    if( timeBegin > timeEnd ) {
        return -0.3;
    }

    // ã§í éûä‘Ç™minCommonTimeRange[usec]ñ¢ñûÇÃèÍçáÇÕñ≥å¿ëÂÇï‘Ç∑
    if( ( timeEnd - timeBegin ) < clusteringParam.minCommonTimeRange ) {
        return -0.4;
    }

//    // ÉeÉXÉg
//    // ã§í éûä‘Ç™í∑Ç¢ï˚ÇÃãOê’ÇÃ8äÑñ¢ñûÇÃèÍçáÇÕñ≥å¿ëÂÇï‘Ç∑
//    TIME_MICRO_SEC timeLenMax = max( trj1.rbegin()->t - trj1.begin()->t, trj2.rbegin()->t - trj2.begin()->t );
//    if( ( timeEnd - timeBegin ) < ( 50 * timeLenMax ) / 100 ) {
//        return -0.4;
//    }
/*
    if( flgUseID ) {
        // trj1ÇÃIDÇãÅÇﬂÇÈ
        int id1 = 0;
        TrajectoryElement::iterator itPos;
        for( itPos = trj1.begin(); itPos != trj1.end() && id1 == 0; ++itPos ) {
            id1 = itPos->ID;
        }

        // ob1ÇÃIDÇãÅÇﬂÇÈ
        int id2 = 0;
        for( itPos = trj2.begin(); itPos != trj2.end() && id2 == 0; ++itPos ) {
            id2 = itPos->ID;
        }

        // trj1Ç∆trj2ÇÃIDÇ™Ç¢Ç∏ÇÍÇ‡0à»äOÇ≈ÅCÇ©Ç¬àÍívÇµÇ»Ç¢èÍçáÇÕñ≥å¿ëÂÇï‘Ç∑
        if( ( id1 != 0 ) && ( id2 != 0 ) && ( id1 != id2 ) ) {
            return -1.0;
        }
    }
*/
    // ã§í Ç∑ÇÈëSÇƒÇÃéûçèÇ≈IDÇ™àÍívÇ∑ÇÈÇ©Ç«Ç§Ç©åüèÿ
    TrajectoryElement::iterator it1, it2;
    it1 = trj1.find( PosXYT( 0.0, 0.0, timeBegin ) );
    it2 = trj2.find( PosXYT( 0.0, 0.0, timeBegin ) );
    for( ; it1 != trj1.end() && it1->t <= timeEnd && it2 != trj2.end() && it2->t <= timeEnd; ++it1, ++it2 ) {
        if( it1->ID != it2->ID ) {
            return -1.0;
        }
    }

    return 0.0;
}

double areIndependent( TrajectoryElement& trj1, TrajectoryElement& trj2, double threshold )
{
    TIME_MICRO_SEC timeBegin; // ãóó£Çï]âøÇ∑ÇÈäJénéûä‘
    TIME_MICRO_SEC timeEnd; // ãóó£Çï]âøÇ∑ÇÈèIóπéûä‘

    // ã§í Ç∑ÇÈéûä‘Åiãóó£Çï]âøÇ∑ÇÈéûä‘îÕàÕÅjÇí≤Ç◊ÇÈ
    timeBegin = max( trj1.begin()->t, trj2.begin()->t );
    timeEnd = min( trj1.rbegin()->t, trj2.rbegin()->t );

    // ã§í éûä‘Ç™Ç»Ç¢èÍçáÇÕ'ì∆óß'Çï‘Ç∑
    if( timeBegin > timeEnd ) {
        return 0.0;
    }

    // ã§í éûä‘Ç™minCommonTimeRange[usec]ñ¢ñûÇÃèÍçáÇÕ'ì∆óß'Çï‘Ç∑
    if( ( timeEnd - timeBegin ) < clusteringParam.minCommonTimeRange ) {
        return 0.0;
    }

    // ã§í Ç∑ÇÈëSÇƒÇÃéûçèÇ≈ãóó£Ç™thresholdÇÊÇËëÂÇ´ÇØÇÍÇŒ'ì∆óß'Çï‘Ç∑ÅB
    // àÍìxÇ≈Ç‡thConnectÇÊÇËè¨Ç≥Ç≠Ç»ÇÈèÍçáÇÕ'ä±è¬Ç†ÇË'Ç∆îªífÅB
    TrajectoryElement::iterator it1, it2;
    it1 = trj1.find( PosXYT( 0.0, 0.0, timeBegin ) );
    it2 = trj2.find( PosXYT( 0.0, 0.0, timeBegin ) );
    for( ; it1 != trj1.end() && it1->t <= timeEnd && it2 != trj2.end() && it2->t <= timeEnd; ++it1, ++it2 ) {
        const double dx = it1->x - it2->x;
        const double dy = it1->y - it2->y;
        const double dist = sqrt( dx * dx + dy * dy );
        if( dist <= threshold ) {
            return -1.0;
        }
    }

    return 0.0;
}

double areConnectable( TrajectoryElement& trj1, TrajectoryElement& trj2, double threshold, unsigned long long termConnect )
{
    TIME_MICRO_SEC timeBegin; // ãóó£Çï]âøÇ∑ÇÈäJénéûä‘
    TIME_MICRO_SEC timeEnd; // ãóó£Çï]âøÇ∑ÇÈèIóπéûä‘

    // ã§í Ç∑ÇÈéûä‘Åiãóó£Çï]âøÇ∑ÇÈéûä‘îÕàÕÅjÇí≤Ç◊ÇÈ
    timeBegin = max( trj1.begin()->t, trj2.begin()->t );
    timeEnd = min( trj1.rbegin()->t, trj2.rbegin()->t );

    // ã§í éûä‘Ç™Ç†ÇÈèÍçáÇÕ'ê⁄ë±ïsâ¬î\'Çï‘Ç∑
    if( timeBegin <= timeEnd ) {
        return -0.5;
    }

    // termConnectà»è„ÇÃéûä‘í∑ó£ÇÍÇƒÇ¢ÇÈèÍçáÇÕ -1 Çï‘Ç∑
    //if( timeBegin - timeEnd >= termConnect ) {
    //    return -1.0;
    //}

    TrajectoryElement trjEarlier, trjLatter;
    if( trj1.rbegin()->t == timeEnd ) {
        trjEarlier = trj1;
        trjLatter = trj2;
    } else {
        trjEarlier = trj2;
        trjLatter = trj1;
    }

    // if trjEarlier and trjLatter can be connected at the speed less than 'threshold'
    // return 0. Return -1 otherwise.
    double dx = trjLatter.begin()->x - trjEarlier.rbegin()->x;
    double dy = trjLatter.begin()->y - trjEarlier.rbegin()->y;
    double dt = (double)( trjLatter.begin()->t - trjEarlier.rbegin()->t ) * 1.0e-6;
    double v = sqrt( dx * dx + dy * dy ) / dt;
    if( v > threshold ) {
        return -1.0;
    }

    return 0.0;
}

double fitting_score( vector<int>& combination, vector<CTrajectory>& trajectories, map<unsigned long long,set<int> >& time_to_hash_where_occupied, vector< map<unsigned long long,set<int> > >& trj_time_to_hash_where_occupied )
{
    map<unsigned long long,map<int,bool> > isOccupied;
    unsigned long total = 0;
    {
        map<unsigned long long,set<int> >::iterator itTimeToHash = time_to_hash_where_occupied.begin();
        for( ; itTimeToHash != time_to_hash_where_occupied.end(); ++itTimeToHash ) {
            for( set<int>::iterator itHash = itTimeToHash->second.begin(); itHash != itTimeToHash->second.end(); ++itHash ) {
                isOccupied[ itTimeToHash->first ][ *itHash ] = false;
                ++total;
            }
        }
    }

    for( int i = 0; i < combination.size(); ++i ) {
        map<unsigned long long,set<int> >::iterator itTimeToHash = trj_time_to_hash_where_occupied[ combination[ i ] ].begin();
        for( ; itTimeToHash != trj_time_to_hash_where_occupied[ combination[ i ] ].end(); ++itTimeToHash ) {
            for( set<int>::iterator itHash = itTimeToHash->second.begin(); itHash != itTimeToHash->second.end(); ++itHash ) {
                if( isOccupied[ itTimeToHash->first ].find( *itHash ) != isOccupied[ itTimeToHash->first ].end() ) {
                    isOccupied[ itTimeToHash->first ][ *itHash ] = true;
                }
            }
        }
    }

    map<unsigned long long,map<int,bool> >::iterator itTimeToMapHash = isOccupied.begin();
    unsigned long occupied = 0;
    for( ; itTimeToMapHash != isOccupied.end(); ++itTimeToMapHash ) {
        for( map<int,bool>::iterator itMapHash = itTimeToMapHash->second.begin(); itMapHash != itTimeToMapHash->second.end(); ++itMapHash ) {
            if( itMapHash->second ) {
                ++occupied;
            }
        }
    }

    return (double)occupied / (double)total;
}


double areCoherent( TrajectoryElement& trj1, TrajectoryElement& trj2, double threshold )
{
    TIME_MICRO_SEC timeBegin; // ãóó£Çï]âøÇ∑ÇÈäJénéûä‘
    TIME_MICRO_SEC timeEnd; // ãóó£Çï]âøÇ∑ÇÈèIóπéûä‘

    // ã§í Ç∑ÇÈéûä‘Åiãóó£Çï]âøÇ∑ÇÈéûä‘îÕàÕÅjÇí≤Ç◊ÇÈ
    timeBegin = max( trj1.begin()->t, trj2.begin()->t );
    timeEnd = min( trj1.rbegin()->t, trj2.rbegin()->t );

    // ã§í éûä‘Ç™Ç»Ç¢èÍçáÇÕñ≥å¿ëÂÇï‘Ç∑
    if( timeBegin > timeEnd ) {
        return -0.3;
    }

    // ã§í éûä‘Ç™minCommonTimeRange[usec]ñ¢ñûÇÃèÍçáÇÕñ≥å¿ëÂÇï‘Ç∑
    if( ( timeEnd - timeBegin ) < clusteringParam.minCommonTimeRange ) {
        return -0.4;
    }

    // ã§í Ç∑ÇÈëSÇƒÇÃéûçèÇ≈ãóó£ÇåvéZÇ™
    TrajectoryElement::iterator it1, it2;
    it1 = trj1.find( PosXYT( 0.0, 0.0, timeBegin ) );
    it2 = trj2.find( PosXYT( 0.0, 0.0, timeBegin ) );
    vector<double> distance;
    double distance_average = 0.0;
    for( ; it1 != trj1.end() && it1->t <= timeEnd && it2 != trj2.end() && it2->t <= timeEnd; ++it1, ++it2 ) {
        double dx = it1->x - it2->x;
        double dy = it1->y - it2->y;
        double dist = sqrt( dx * dx + dy * dy );
        distance.push_back( dist );
        distance_average += dist;
        //if( sqrt( dx * dx + dy * dy ) > threshold ) {
        //    return -1.0;
        //}
    }
    distance_average /= (double)distance.size();

    // äOÇÍílÇèúÇ≠Ç∑Ç◊ÇƒÇÃãóó£Ç™thresholdà»â∫Ç»ÇÁÇŒè\ï™Ç…ãﬂÇ¢Ç∆îªíf
    for( int i = 0 ; i < distance.size(); ++i ) {
        if( ( distance[ i ] - distance_average ) < 0.2 ) {
            if( distance[ i ] > threshold ) {
                return -1.0;
            }
        }
    } 

    return 0.0;
}

void _connect( vector< vector< vector<int> > >* pDst, vector< vector<int> > combination, int nCluster, int nElements, double* tableConnectable )
{
    bool flgEmpty = combination.empty();
    sort( combination.begin(), combination.end() );
    pDst->push_back( combination );

    for( int i = 0; i < combination.size(); ++i ) {
        for( int j = i + 1; j < combination.size(); ++j ) {
            // see if trajectories in combination[ i ] and combination[ j ] are connectable
            bool flgConnectable = false;
            bool flgInvalid = false;

            for( vector<int>::iterator itIdxTrj1 = combination[ i ].begin(); itIdxTrj1 != combination[ i ].end(); ++itIdxTrj1 ) {
                for( vector<int>::iterator itIdxTrj2 = combination[ j ].begin(); itIdxTrj2 != combination[ j ].end(); ++itIdxTrj2 ) {
                    int index = *itIdxTrj1 + *itIdxTrj2 * nElements;
                    if( tableConnectable[ index ] == 0.0 ) {
                        flgConnectable = true;
                    } else if( tableConnectable[ index ] == -0.5 ) {
                        flgInvalid = true;
                    }
                }
            }
            
            vector<int> trajectories;
            if( flgConnectable = ( flgConnectable && !flgInvalid ) ) {
                //flgConnectable = true;
                trajectories.insert( trajectories.end(), combination[ i ].begin(), combination[ i ].end() );
                trajectories.insert( trajectories.end(), combination[ j ].begin(), combination[ j ].end() );
		sort( trajectories.begin(), trajectories.end() );
            }

            //vector<int> trajectories;
            //trajectories.insert( trajectories.end(), combination[ i ].begin(), combination[ i ].end() );
            //trajectories.insert( trajectories.end(), combination[ j ].begin(), combination[ j ].end() );
            //sort( trajectories.begin(), trajectories.end() );
            //for( int iTrj1 = 0; !flgInvalid && iTrj1 < trajectories.size(); ++iTrj1 ) {
            //    for( int iTrj2 = 0; !flgInvalid && iTrj2 < trajectories.size(); ++iTrj2 ) {
            //        int index = trajectories[ iTrj1 ] + trajectories[ iTrj2 ] * nElements;
            //        if ( tableConnectable[ index ] == 0.0 ) {
            //            flgConnectable = true;
            //        }
            //        flgInvalid = ( tableConnectable[ index ] == -0.5 );
            //    }
            //}
            //if( flgInvalid ) {
            //    flgConnectable = false;
            //}

            // Integrate combination[ i ] and combination[ j ] if they are connectable 
            if( flgConnectable ) {
                vector< vector< int > > new_combination;
                new_combination.push_back( trajectories );
                for( int k = 0; k < combination.size(); ++k ) {
                    if( k != i && k != j ) {
                        new_combination.push_back( combination[ k ] );
                    }
                }
                _connect( pDst, new_combination, nCluster, nElements, tableConnectable );
            }
        }
    }
}

void connect( vector< vector< vector<int> > >* pDst, vector< vector<int> > init, vector<int>& combination, int nCluster, int nElements, double* tableConnectable )
{
    _connect( pDst, init, nCluster, nElements, tableConnectable );
    
    return;
}

bool load_track_parameters( std::string strPath, std::string strFileName )
{
    commonParam.termTracking = 10000000;//8500000;//10000000;
    commonParam.intervalTracking = 7000000;//7000000;
    commonParam.intervalTrajectory = 100000;
    extractlumParam.term = 1500000;
    extractlumParam.interval = 500000;
    extractlumParam.minPEPMapValue = 0.0;//50.0;
    extractlumParam.maxPEPMapValue = UINT_MAX;//70.0;//1500.0;
    extractlumParam.minDiffTime = 600000;
    extractlumParam.maxSpeed = 8.0;//10.44;
    extractlumParam.kLUM = 1.2e-1;//4.0e-1;//6.0e-1;//3.5e-1;//3.0e-2;
    extractlumParam.kVerifySample = 2.4e-1;//3.0e-1;//3.5e-1;//6.0e-2;
    extractlumParam.distVerifySample = 0.15;
    extractlumParam.thMean = 0.05;
    extractlumParam.thVariance = 0.4;
    extractlumParam.intervalVerify = 100000;
    extractlumParam.rangeVerifySample = 100000;
    extractlumParam.stDeviation = 0.05;
    mktrajectoryParam.distanceImpact = 0.07;
    mktrajectoryParam.densityOrigin = 400.0;//60.0;
    clusteringParam.thConnect = 0.4;//0.18;//0.2;
    clusteringParam.thDistance = 0.7;
    clusteringParam.minLength = 1000000;
    clusteringParam.distanceLimit = 0.1;//0.35;//clusteringParam.thDistance;
    clusteringParam.nLimit = 1;
    clusteringParam.minCommonTimeRange = 0;///250000;
    clusteringParam.distVerifyCluster = 0.28;
    rnvtrjParam.lambda1 = 10.0;
    rnvtrjParam.lambda2 = 400.0;//200.0;
    rnvtrjParam.lambda3 = 6.0;
    rnvtrjParam.territory = 0.12;//0.12;//0.07;
    rnvtrjParam.territoryVeryNear = 0.01;//0.07;//min( 0.1, rnvtrjParam.territory );;
    rnvtrjParam.termConnect = 3000000;
    //plotParam.rangeLeft = -2.0;
    //plotParam.rangeRight = 2.0;
    //plotParam.rangeTop = 2.0;//8.0;
    //plotParam.rangeBottom = -2.0;//1.0;
    plotParam.rangeLeft = roi_x - roi_width / 2.0;
    plotParam.rangeRight = roi_x + roi_width / 2.0;
    plotParam.rangeBottom = roi_y - roi_height / 2.0;
    plotParam.rangeTop = roi_y + roi_height / 2.0;
#if 0
    plotParam.kSample = 1.0e-2;//1.0e-3;
#else
    plotParam.kSample = 1.0e-3;//1.0e-3;
#endif


    ifstream ifs;

    ostringstream oss;
    oss << strPath << strFileName;//"tracking.cfg";
    ifs.open( oss.str().c_str() );

    if( !ifs.is_open() ) {
        return false;
    }

    cout << "Loading the tracking parameters..." << endl;

    string str;
    vector<string> strEq;
    while( !ifs.eof() ) {
        getline( ifs, str );
        
        char* cstrcmd = new char[ str.size() + 1 ];
        strcpy( cstrcmd, str.c_str() );
        
        strEq.clear();
        char* tp = strtok( cstrcmd, "=" );
        strEq.push_back( string( tp ) );
        while ( tp != NULL ) {
            tp = strtok( NULL, " " );
            if ( tp != NULL ) {
                strEq.push_back( string( tp ) );
            }
        }

        delete [] cstrcmd;

        if( strEq.size() == 2 ) {
            if( strEq[ 0 ] == "PARAM_EXTRACTLUM.kLUM" ) {
                extractlumParam.kLUM = atof( strEq[ 1 ].c_str() );
		cout << "  " << strEq[ 0 ] << "=" << extractlumParam.kLUM << endl;
            } else if( strEq[ 0 ] == "PARAM_EXTRACTLUM.kVerifySample" ) {
                extractlumParam.kVerifySample = atof( strEq[ 1 ].c_str() );
		cout << "  " << strEq[ 0 ] << "=" << extractlumParam.kVerifySample << endl;
            } else if( strEq[ 0 ] == "PARAM_COMMON.termTracking" ) {
                commonParam.termTracking = atof( strEq[ 1 ].c_str() );
		cout << "  " << strEq[ 0 ] << "=" << commonParam.termTracking << endl;
            } else if( strEq[ 0 ] == "PARAM_COMMON.intervalTracking" ) {
                commonParam.intervalTracking = atof( strEq[ 1 ].c_str() );
		cout << "  " << strEq[ 0 ] << "=" << commonParam.intervalTracking << endl;
            } 
        }
    }

    return true;
}

void initialize_tracker()
{
    flgFirst = true;
}

bool track( std::map< unsigned long long, std::map<int,cv::Point2d> >* p_result, std::map<unsigned long long, std::multimap<int,cv::Point2d> >* p_ext_result, const Mat& occupancy, unsigned long long time_stamp )
{
    static TIME_MICRO_SEC timeTracking; // $BDI@W;~9o(B[usec]
                                 // [timeTracking - commonParam.termTracking, timeTrackig) $B$NHO0O$GDI@W=hM}$r9T$&;v$r0UL#$9$k(B

    static SamplerPosXYTVID sampler; // $BFCD'NL%5%s%W%i(B

    // storageLUM
    // éûçètkÅiÉLÅ[ÅjÇ…ëŒâûÇ∑ÇÈLUMèWçáÇ÷ÇÃÉ}ÉbÉv
    // éûçètkÇ…ëŒâûÇ∑ÇÈìôë¨íºê¸â^ìÆèWçáÇ∆ÇÕÅA[tk - extractlumParam.term, tk)ÇÃPEPMapÇ©ÇÁçÏê¨ÇµÇΩìôë¨íºê¸â^ìÆèWçáÇÃÇ±Ç∆Ç≈Ç†ÇÈÅB
    static LUMStorage storageLUM;

    // tableLUMSlice
    // LUMÉXÉâÉCÉXÉeÅ[ÉuÉã
    // éûçètkÅiÉLÅ[ÅjÇ…Ç®ÇØÇÈLUMÉXÉâÉCÉXÇÃîzóÒÇ÷ÇÃÉ}ÉbÉv
    static LUMSliceTable tableLUMSlice;

    // storageTrajectory
    // çÏê¨íÜÇÃãOê’óvëf
    static vector<TrajectoryElement> storageTrajectoryElement;

    // resultTrajectory
    // í«ê’åãâ ÅiIDÇ∆ãOê’ÇÃÉ}ÉbÉvÅj
    static map<int,CTrajectory> resultTrajectory;

    //static vector<CTrajectory> prevTrajectoriesClustered;
    static map<unsigned long long, std::multimap<int,cv::Point2d> >  remainedExtendedResult;

    // idNext
    // í«ê’åãâ ÅiIDÇ∆ãOê’ÇÃÉ}ÉbÉvÅj
    static int idNext;

    static TIME_MICRO_SEC timeEarliestPEPMap;

    static bool flgTrackingStarts;

    static set<TIME_MICRO_SEC> time_of_received_pepmap;

    bool ret = false;

    if( flgFirst ) {
        sampler.clear();
        storageLUM.clear();
        tableLUMSlice.clear();
        storageTrajectoryElement.clear();
        resultTrajectory.clear();
        remainedExtendedResult.clear();
        idNext = 1;
        timeEarliestPEPMap = time_stamp;
        timeTracking = timeEarliestPEPMap + commonParam.termTracking;
        //timeTracking = commonParam.termTracking + 1000000000; // debug code
        flgFirst = false;
        flgTrackingStarts = true;
        logTracking.init( "tracking.log" );
        //viewer.SetStartTime( timeEarliestPEPMap );
        {
            ostringstream oss;
#ifdef WINDOWS_OS
		    oss << "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_trajectories\\";
#endif
#ifdef LINUX_OS
		    oss << "/home/fukushi/project/HumanTracking/bin/tmp_trajectories/";
#endif
            oss << "tracking.csv";
            ofstream ofs( oss.str().c_str(), std::ios::out );
            ofs << "time, # of clusters, # of combinations, # of connection patterns" << endl;
        }
    }

    // debug code
    //time_stamp = time_stamp - timeEarliestPEPMap + 1000000000;

    if( flgTrackingStarts ) {
        logTracking.set_tracking_block( timeTracking - commonParam.termTracking, timeTracking );
        logTracking.start();
        //viewer.SetTrackingStatus( 0 );
        //viewer.SetTrackingBlock( timeTracking - commonParam.termTracking, timeTracking );
        flgTrackingStarts = false;
    }
    
    time_of_received_pepmap.insert( time_stamp );

    logTracking.making_trajectory( TrackingProcessLogger::Start );

    // PEPMapÇÉTÉìÉvÉâÇ…í«â¡Ç∑ÇÈ
    AddPEPMapToSampler( occupancy
                      , time_stamp
                        , &sampler
                        , extractlumParam.minPEPMapValue
                        , extractlumParam.maxPEPMapValue );

    //
    // LUMíäèo
    vector<TIME_MICRO_SEC> addedTime;
    for( TIME_MICRO_SEC tk = timeTracking - commonParam.termTracking; tk <= time_stamp; tk += extractlumParam.interval ) {
        if( storageLUM.find( tk ) == storageLUM.end() ) {
            // éûçètkÇÃLUMÇíäèo
            ExtractLUM( &sampler
                        , tk
                        , &storageLUM[ tk ]
                        , &commonParam
                        , &extractlumParam
                        , /*true*/false );
            addedTime.push_back( tk );

        }
    }

    //
    //  LUMÉXÉâÉCÉXÉeÅ[ÉuÉãçÏê¨
    if( !storageLUM.empty() ) {
        TIME_MICRO_SEC timeLatestLUM = storageLUM.rbegin()->first;
        addedTime.clear();
        // timeLatestLUMÇ‹Ç≈ÇÃLUMÇ™ìæÇÁÇÍÇƒÇ¢ÇÈÇ∆Ç´ÅCLUMÉXÉâÉCÉXÉeÅ[ÉuÉãÇ™çÏê¨Ç≈Ç´ÇÈÇÃÇÕ
        // (timeLatestLUM - extractlumParam.term)Ç‹Ç≈ÅBÇ±ÇÃîÕàÕÇ≈LUMÉXÉâÉCÉXÉeÅ[ÉuÉãÇçÏê¨Ç∑ÇÈÅB
        for( TIME_MICRO_SEC tk = timeTracking - commonParam.termTracking
            ; tk <= timeLatestLUM - extractlumParam.term
            ; tk += commonParam.intervalTrajectory ) {
            if( tableLUMSlice.find( tk ) == tableLUMSlice.end() ) {
                // éûçètkÇÃLUMÉXÉâÉCÉXçÏê¨
                //cerr << "LUMÉXÉâÉCÉXí«â¡: ";
                MakeLUMSlice( tk, &storageLUM, &tableLUMSlice[ tk ], &extractlumParam );
                addedTime.push_back( tk );
            }
        }
    }

    // tableLUMSliceÇ…í«â¡Ç≥ÇÍÇΩäeéûçèÇ…énì_ÇíËÇﬂÇÈÅB
    set<PosXYT,PosXYT_XYT_Less> originPos;
    const double intersticeOrigin = 1.0 / sqrt( mktrajectoryParam.densityOrigin ); // $B50@W$N;OE@F1;N$N4V3V(B
    const double rangeOrigin = mktrajectoryParam.distanceImpact * 2.0; // $B50@W$N>pJs(BX,Y$B:BI8$N<~$j(BrangeOrigin$B;MJ}$NHO0O$K;OE@$N8uJd$r:n@.$9$k(B
    int nOrigin = 0;
    for( vector<TIME_MICRO_SEC>::iterator iTk = addedTime.begin(); iTk != addedTime.end(); ++iTk ) {
        TIME_MICRO_SEC tk = *iTk;
        vector<LUMSlice>::iterator itLUMSlice = tableLUMSlice[ tk ].begin();
        for( ; itLUMSlice != tableLUMSlice[ tk ].end(); ++itLUMSlice ) {
            for( double px = itLUMSlice->pos.x - rangeOrigin / 2.0; px < itLUMSlice->pos.x + rangeOrigin / 2.; px += intersticeOrigin ) {
                for( double py = itLUMSlice->pos.y - rangeOrigin / 2.0; py < itLUMSlice->pos.y + rangeOrigin / 2.0; py += intersticeOrigin ) {
                    originPos.insert( PosXYT( (int)( px / intersticeOrigin ) * intersticeOrigin
                                                , (int)( py / intersticeOrigin ) * intersticeOrigin
                                                , tk ) );
                }
            }
        }
        nOrigin = originPos.size();
    }

    // MPIÇ…ÇÊÇËénì_ÇäeÉvÉçÉZÉXÇ…äÑÇËêUÇÈ
    // à»â∫ÅCâºãLèq
    vector<PosXYT> originPosPerProcess;
    originPosPerProcess.assign( originPos.begin(), originPos.end() );

    //
    // éÛÇØéÊÇ¡ÇΩénì_ÇäÓÇ…êVÇΩÇ»ãOê’Çê∂ê¨Ç∑ÇÈ
    int nNewTrj = 0;
    vector<PosXYT>::iterator itOrigin = originPosPerProcess.begin();
    for( int i = 0; itOrigin != originPosPerProcess.end(); ++itOrigin, ++i ) {
        TrajectoryElement trjElement;
        trjElement.insert( *itOrigin );
        storageTrajectoryElement.push_back( trjElement );
        ++nNewTrj;
    }

    //
    // tableLUMSliceÇ…í«â¡Ç≥ÇÍÇΩäeéûçèÇ…Ç¬Ç¢ÇƒãOê’ÇâÑí∑Ç∑ÇÈ
    for( vector<TIME_MICRO_SEC>::iterator iTk = addedTime.begin(); iTk != addedTime.end(); ++iTk ) {
        TIME_MICRO_SEC tk = *iTk;
        if( tableLUMSlice.begin()->first != tk ) {
            ExtendTrajectories( tk - commonParam.intervalTrajectory
                                , tk
                                , &storageTrajectoryElement
                                , &tableLUMSlice[ tk - commonParam.intervalTrajectory ]
                                , &commonParam
                                , &mktrajectoryParam );
        }
    }

    logTracking.making_trajectory( TrackingProcessLogger::End );

    if( !tableLUMSlice.empty() && tableLUMSlice.rbegin()->first >= timeTracking ) {
        cout << "Done with making trajectories." << endl;

        if( 0/*flgOutputTrackingProcessData2Files*/ ) {
            // Output process infomation of making trajectories.
            double sumValue = std::accumulate( sampler.begin(), sampler.end(), 0.0, PosXYTV_Sum() );
            unsigned int nSample = (unsigned int)( plotParam.kSample /*3.0e-2*//*1.04e-4*/ * sumValue );
            OutputProcess( timeTracking - commonParam.termTracking//tableLUMSlice.begin()->first
                            , timeTracking//tableLUMSlice.rbegin()->first
                            , timeEarliestPEPMap
                            , &sampler
                            , nSample
                            , &storageTrajectoryElement
                            , NULL//pipeGnuplot_Trajectory
                            , extractlumParam.stDeviation
                            , &plotParam );
            cerr << "äÆóπ(nSample=" << nSample << ")" << endl;

            // debug code
            if( nSample <= 3 ) {
                cerr << "nSample is no more than 3." << endl;
            }
        }

        //
        // Clustering the trajectories
        //
        logTracking.clustering( TrackingProcessLogger::Start );
        //viewer.SetTrackingStatus( 1 );

        vector<CTrajectory> trajectoriesClustered;

        cout << "Started Clustering. " ;
        map<int,CTrajectory> trajectoryForClustering;
        vector<int> idxTrajectoryForClustering;
        map<unsigned long long, map<int,PosXYTID> > time_to_TrjIdx_and_pos;
        vector<TrajectoryElement>::iterator it = storageTrajectoryElement.begin();
        int iTrj = 0;
        for( ; it != storageTrajectoryElement.end(); ++it, ++iTrj ) {
            if( it->rbegin()->t - it->begin()->t >= clusteringParam.minLength ) {
                if( rand() < (int)( (float)RAND_MAX * 0.1f /*0.25f*//*0.6f*//*0.4f*/ ) ) { 
                    trajectoryForClustering[ iTrj ].push_back( *it );
                    idxTrajectoryForClustering.push_back( iTrj );
                }
            }
            if(  it->size() > 2 ) {
                TrajectoryElement::iterator itPosID = it->begin();
                for( ; itPosID != it->end(); ++itPosID ) {
                    time_to_TrjIdx_and_pos[ itPosID->t ][ iTrj ] = *itPosID;
                }
            }
        }

        cout << trajectoryForClustering.size() << " trajectories out of " << storageTrajectoryElement.size() << " are chosen for clustering."
             << endl;

        const float scale_m2px_for_clustering_video = 100.0f;
        Mat img( (int)( roi_height * scale_m2px_for_clustering_video ), (int)( roi_width * scale_m2px_for_clustering_video ), CV_8UC3 );
        VideoWriter videoWriter;
        if( flgOutputTrackingProcessData2Files ) {
            ostringstream oss;
#ifdef WINDOWS_OS
	        oss << "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_trajectories\\";
#endif
#ifdef LINUX_OS
			oss << "/home/fukushi/project/HumanTracking/bin/tmp_trajectories/";
#endif
            oss << "clusterieng_1st_" << timeTracking - commonParam.termTracking - timeEarliestPEPMap << ".avi";
            videoWriter.open( oss.str(), CV_FOURCC('X','V','I','D'), 10, Size( (int)img.cols, (int)img.rows ) );
        }

        map<unsigned long long,set<int> > time_to_hash_where_occupied;

        map<unsigned long long, map<int,PosXYTID> >::iterator itTrjIdxToPos = time_to_TrjIdx_and_pos.begin();
        for( ; itTrjIdxToPos != time_to_TrjIdx_and_pos.end(); ++itTrjIdxToPos ) {
            const unsigned long long time = itTrjIdxToPos->first;
            const int nTrj = itTrjIdxToPos->second.size();
            double* dist = new double[ nTrj * nTrj ];

            // Make a distance table at 'time'
            map<int,PosXYTID>::iterator itPos1 = itTrjIdxToPos->second.begin();
            for( int idx1 = 0; idx1 < nTrj; ++idx1, ++itPos1 ) {

                {
                    const int row_on_pepmap = scale_m2px * ( ( itPos1->second.x - roi_x ) + roi_height / 2.0f );
                    const int col_on_pepmap = scale_m2px * ( ( itPos1->second.y - roi_y ) + roi_width / 2.0f );
                    const int nCol = (int)( scale_m2px * roi_width );
                    int keyval = row_on_pepmap * nCol + col_on_pepmap + 1;
                    time_to_hash_where_occupied[ time ].insert( keyval );
                }

                map<int,PosXYTID>::iterator itPos2 = itPos1;
                for( int idx2 = idx1; idx2 < nTrj; ++idx2, ++itPos2 ) {
                    double dist_x, dist_y, d;
                    dist_x = ( itPos1->second.x - itPos2->second.x );
                    dist_y = ( itPos1->second.y - itPos2->second.y );
                    d = sqrt( dist_x * dist_x + dist_y * dist_y );
                    dist[ idx1 + idx2 * nTrj ] = dist[ idx2 + idx1 * nTrj ] = d;
                }
            }

            // clustering
            vector<int> classID( nTrj, -1 );
            int nClass = Clustering( &classID, dist, nTrj, 0.07/*0.18*//*0.22*//*0.2*//*0.07*/ );
            cout << " time=" << time << ", nClass=" << nClass << ", nTrj=" << nTrj << endl;

            if( flgOutputTrackingProcessData2Files ) {
                img = Scalar( 255, 255, 255 );

                map<int,PosXYTID>::iterator itPos = itTrjIdxToPos->second.begin();
                for( int i = 0; i < nTrj; ++i, ++itPos ) {
                    int row = (int)( scale_m2px_for_clustering_video * ( ( itPos->second.x - roi_x ) + roi_height / 2.0f ) );
                    int col = (int)( scale_m2px_for_clustering_video * ( ( itPos->second.y - roi_y ) + roi_width / 2.0f ) );
                    Scalar color = color_table[ classID[ i ] % sizeColorTable ];
                    img.at<Vec3b>( row, col )[ 0 ] = color[ 0 ];
                    img.at<Vec3b>( row, col )[ 1 ] = color[ 1 ];
                    img.at<Vec3b>( row, col )[ 2 ] = color[ 2 ];
                    //line( img, Point( col, row ), Point( col, row ), color_table[ classID[ itPos->first ] % sizeColorTable ], 1 );
                }

                videoWriter.write( img );
            }

            // Set classID to the field of 'ID' of PosXYTID at each trajectory.
            map<int,PosXYTID>::iterator itPos = itTrjIdxToPos->second.begin();
            for( int idx = 0; idx < nTrj; ++idx, ++itPos ) {
                map<int,CTrajectory>::iterator itIdxToTrj = trajectoryForClustering.find( itPos->first ); 
                if( itIdxToTrj != trajectoryForClustering.end() ) {
                    TrajectoryElement::iterator it = itIdxToTrj->second.begin()->find( PosXYT( 0.0, 0.0, time ) );
                    if( it != trajectoryForClustering[ itPos->first ].begin()->end() ) {
                        PosXYTID pos_id = *it;
                        itIdxToTrj->second.begin()->erase( it );
                        pos_id.ID = classID[ idx ];
                        itIdxToTrj->second.begin()->insert( pos_id );
                    }
                }
            }

            delete [] dist;
        }

        // Make a distance table among every trajectory
        cout << " Make a distance table among every trajectory...";
        const int nTrj = trajectoryForClustering.size();
        double* dist = new double[ nTrj * nTrj ];
        map<int,CTrajectory>::iterator itTrj1 = trajectoryForClustering.begin();
        for( int idx1 = 0; idx1 < nTrj; ++idx1, ++itTrj1 ) {
            map<int,CTrajectory>::iterator itTrj2 = itTrj1;
            for( int idx2 = idx1; idx2 < nTrj; ++idx2, ++itTrj2 ) {
                dist[ idx1 + idx2 * nTrj ] = dist[ idx2 + idx1 * nTrj ] = 
                    dist_trajectory_element( *itTrj1->second.begin(), *itTrj2->second.begin() );
            }
        }
        cout << "done." << endl;
        
        //Clustering
        //vector<int> classID( nTrj, -1 );
        //int nClass = Clustering( &classID, dist, nTrj, 0.1 );

        //cerr << "  nClass=" << nClass << endl;

        //vector< vector<int> > tmp;
        //Clustering2( &tmp, classID, dist, nTrj, 0.1, 10.0 );
        cout << " Clustering3()...";
        vector< vector<int> > tmp;
        Clustering3( &tmp, dist, nTrj, 0.1 );
        cout << "done." << endl;

        cout << " Storing the trajectories to trajectoriesClustered...";
        trajectoriesClustered.clear();
        trajectoriesClustered.resize( tmp.size() );
        for( int i = 0; i < tmp.size(); ++i ) {
            //CTrajectory trj;
            for( int j = 0; j < tmp[ i ].size(); ++j ) {
                //trj.push_back( *trajectoryForClustering[ tmp[ i ][ j ] ].begin() );
                trajectoriesClustered[ i ].push_back( *trajectoryForClustering[ idxTrajectoryForClustering[ tmp[ i ][ j ] ] ].begin() );
            }
            //trajectoriesClustered.push_back( trj );
        }
        cout << "done." << endl;

        cerr << "Done. "
             << trajectoriesClustered.size() << " clusters has been made." << endl;

        //
        // Output process information of clustering
//        {
//            double sumValue = accumulate( sampler.begin(), sampler.end(), 0.0, PosXYTV_Sum() );
//            unsigned int nSample = (unsigned int)( plotParam.kSample /*3.0e-2*//*1.04e-4*/ * sumValue );
//            OutputProcess( timeTracking - commonParam.termTracking//tableLUMSlice.begin()->first
//                           , timeTracking//tableLUMSlice.rbegin()->first
//                            , timeEarliestPEPMap
//                            , &sampler
//                            , nSample
//                            , &trajectoriesClustered
//#ifdef WINDOWS_OS
//			                , "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_trajectories\\"
//#endif
//#ifdef LINUX_OS
//			                , "/home/fukushi/project/HumanTracking/bin/tmp_trajectories/"
//#endif
//                            , "before-integration"//oss.str()
//                            , NULL
//                            , &plotParam );
//        }


        cout << "Integrating Clusters that have similar shape..." << endl;

        const int nCluster_before_integration = trajectoriesClustered.size();
        //vector<CTrajectory> trajectoriesAveraged( nCluster );
        //for( int i = 0; i < nCluster; ++i ) {
        //    trajectoriesClustered[ i ].Integrate( &trajectoriesAveraged[ i ] );
        //}

        //
        // Output process information of clustering
//        {
//            double sumValue = accumulate( sampler.begin(), sampler.end(), 0.0, PosXYTV_Sum() );
//            unsigned int nSample = (unsigned int)( plotParam.kSample /*3.0e-2*//*1.04e-4*/ * sumValue );
//            OutputProcess( timeTracking - commonParam.termTracking//tableLUMSlice.begin()->first
//                            , timeTracking//tableLUMSlice.rbegin()->first
//                            , timeEarliestPEPMap
//                            , &sampler
//                            , nSample
//                            , &trajectoriesAveraged
//#ifdef WINDOWS_OS
//			                , "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_trajectories\\"
//#endif
//#ifdef LINUX_OS
//			                , "/home/fukushi/project/HumanTracking/bin/tmp_trajectories/"
//#endif
//                            , "averaged"//oss.str()
//                            , NULL
//                            , &plotParam );
//        }

        CTrajectory_Distance distanceTrajectory( clusteringParam.distanceLimit, clusteringParam.nLimit, clusteringParam.minCommonTimeRange, false );
        double* distTable = new double[ nCluster_before_integration * nCluster_before_integration ];
        {
//            ostringstream oss;
//#ifdef WINDOWS_OS
//		    oss << "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_trajectories\\";
//#endif
//#ifdef LINUX_OS
//			oss << "/home/fukushi/project/HumanTracking/bin/tmp_trajectories/";
//#endif
//            oss << "distTable_" << timeTracking - commonParam.termTracking - timeEarliestPEPMap << ".txt";
//            ofstream ofs( oss.str().c_str() );
            for( size_t iTrj1 = 0; iTrj1 < nCluster_before_integration; ++iTrj1 ) {
                for( size_t iTrj2 = iTrj1; iTrj2 < nCluster_before_integration; ++iTrj2 ) {
                    distTable[ iTrj1 * nCluster_before_integration + iTrj2 ] = distTable[ iTrj2 * nCluster_before_integration + iTrj1 ]
                    //= distanceTrajectory( trajectoriesAveraged[ iTrj1 ], trajectoriesAveraged[ iTrj2 ] );
                    = distanceTrajectory( trajectoriesClustered[ iTrj1 ], trajectoriesClustered[ iTrj2 ] );
                    //ofs << "distTable[" << iTrj1 << "][" << iTrj2 << "]=" << distTable[ iTrj1 * nCluster + iTrj2 ] << endl;
                }
            }
        }
       
        {
            vector< vector<int> > clst;
            Clustering3( &clst, distTable, nCluster_before_integration, clusteringParam.thConnect );
            vector<CTrajectory> tmp( clst.size() );
            for( int i = 0; i < clst.size(); ++i ) {
                for( int j = 0; j < clst[ i ].size(); ++j ) {
                    tmp[ i ].insert( tmp[ i ].end()
                                   , trajectoriesClustered[ clst[ i ][ j ] ].begin()
                                   , trajectoriesClustered[ clst[ i ][ j ] ].end() );
                }
            }
            trajectoriesClustered = tmp;
        }

        delete [] distTable;

        cout << "Done. # of the cluster is " << trajectoriesClustered.size() << "." << endl;

        //
        // Output process information of clustering
        if( flgOutputTrackingProcessData2Files ) {
            double sumValue = accumulate( sampler.begin(), sampler.end(), 0.0, PosXYTV_Sum() );
            unsigned int nSample = (unsigned int)( plotParam.kSample /*3.0e-2*//*1.04e-4*/ * sumValue );
            OutputProcess( timeTracking - commonParam.termTracking//tableLUMSlice.begin()->first
                            , timeTracking//tableLUMSlice.rbegin()->first
                            , timeEarliestPEPMap
                            , &sampler
                            , nSample
                            , &trajectoriesClustered
#ifdef WINDOWS_OS
			                , "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_trajectories\\"
#endif
#ifdef LINUX_OS
			                , "/home/fukushi/project/HumanTracking/bin/tmp_trajectories/"
#endif
                            , ""//oss.str()
                            , NULL
                            , &plotParam );
        }

        const int nCluster = trajectoriesClustered.size();

        vector<CTrajectory> trajectoriesAveraged( nCluster );
        for( int i = 0; i < nCluster; ++i ) {
            CTrajectory tmpTrj;
            TrajectoryElement trjElement;
            trajectoriesClustered[ i ].Integrate( &tmpTrj );

            for( TrajectoryElement::const_iterator it = tmpTrj.front().begin(); it != tmpTrj.front().end(); ++it ) {
                PosXYTID posid;
                posid.x = it->x;
                posid.y = it->y;
                posid.t = it->t;
                posid.ID = i;
                trjElement.insert( posid );
            }
            trajectoriesAveraged[ i ].push_back( trjElement );
        }
        
        if( flgOutputTrackingProcessData2Files ) {
            double sumValue = accumulate( sampler.begin(), sampler.end(), 0.0, PosXYTV_Sum() );
            unsigned int nSample = (unsigned int)( plotParam.kSample /*3.0e-2*//*1.04e-4*/ * sumValue );
            OutputProcess( timeTracking - commonParam.termTracking//tableLUMSlice.begin()->first
                            , timeTracking//tableLUMSlice.rbegin()->first
                            , timeEarliestPEPMap
                            , &sampler
                            , nSample
                            , &trajectoriesAveraged
#ifdef WINDOWS_OS
			                , "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_trajectories\\"
#endif
#ifdef LINUX_OS
			                , "/home/fukushi/project/HumanTracking/bin/tmp_trajectories/"
#endif
                            , "averaged"//oss.str()
                            , NULL
                            , &plotParam );
        }

        logTracking.clustering( TrackingProcessLogger::End );

        logTracking.renovation( TrackingProcessLogger::Start );
        //
        // Selection of the best combination of the clusters which fits
        // to the temporal stack of the occupancy maps.
        
        cout << endl;
        cout << "Selecting the best combination of the clusters..." << endl;
        cout << " creating a hash table...";
        logTracking.renovation_hashmap( TrackingProcessLogger::Start );
        // Create a hash table that maps a trajectory (CTrajectory) to where it occupied on plan-view map.
        vector< map<unsigned long long, set<int> > > trj_time_to_hash_where_occupied( nCluster );
        for( int i = 0; i < trajectoriesClustered.size(); ++i ) {
            for( CTrajectory::iterator itTrj = trajectoriesClustered[ i ].begin(); itTrj != trajectoriesClustered[ i ].end(); ++itTrj ) {
                for( TrajectoryElement::iterator itPos = itTrj->begin(); itPos != itTrj->end(); ++itPos ) {
                    const int row_on_pepmap = scale_m2px * ( ( itPos->x - roi_x ) + roi_height / 2.0f );
                    const int col_on_pepmap = scale_m2px * ( ( itPos->y - roi_y ) + roi_width / 2.0f );
                    const int nCol = (int)( scale_m2px * roi_width );
                    int keyval = row_on_pepmap * nCol + col_on_pepmap + 1;
                    trj_time_to_hash_where_occupied[ i ][ itPos->t ].insert( keyval );
                }
            }
        }   
        logTracking.renovation_hashmap( TrackingProcessLogger::End );
        cout << endl;

        // Create an 'independence' table for every two trajectories. 
        // '0' if they don't mutually interfere, which means they can exist coincidently in a tracking result.
        // '<0' otherwise.
        cout << " creating an independence table...";
        double* tableIndependent = new double[ nCluster * nCluster ];
        {
             ostringstream oss;
#ifdef WINDOWS_OS
		    oss << "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_trajectories\\";
#endif
#ifdef LINUX_OS
			oss << "/home/fukushi/project/HumanTracking/bin/tmp_trajectories/";
#endif
            oss << "independence_table_" << timeTracking - commonParam.termTracking - timeEarliestPEPMap << ".txt";
            ofstream ofs( oss.str().c_str() );
            
            for( int iTrj1 = 0; iTrj1 < nCluster; ++iTrj1 ) {
                for( int iTrj2 = iTrj1; iTrj2 < nCluster; ++iTrj2 ) {
                    tableIndependent[ iTrj1 + iTrj2 * nCluster ] 
                        = tableIndependent[ iTrj2 + iTrj1 * nCluster ] 
                        = areIndependent( trajectoriesAveraged[ iTrj1 ].front(), trajectoriesAveraged[ iTrj2 ].front(), 0.12/*clusteringParam.thConnect*/ );
                    ofs << "tableIndependent[" << iTrj1 << "][" << iTrj2 << "]=";
                    if( tableIndependent[ iTrj1 + iTrj2 * nCluster ] == 0.0 ) {
                        ofs << "true" << endl;
                    } else {
                        ofs << "false" << endl;
                    }
                }
            }
        }
        cout << "done." << endl;

        // Make all the possible combinations in which trajectories are independent.
        cout << " making all the possible combinations..." << endl;
        vector< vector<int> > combination;
        vector<double> score;
        {
            Clustering3( &combination, tableIndependent, nCluster, 0.1 );
            cout << "  # of combinations: " << combination.size() << endl;

            ostringstream oss;
#ifdef WINDOWS_OS
		    oss << "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_trajectories\\";
#endif
#ifdef LINUX_OS
			oss << "/home/fukushi/project/HumanTracking/bin/tmp_trajectories/";
#endif
            oss << "possible_combinations_" << timeTracking - commonParam.termTracking - timeEarliestPEPMap << ".txt";
            ofstream ofs( oss.str().c_str() );

            score.resize( combination.size() );
            for( int i = 0; i < combination.size(); ++i ) {
                logTracking.renovation_score( TrackingProcessLogger::Start );
                score[ i ] = fitting_score( combination[ i ], trajectoriesAveraged, time_to_hash_where_occupied, trj_time_to_hash_where_occupied );
                logTracking.renovation_score( TrackingProcessLogger::End );
                ofs << "Combination " << i << ": "
                     << "fitting_score=" << score[ i ]
                     << ", ";
                cout << "  Combination " << i << ": "
                     << "fitting_score=" << score[ i ]
                     << ", ";
                for( int j = 0; j < combination[ i ].size(); ++j ) {
                    ofs << combination[ i ][ j ] << " ";
                    cout << combination[ i ][ j ] << " ";
                }
                ofs << endl;
                cout << endl;

                if( flgOutputTrackingProcessData2Files ) {
                    OutputProcess( timeTracking - commonParam.termTracking//tableLUMSlice.begin()->first
                                 , timeTracking//tableLUMSlice.rbegin()->first
                                    , timeEarliestPEPMap
                                    , combination[ i ]
                                    , i
                                    , trajectoriesAveraged
#ifdef WINDOWS_OS
			                        , "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_trajectories\\"
#endif
#ifdef LINUX_OS
			                        , "/home/fukushi/project/HumanTracking/bin/tmp_trajectories/"
#endif
                                    , "averaged"
                                    , &plotParam );    
                }
            }
        }
        cout << endl;
        delete [] tableIndependent;
        tableIndependent = NULL;

        // Select the high-scored combinations
        cout << " High-scored combinations: ";
        vector< vector<int> > combination_highscored;
        double max = 0.0;
        int idxCombinationMax = -1;
        for( int i = 0; i < combination.size(); ++i ) {
            if( score[ i ] > max ) {
                max = score[ i ];
                idxCombinationMax = i;
            }
            //if( score[ i ] > 0.55 ) {
            //    combination_highscored.push_back( combination[ i ] );
            //    cout << i << " ";
            //}
        }

        int nConnectionPattern_for_record = 0;

        vector<TrajectoryElement> opt;
        vector<int> idOpt;
        if( idxCombinationMax != -1 ) {
            combination_highscored.push_back( combination[ idxCombinationMax ] );
            cout << idxCombinationMax << endl;

            vector<CTrajectory> trajectoriesPrevResult;
            map<int,int> idxResultTrjToID;
            const int nPrevResultTrj = resultTrajectory.size();
            {
                int idx = nCluster;
                for( map<int,CTrajectory>::iterator itResult = resultTrajectory.begin(); itResult != resultTrajectory.end(); ++itResult, ++idx ) {
                    trajectoriesPrevResult.push_back( itResult->second );
                    idxResultTrjToID[ idx ] = itResult->first;
                }
            }

            // Create a 'connectable' table for every two trajectories.
            // '0' if they can connect smoothly.
            // '<0' otherwise.
            cout << " creating a connectable table..." << flush;
            int sizeTableConnectable = nCluster + nPrevResultTrj;
            double* tableConnectable = new double[ sizeTableConnectable * sizeTableConnectable ];
            {
                 ostringstream oss;
    #ifdef WINDOWS_OS
		        oss << "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_trajectories\\";
    #endif
    #ifdef LINUX_OS
			    oss << "/home/fukushi/project/HumanTracking/bin/tmp_trajectories/";
    #endif
                oss << "connectable_table_" << timeTracking - commonParam.termTracking - timeEarliestPEPMap << ".txt" << flush;
                ofstream ofs( oss.str().c_str() );
           
                cout << "(nCluster=" << nCluster << ",nPrevResultTrj=" << nPrevResultTrj << ")" << endl << flush;

                const double threshold = 3.0;
                for( int iTrj1 = 0; iTrj1 < sizeTableConnectable; ++iTrj1 ) {
                    for( int iTrj2 = iTrj1; iTrj2 < sizeTableConnectable; ++iTrj2 ) {
                        double value;
                        cout << "  tableConnectable[" << iTrj1 << "][" << iTrj2 << "]=" << flush;
                        if( iTrj1 >= nCluster && iTrj2 >= nCluster ) {
                            value = -0.5;//-1.0
                        } else if( iTrj1 >= nCluster ) {
                            value = areConnectable( trajectoriesPrevResult[ iTrj1 - nCluster ].front(), trajectoriesAveraged[ iTrj2 ].front(), threshold, rnvtrjParam.termConnect );
                        } else if( iTrj2 >= nCluster ) {
                            value = areConnectable( trajectoriesAveraged[ iTrj1 ].front(), trajectoriesPrevResult[ iTrj2 - nCluster ].front(), threshold, rnvtrjParam.termConnect );
                        } else {
                            value = areConnectable( trajectoriesAveraged[ iTrj1 ].front(), trajectoriesAveraged[ iTrj2 ].front(), threshold, rnvtrjParam.termConnect );                   
                        }
                        cout << value << endl << flush;
                        tableConnectable[ iTrj1 + iTrj2 * sizeTableConnectable ] = tableConnectable[ iTrj2 + iTrj1 * sizeTableConnectable ] = value;
                        ofs << "tableConnectable[" << iTrj1 << "][" << iTrj2 << "]=" << flush;
                        if( tableConnectable[ iTrj1 + iTrj2 * sizeTableConnectable ] == 0.0 ) {
                            ofs << "true" << endl << flush;
                        } else {
                            ofs << "false" << endl << flush;
                        }
                    }
                }
            }
            cout << "done." << endl << flush;


            // Try to connect trajectories in each combination if possible
            cout << " trying to connect trajectories...";
            vector< vector< vector<int> > > connection_patterns;
            for( int i = 0; i < combination_highscored.size(); ++i ) {
                vector< vector<int> > init( combination_highscored[ i ].size() );
                set<int> idxPrevResultCluster;
                for( int i = 0; i < resultTrajectory.size(); ++ i ) {
                    idxPrevResultCluster.insert( nCluster + i );
                }
               for( int j = 0; j < combination_highscored[ i ].size(); ++j ) {
                    init[ j ].push_back( combination_highscored[ i ][ j ] );
                    int k = 0;
                    for( map<int,CTrajectory>::iterator itResult = resultTrajectory.begin(); itResult != resultTrajectory.end(); ++itResult, ++k ) {
                        if( areCoherent( trajectoriesAveraged[ combination_highscored[ i ][ j ] ].front()
                                       , itResult->second.front(), clusteringParam.thConnect ) == 0.0 ) {
                            init[ j ].push_back( nCluster + k );
                            idxPrevResultCluster.erase( nCluster + k );
                            sort( init[ j ].begin(), init[ j ].end() );
                        }
                    }
                }
                for( set<int>::iterator it = idxPrevResultCluster.begin(); it != idxPrevResultCluster.end(); ++it ) {
                    // Add the result trajectory which didn't match with any trajectories in trajectoriesAveraged.
                    vector<int> cluster( 1, *it );
                    init.push_back( cluster );
                }
                connect( &connection_patterns, init, combination_highscored[ i ], nCluster, sizeTableConnectable, tableConnectable );
            }
            sort( connection_patterns.begin(), connection_patterns.end() );
            connection_patterns.erase( unique( connection_patterns.begin(), connection_patterns.end() )
                                     , connection_patterns.end() );
            cout << " done." << endl;
        
            nConnectionPattern_for_record = connection_patterns.size();
            {
                ostringstream oss;
    #ifdef WINDOWS_OS
		        oss << "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_trajectories\\";
    #endif
    #ifdef LINUX_OS
			    oss << "/home/fukushi/project/HumanTracking/bin/tmp_trajectories/";
    #endif
                oss << "connection_patterns_" << timeTracking - commonParam.termTracking - timeEarliestPEPMap << ".txt";
                ofstream ofs( oss.str().c_str() );

                for( int i = 0; i < connection_patterns.size(); ++i ) {
                    ofs << "Pattern " << i << ": ";
                    cout << "  Pattern " << i << ": ";
                    for( int j = 0; j < connection_patterns[ i ].size(); ++j ) {
                        ofs << "(";
                        cout << "(";
                        for( int k = 0; k < connection_patterns[ i ][ j ].size(); ++k ) {
                            cout << connection_patterns[ i ][ j ][ k ];
                            ofs << connection_patterns[ i ][ j ][ k ];
                            if( k != connection_patterns[ i ][ j ].size() - 1 ) {
                                ofs << ",";
                                cout << ",";
                            }
                        }
                        ofs << ") ";
                        cout << ") ";
                    }
                    ofs << endl;
                    cout << endl;
                }
            }

            delete [] tableConnectable;
            tableConnectable = NULL;


            // Find the optimum
	        cout << " Started optimization." << endl;
            {
                ostringstream oss;
    #ifdef WINDOWS_OS
		        oss << "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_trajectories\\";
    #endif
    #ifdef LINUX_OS
			    oss << "/home/fukushi/project/HumanTracking/bin/tmp_trajectories/";
    #endif
                oss << "optimization_" << timeTracking - commonParam.termTracking - timeEarliestPEPMap << ".txt";
                ofstream ofs( oss.str().c_str() );

                vector< vector<TrajectoryElement> > trajectories_for_pattern( connection_patterns.size() );
                for( int i = 0; i < connection_patterns.size(); ++i ) {
                    for( int j = 0; j < connection_patterns[ i ].size(); ++j ) {
                        trajectories_for_pattern[ i ].resize( connection_patterns[ i ].size() );
                        for( int k = 0; k < connection_patterns[ i ][ j ].size(); ++k ) {
                            int idxCluster = connection_patterns[ i ][ j ][ k ];
		            if( idxCluster < nCluster ) {
			        trajectories_for_pattern[ i ][ j ].insert( trajectoriesAveraged[ idxCluster ].front().begin(), trajectoriesAveraged[ idxCluster ].front().end() );
		            } else {
				trajectories_for_pattern[ i ][ j ].insert( trajectoriesPrevResult[ idxCluster - nCluster ].front().begin(), trajectoriesPrevResult[ idxCluster - nCluster ].front().end() );
			    }
                        }
                    }
                }
        
                int idxOptPattern = -1;
                if( connection_patterns.size() >= 2 ) {
                    cout << " scoring the patterns..." << endl;
                    vector<double> score( connection_patterns.size() );
                    for( int i = 0; i < connection_patterns.size(); ++i ) {
                        vector<double> speed( trajectories_for_pattern[ i ].size(), 0.0 );
                        vector<int> count( trajectories_for_pattern[ i ].size(), 0 );
                        double speed_overall = 0.0;
                        int count_total = 0;
                        for( int j = 0; j < trajectories_for_pattern[ i ].size(); ++j ) {
                            PosXYTID posPrev;
                            for( TrajectoryElement::iterator itPos = trajectories_for_pattern[ i ][ j ].begin(); itPos != trajectories_for_pattern[ i ][ j ].end(); ++itPos ) {
                                if( itPos != trajectories_for_pattern[ i ][ j ].begin() ) {
                                    double dx = itPos->x - posPrev.x;
                                    double dy = itPos->y - posPrev.y;
                                    double dt = (double)( itPos->t - posPrev.t ) * 1.0e-6;
                                    double v = sqrt( dx * dx + dy * dy );
                                    speed[ j ] += v;
                                    count[ j ] += 1;
                                    speed_overall += v;
                                    ++count_total;
                                }
                                posPrev = *itPos;
                            }
                            speed[ j ] /= (double)count[ j ];
                        }
                        speed_overall /= (double)count_total;
                        double _score = speed_overall + 200.0 * (double)trajectories_for_pattern[ i ].size();
                        score[ i ] = _score;
                        cout << "  Pattern " << i << ": " << speed_overall << "(speed), " << trajectories_for_pattern[ i ].size() << "(# of people) -> " << _score << endl;
                        ofs << "Pattern " << i << ": " << speed_overall << "(speed), " << trajectories_for_pattern[ i ].size() << "(# of people) -> " << _score << endl;
                    }
                    cout << " done." << endl;

                    double min = -1.0;
                    for( int i = 0; i < score.size(); ++i ) {
                        if( score[ i ] < min || min < 0.0 ) {
                            min = score[ i ];
                            idxOptPattern = i;
                        }
                    }
                    opt = trajectories_for_pattern[ idxOptPattern ];
                } else {
                    idxOptPattern = 0;
                    opt = trajectories_for_pattern[ 0 ];
                }

                cout << "  Pattern " << idxOptPattern << " has been chosen as a final result." << endl;
                ofs << "  Pattern " << idxOptPattern << " has been chosen as a final result." << endl;
                //Debug code!
                idOpt.resize( opt.size(), -1 );
                for( int i = 0; i < opt.size(); ++i ) {
                    for( int j = 0; j < connection_patterns[ idxOptPattern ][ i ].size(); ++j ) {
                        map<int,int>::iterator it = idxResultTrjToID.find( connection_patterns[ idxOptPattern ][ i ][ j ] );
                        if( it != idxResultTrjToID.end() ) {
                            idOpt[ i ] = it->second;
                        }
                    }

                }
                cout << "  ID: ";
                for( int j = 0; j < connection_patterns[ idxOptPattern ].size(); ++j ) {
                    ofs << "(";
                    cout << "(";
                    for( int k = 0; k < connection_patterns[ idxOptPattern ][ j ].size(); ++k ) {
                        cout << connection_patterns[ idxOptPattern ][ j ][ k ];
                        ofs << connection_patterns[ idxOptPattern ][ j ][ k ];
                        if( k != connection_patterns[ idxOptPattern ][ j ].size() - 1 ) {
                            ofs << ",";
                            cout << ",";
                        }
                    }
                    ofs << ")->" << idOpt[ j ] << " ";
                    cout << ")->" << idOpt[ j ] << " ";

                }
            }
        } else {
            cout << "No cluster found." << endl;
        }

        cout << "Done." << endl << flush;
        logTracking.renovation( TrackingProcessLogger::End );

        {
                ostringstream oss;
    #ifdef WINDOWS_OS
		        //oss << "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_trajectories\\";
    #endif
    #ifdef LINUX_OS
			    //oss << "/home/fukushi/project/HumanTracking/bin/tmp_trajectories/";
    #endif
                oss << "tracking.csv";
                ofstream ofs( oss.str().c_str(), std::ios::out | std::ios::app );
                
                ofs << timeTracking - commonParam.termTracking - timeEarliestPEPMap << ", "
                    << nCluster << ", " 
                    << combination.size() << ", " 
                    << nConnectionPattern_for_record << endl;
        }

        logTracking.finishing( TrackingProcessLogger::Start );

        // IDñ¢äÑÇËìñÇƒÇÃãOê’Ç…êVÇµÇ¢IDÇêUÇÈ
        for( vector<int>::iterator itID = idOpt.begin(); itID != idOpt.end(); ++itID ) {
            if( *itID == -1 ) {
                *itID = idNext;
                ++idNext;
            }
        }

        // ãOê’ÇÃï‚ä‘ÇçsÇ§
	cout << "Interpolation of the trajectories (opt.size()=" << opt.size() << ")..." << flush;
	int _count_debug = 0;
        for( vector<TrajectoryElement>::iterator itTrj = opt.begin(); itTrj != opt.end(); ++itTrj ) {
	    cout << " opt[" << _count_debug << "](size=" << itTrj->size() << "):" << flush;
	    if( itTrj->empty() ) {
		continue;
	    }
            TrajectoryElement::iterator it = itTrj->begin();
            TrajectoryElement::iterator itNext = it;
            advance( itNext, 1 );
            while( itNext != itTrj->end() ) {
                int idxCluster = it->ID;
                while( it->t + commonParam.intervalTrajectory < itNext->t ) {
                    PosXYTID pos;
                    pos.x = ( itNext->x - it->x ) / ( (double)( itNext->t - it->t ) * 1.0e-6 ) * ( (double)commonParam.intervalTrajectory * 1.0e-6 ) + it->x;
                    pos.y = ( itNext->y - it->y ) / ( (double)( itNext->t - it->t ) * 1.0e-6 ) * ( (double)commonParam.intervalTrajectory * 1.0e-6 ) + it->y;
                    pos.t = it->t + commonParam.intervalTrajectory;
                    pos.ID = nCluster + 1;
                    it = itTrj->insert( it, pos );
		    cout << "+" << flush;
                }
                ++it;
                ++itNext;
		cout << "-" << flush;
            }
	    cout << " done." << endl << flush;
	    ++_count_debug;
        }
	cout << "done." << endl << flush;

        // åãâ ÇÃï€ë∂
	cout << "Preparing 'resultTrajectory'..." << flush;
        p_result->clear();
        resultTrajectory.clear();
        vector<int>::iterator itID = idOpt.begin();
        for( vector<TrajectoryElement>::iterator itTrj = opt.begin(); itTrj != opt.end(); ++itTrj, ++itID ) {
            int id = ( *itID == -1 ) ? idNext++ : *itID;
            CTrajectory trj;
            trj.push_back( *itTrj );
            resultTrajectory[ id ] = trj; // resultTrajectory$B$O<!2s$NDI@W$K$b;H$&$N$G>/$7D9$$(B
            //trj.Clip( 0, timeTracking - ( commonParam.termTracking - commonParam.intervalTracking ) );
            //(*p_result)[ id ] = trj; // $BI=<(MQ(B
        }
	cout << "done." << endl << flush;

        //map<unsigned long long, multimap<int,Point2d> > ext_result;

	cout << "Preparing 'p_result' and 'p_ext_result'..." << flush;
        unsigned long long time;
        p_ext_result->insert( remainedExtendedResult.begin(), remainedExtendedResult.end() );
        for( /*unsigned long long*/ time = max( timeTracking - commonParam.termTracking, timeEarliestPEPMap )
                ; time < timeTracking - ( commonParam.termTracking - commonParam.intervalTracking )
                ; time += commonParam.intervalTrajectory ) {

            (*p_result)[ time ];
            (*p_ext_result)[ time ];

            //const set<TIME_MICRO_SEC>::iterator it_start_pepmap_time = time_of_received_pepmap.lower_bound( time );
            //const set<TIME_MICRO_SEC>::iterator it_end_pepmap_time = time_of_received_pepmap.lower_bound( time + commonParam.intervalTrajectory );
            //for( set<TIME_MICRO_SEC>::iterator it = it_start_pepmap_time; it != it_end_pepmap_time; ++it ) {
            //   (*p_ext_result)[ *it ]; 
            //}

	    cout << " time=" << time << "..." << flush;
            map<int,CTrajectory>::iterator itResult = resultTrajectory.begin();
            for( ; itResult != resultTrajectory.end(); ++itResult ) {
                TrajectoryElement::iterator itPos = itResult->second.front().find( PosXYT( 0.0, 0.0, time ) );
                if( itPos != itResult->second.front().end() ) {
                    (*p_result)[ time ][ itResult->first ] = Point2d( itPos->x, itPos->y );

                    if( itPos->ID < nCluster) {
                        set<int>::iterator itKeyval = trj_time_to_hash_where_occupied[ itPos->ID ][ time ].begin();
                        for( ; itKeyval != trj_time_to_hash_where_occupied[ itPos->ID ][ time ].end(); ++itKeyval ) {
                            const int cols = (int)( scale_m2px * roi_width );
                            const int row_on_pepmap = ( *itKeyval - 1 ) / cols;
                            const int col_on_pepmap = ( *itKeyval - 1 ) % cols;
                        
                            Point2d pos;
                            pos.x = ( (double)row_on_pepmap / scale_m2px ) + roi_x - ( roi_height / 2.0f );
                            pos.y = ( (double)col_on_pepmap / scale_m2px ) + roi_y - ( roi_width / 2.0f );
                            (*p_ext_result)[ time ].insert( pair<int,Point2d>( itResult->first, Point2d( pos.x, pos.y ) ) );                                            
                        }
                    }
                }
            }
	    cout << "done." << endl << flush;
        }
	cout << "done." << endl << flush;
        /*
        remainedExtendedResult.clear();
        for( ; time < timeTracking; time += commonParam.intervalTrajectory ) {
            const set<TIME_MICRO_SEC>::iterator it_start_pepmap_time = time_of_received_pepmap.lower_bound( time );
            const set<TIME_MICRO_SEC>::iterator it_end_pepmap_time = time_of_received_pepmap.lower_bound( time + commonParam.intervalTrajectory );

            map<int,CTrajectory>::const_iterator itResult = resultTrajectory.begin();
            for( ; itResult != resultTrajectory.end(); ++itResult ) {
                const CTrajectory& trajectory = itResult->second;
                if( trajectory.size() > 0 ) {
                    TrajectoryElement::iterator itPos = trajectory.front().begin();
                    for( ; itPos != trajectory.front().end(); ++itPos ) {
                        if( itPos->t == time ) {
                            //(*p_result)[ time ][ itResult->first ] = Point2d( itPos->x, itPos->y );

                            const int trj_no = itPos->ID;
                            //if( trj_no < trajectoriesClustered.size() ) {
                                for( CTrajectory::iterator it = trajectoriesClustered[ trj_no ].begin()
                                   ; it != trajectoriesClustered[ trj_no ].end(); ++it ) {
                                    TrajectoryElement::iterator itPos2;
                                    if( ( itPos2 = it->find( PosXYT( 0.0, 0.0, time ) ) ) != it->end() ) {
                                        (*p_ext_result)[ time ].insert( pair<int,Point2d>( itResult->first, Point2d( itPos2->x, itPos2->y ) ) );
                                        PosXYT pos0, pos1;
                                        pos0 = *itPos2;
                                        advance( itPos2, 1 );
                                        if( itPos2 != it->end() ) {
                                            pos1 = *itPos2;
                                            for( set<TIME_MICRO_SEC>::iterator it = it_start_pepmap_time; it != it_end_pepmap_time; ++it ) {
                                                PosXYT pos;
                                                pos.x = ( pos1.x - pos0.x ) / ( (double)( pos1.t - pos0.t ) ) * ( (double)( *it - *it_start_pepmap_time ) ) + pos0.x;
                                                pos.y = ( pos1.y - pos0.y ) / ( (double)( pos1.t - pos0.t ) ) * ( (double)( *it - *it_start_pepmap_time ) ) + pos0.y;
                                                remainedExtendedResult[ *it ].insert( pair<int,Point2d>( itResult->first, Point2d( pos.x, pos.y ) ) );
                                            }
                                        }
                                    }
                                }
                            //}
                        }
                    }
                }
            }   
        }*/

        //viewer.SetResult( *p_result );

        //
        // Output process information of renovation
        //if( myRank == 0 ) {
	cout << "Output the result to the file..." << flush;
        if( flgOutputTrackingProcessData2Files ) {
            {
                double sumValue = accumulate( sampler.begin(), sampler.end(), 0.0, PosXYTV_Sum() );
                unsigned int nSample = (unsigned int)( plotParam.kSample /*3.0e-2*//*1.04e-4*/ * sumValue );
                OutputProcess( timeTracking - commonParam.termTracking//tableLUMSlice.begin()->first
                             , timeTracking//tableLUMSlice.rbegin()->first
                             , timeEarliestPEPMap
                             , &sampler
                             , nSample
                             , &resultTrajectory
                             , NULL
                             , &plotParam );
            }
        }
	cout << "done." << endl << flush;

	cout << "Finalizing..." << flush;
        //
        // sampler
        sampler.erase( sampler.begin()
                        , lower_bound( sampler.begin()
                                    , sampler.end()
                                    , pair<PosXYT,unsigned long>( PosXYT( 0.0, 0.0, timeTracking - ( commonParam.termTracking - commonParam.intervalTracking ) ), 0 )
                                    , PosXYT_T_Less() ) );

        //
        // storageLUM
        storageLUM.erase( storageLUM.begin()
                        , storageLUM.lower_bound( timeTracking - ( commonParam.termTracking - commonParam.intervalTracking ) ) );

        //
        // tableLUMSlice
        tableLUMSlice.erase( tableLUMSlice.begin()
                            , tableLUMSlice.lower_bound( timeTracking - ( commonParam.termTracking - commonParam.intervalTracking ) ) );


        //
        // storageTrajectoryElement
        // [timeTracking - commonParam.termTracking, timeTracking]$B$K%/%j%C%T%s%0(B
        CTrajectory newStorageTrajectoryElement;
        newStorageTrajectoryElement.assign( storageTrajectoryElement.begin(), storageTrajectoryElement.end() );
        newStorageTrajectoryElement.Clip( timeTracking - ( commonParam.termTracking - commonParam.intervalTracking ), timeTracking );
        storageTrajectoryElement.assign( newStorageTrajectoryElement.begin(), newStorageTrajectoryElement.end() );

        time_of_received_pepmap.erase( time_of_received_pepmap.begin()
                                     , time_of_received_pepmap.lower_bound( timeTracking - ( commonParam.termTracking - commonParam.intervalTracking ) ) );


        //prevTrajectoriesClustered = trajectoriesClustered;


        timeTracking += commonParam.intervalTracking;

        ret = true;

        logTracking.finishing( TrackingProcessLogger::End );

        logTracking.end_and_output2file();
        flgTrackingStarts = true;
	cout << "done." << endl << flush;
    }
    cout << "Exitting track()." << endl << flush;

    return ret;
}
