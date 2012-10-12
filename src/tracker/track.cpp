#include <numeric>

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "../humantracking.h"
#include "track.h"
#include "TrackingProcessLogger.h"
#include "Viewer.h"

using namespace std;
using namespace cv;

extern TrackingProcessLogger logTracking;
extern Viewer viewer;

PARAM_COMMON commonParam; // ‹¤’Êƒpƒ‰ƒ[ƒ^
PARAM_EXTRACTLUM extractlumParam; // “™‘¬’¼ü‰^“®’Šoƒpƒ‰ƒ[ƒ^
PARAM_MKTRAJECTORY mktrajectoryParam; // ‹OÕì¬ƒpƒ‰ƒ[ƒ^
PARAM_CLUSTERING clusteringParam; // ƒNƒ‰ƒXƒ^ƒŠƒ“ƒOƒpƒ‰ƒ[ƒ^
//PARAM_MAKERESULT makeresultParam; // ’ÇÕŒ‹‰Êì¬ƒpƒ‰ƒ[ƒ^
PARAM_RENOVATE_TRAJECTORY rnvtrjParam; // ‹OÕC•œƒpƒ‰ƒ[ƒ^
PARAM_PLOT plotParam; // ŒvZ‰ß’öƒvƒƒbƒgƒpƒ‰ƒ[ƒ^

bool flgFirst;

extern float roi_width, roi_height;
extern float roi_x, roi_y;
extern bool flgOutputTrackingProcessData2Files;

bool load_track_parameters( std::string strPath, std::string strFileName )
{
    commonParam.termTracking = 10000000;//8500000;//10000000;
    commonParam.intervalTracking = 7000000;//7000000;
    commonParam.intervalTrajectory = 100000;
    extractlumParam.term = 1500000;
    extractlumParam.interval = 500000;
    extractlumParam.minPEPMapValue = 50.0;
    extractlumParam.maxPEPMapValue = 70.0;//1500.0;
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
    mktrajectoryParam.densityOrigin = 60.0;
    clusteringParam.thConnect = 0.19;//0.18;//0.2;
    clusteringParam.thDistance = 0.7;
    clusteringParam.minLength = 1000000;
    clusteringParam.distanceLimit = clusteringParam.thDistance;
    clusteringParam.nLimit = 1;
    clusteringParam.minCommonTimeRange = 250000;
    clusteringParam.distVerifyCluster = 0.28;
    rnvtrjParam.lambda1 = 10.0;
    rnvtrjParam.lambda2 = 200.0;
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
    plotParam.kSample = 1.0e-2;//1.0e-3;


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
    // tkiƒL[j‚É‘Î‰‚·‚éLUMW‡‚Ö‚Ìƒ}ƒbƒv
    // tk‚É‘Î‰‚·‚é“™‘¬’¼ü‰^“®W‡‚Æ‚ÍA[tk - extractlumParam.term, tk)‚ÌPEPMap‚©‚çì¬‚µ‚½“™‘¬’¼ü‰^“®W‡‚Ì‚±‚Æ‚Å‚ ‚éB
    static LUMStorage storageLUM;

    // tableLUMSlice
    // LUMƒXƒ‰ƒCƒXƒe[ƒuƒ‹
    // tkiƒL[j‚É‚¨‚¯‚éLUMƒXƒ‰ƒCƒX‚Ì”z—ñ‚Ö‚Ìƒ}ƒbƒv
    static LUMSliceTable tableLUMSlice;

    // storageTrajectory
    // ì¬’†‚Ì‹OÕ—v‘f
    static vector<TrajectoryElement> storageTrajectoryElement;

    // resultTrajectory
    // ’ÇÕŒ‹‰ÊiID‚Æ‹OÕ‚Ìƒ}ƒbƒvj
    static map<int,CTrajectory> resultTrajectory;

    //static vector<CTrajectory> prevTrajectoriesClustered;
    static map<unsigned long long, std::multimap<int,cv::Point2d> >  remainedExtendedResult;

    // idNext
    // ’ÇÕŒ‹‰ÊiID‚Æ‹OÕ‚Ìƒ}ƒbƒvj
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
        viewer.SetStartTime( timeEarliestPEPMap );
    }

    // debug code
    //time_stamp = time_stamp - timeEarliestPEPMap + 1000000000;

    if( flgTrackingStarts ) {
        logTracking.set_tracking_block( timeTracking - commonParam.termTracking, timeTracking );
        logTracking.start();
        viewer.SetTrackingStatus( 0 );
        viewer.SetTrackingBlock( timeTracking - commonParam.termTracking, timeTracking );
        flgTrackingStarts = false;
    }
    
    time_of_received_pepmap.insert( time_stamp );

    logTracking.making_trajectory( TrackingProcessLogger::Start );

    // PEPMap‚ğƒTƒ“ƒvƒ‰‚É’Ç‰Á‚·‚é
    AddPEPMapToSampler( occupancy
                      , time_stamp
                        , &sampler
                        , extractlumParam.minPEPMapValue
                        , extractlumParam.maxPEPMapValue );

    //
    // LUM’Šo
    vector<TIME_MICRO_SEC> addedTime;
    for( TIME_MICRO_SEC tk = timeTracking - commonParam.termTracking; tk <= time_stamp; tk += extractlumParam.interval ) {
        if( storageLUM.find( tk ) == storageLUM.end() ) {
            // tk‚ÌLUM‚ğ’Šo
            ExtractLUM( &sampler
                        , tk
                        , &storageLUM[ tk ]
                        , &commonParam
                        , &extractlumParam
                        , true );
            addedTime.push_back( tk );

        }
    }

    //
    //  LUMƒXƒ‰ƒCƒXƒe[ƒuƒ‹ì¬
    if( !storageLUM.empty() ) {
        TIME_MICRO_SEC timeLatestLUM = storageLUM.rbegin()->first;
        addedTime.clear();
        // timeLatestLUM‚Ü‚Å‚ÌLUM‚ª“¾‚ç‚ê‚Ä‚¢‚é‚Æ‚«CLUMƒXƒ‰ƒCƒXƒe[ƒuƒ‹‚ªì¬‚Å‚«‚é‚Ì‚Í
        // (timeLatestLUM - extractlumParam.term)‚Ü‚ÅB‚±‚Ì”ÍˆÍ‚ÅLUMƒXƒ‰ƒCƒXƒe[ƒuƒ‹‚ğì¬‚·‚éB
        for( TIME_MICRO_SEC tk = timeTracking - commonParam.termTracking
            ; tk <= timeLatestLUM - extractlumParam.term
            ; tk += commonParam.intervalTrajectory ) {
            if( tableLUMSlice.find( tk ) == tableLUMSlice.end() ) {
                // tk‚ÌLUMƒXƒ‰ƒCƒXì¬
                //cerr << "LUMƒXƒ‰ƒCƒX’Ç‰Á: ";
                MakeLUMSlice( tk, &storageLUM, &tableLUMSlice[ tk ], &extractlumParam );
                addedTime.push_back( tk );
            }
        }
    }

    // tableLUMSlice‚É’Ç‰Á‚³‚ê‚½Še‚Én“_‚ğ’è‚ß‚éB
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

    // MPI‚É‚æ‚èn“_‚ğŠeƒvƒƒZƒX‚ÉŠ„‚èU‚é
    // ˆÈ‰ºC‰¼‹Lq
    vector<PosXYT> originPosPerProcess;
    originPosPerProcess.assign( originPos.begin(), originPos.end() );

    //
    // ó‚¯æ‚Á‚½n“_‚ğŠî‚ÉV‚½‚È‹OÕ‚ğ¶¬‚·‚é
    int nNewTrj = 0;
    vector<PosXYT>::iterator itOrigin = originPosPerProcess.begin();
    for( int i = 0; itOrigin != originPosPerProcess.end(); ++itOrigin, ++i ) {
        TrajectoryElement trjElement;
        trjElement.insert( *itOrigin );
        storageTrajectoryElement.push_back( trjElement );
        ++nNewTrj;
    }

    //
    // tableLUMSlice‚É’Ç‰Á‚³‚ê‚½Še‚É‚Â‚¢‚Ä‹OÕ‚ğ‰„’·‚·‚é
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

        if( flgOutputTrackingProcessData2Files ) {
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
            cerr << "Š®—¹(nSample=" << nSample << ")" << endl;

            // debug code
            if( nSample <= 3 ) {
                cerr << "nSample is no more than 3." << endl;
            }
        }

        //
        // Clustering the trajectories
        //
        logTracking.clustering( TrackingProcessLogger::Start );
        viewer.SetTrackingStatus( 1 );

        cerr << "Calculating a distance table..." << endl;

        //
        // ƒNƒ‰ƒXƒ^ƒŠƒ“ƒO‚É—p‚¢‚é‹OÕi’·‚³‚ªclusterigParam.minLengthˆÈãj‚ğæ‚èo‚·
        vector<TrajectoryElement> trajectoryElementOfMyProc;
        vector<TrajectoryElement>::iterator it = storageTrajectoryElement.begin();
        for( ; it != storageTrajectoryElement.end(); ++it ) {
            if( it->rbegin()->t - it->begin()->t >= clusteringParam.minLength ) {
                trajectoryElementOfMyProc.push_back( *it );
            }
        }

        size_t nAllTrj; // ‘‹OÕ”
        nAllTrj = trajectoryElementOfMyProc.size();
        map<int,CTrajectory> trajectoryForClustering;
        int iTrj = 0;
        for( vector<TrajectoryElement>::iterator it = trajectoryElementOfMyProc.begin(); it != trajectoryElementOfMyProc.end(); ++it ) {
            trajectoryForClustering[ iTrj ].push_back( *it );
            //(*pRecv)[ iTrj ].push_back( *it );
            ++iTrj;
        }


        // ‹——£ƒe[ƒuƒ‹‚Ì‰Šú‰»
        double* distTable = new double[ nAllTrj * nAllTrj ];
        for( size_t i = 0; i < nAllTrj * nAllTrj; ++i ) {
            distTable[ i ] = -2.0;
        }
        CTrajectory_Distance distanceTrajectory( clusteringParam.distanceLimit, clusteringParam.nLimit, clusteringParam.minCommonTimeRange );
        vector<size_t> iTrjToCol( nAllTrj ); // ŠeƒvƒƒZƒX‚Ì‹——£ƒe[ƒuƒ‹‚É‚¨‚¢‚Ä—ñ”Ô†‚Æ‹OÕ”Ô†‚Ì‘Î‰‚ğ¦‚µ‚½‚à‚Ì
        for( size_t i = 0; i < nAllTrj; ++i ) {
            iTrjToCol[ i ] = i;
        }
        CalculateDistanceTable( distTable
                              , &iTrjToCol
                              , nAllTrj
                              , trajectoryForClustering.begin()
                              , trajectoryForClustering.end()
                              , &trajectoryForClustering
                              , distanceTrajectory );


        // ƒNƒ‰ƒXƒ^ƒŠƒ“ƒO
        vector<CTrajectory> trajectoriesClustered;

        // ‰ŠúƒNƒ‰ƒXƒ^‚Ìî•ñ‚ğCóM‚µ‚½‹OÕˆê‚Â‚¸‚Â‚©‚ç¬‚éƒNƒ‰ƒXƒ^‚ª¶¬‚³‚ê‚é‚æ‚¤€”õ‚·‚éB
        vector< vector<int> > indexCluster;
        vector<int> classID( nAllTrj, -1 );
        for( int i = 0; i < (int)nAllTrj; ++i ) {
            indexCluster.push_back( vector<int>( 1, i ) );
        }

        double* dist;

        size_t nCluster;// = indexCluster.size();
        size_t prevNumOfCluster;// = nCluster;
        int cnt_loop = 0;
        do {
            vector< vector<int> > tmpIndexCluster;
            trajectoriesClustered.clear();
            for( vector< vector<int> >::iterator itCluster = indexCluster.begin(); itCluster != indexCluster.end(); ++itCluster ) {
                CTrajectory trj;
                for( vector<int>::iterator itIdxTrj = itCluster->begin(); itIdxTrj != itCluster->end(); ++itIdxTrj ) {
                    map<int,CTrajectory>::iterator itTrj;
                    if( ( itTrj = trajectoryForClustering.find( *itIdxTrj ) ) != trajectoryForClustering.end() ) {
                        trj.insert( trj.end(), itTrj->second.begin(), itTrj->second.end() );
                    } else {
                        cerr << "Trajectory no." << *itIdxTrj << " Not Found(" << nAllTrj << ")." << endl;
                        //exit( 1 );
                    }
                }
                if( !trj.empty() && VerifyClusteredTrajectories( trj, clusteringParam.distVerifyCluster ) ) {
                    trajectoriesClustered.push_back( trj );
                    tmpIndexCluster.push_back( *itCluster );
                }
            }

            // ‹——£ƒe[ƒuƒ‹”z’u
            nCluster = trajectoriesClustered.size();
            dist = new double[ nCluster * nCluster ];

            //cerr << "ÄƒNƒ‰ƒXƒ^ƒŠƒ“ƒOiŒ»İ‚ÌƒNƒ‰ƒXƒ^F" << nCluster << "[$B8D(B], $BMxMQ50@W!'(B" << usetrj.size() << "[$BK\(B]$B!K(B...";
            cerr << cnt_loop << "‰ñ–Ú...";

            vector<CTrajectory> tmpTrajectoriesClustered( trajectoriesClustered.size() );
            vector<CTrajectory>::iterator itTmpTrj = tmpTrajectoriesClustered.begin();
            for( vector<CTrajectory>::iterator itTrj = trajectoriesClustered.begin(); itTrj != trajectoriesClustered.end(); ++itTrj, ++itTmpTrj ) {
                itTrj->Integrate( &(*itTmpTrj) );
                //cerr << "itTrj->front().size():" << itTrj->front().size() << ", itTmpTrj->front().size():" << itTmpTrj->front().size() << endl;
                //if( itTmpTrj->front().size() == 0 ) {
                //    exit( 1 );
                //}
            }

            for( size_t idx1 = 0; idx1 < tmpTrajectoriesClustered.size(); ++idx1 ) {
                for( size_t idx2 = idx1; idx2 < tmpTrajectoriesClustered.size(); ++idx2 ) {
                    size_t index1 = idx1 * nCluster + idx2;
                    size_t index2 = idx2 * nCluster + idx1;
                    //cerr << "nCluster = " << nCluster << endl;
                    //cerr << "idx1 = " << idx1 << ", $BMWAG?t(B = " << tmpTrajectoriesClustered[ idx1 ].size() << endl;
                    //cerr << "idx2 = " << idx2 << ", $BMWAG?t(B = " << tmpTrajectoriesClustered[ idx2 ].size() << endl;
                    if( false/*cnt_loop == 0*/ ) {
                        // $B:G=i$N%/%i%9%?%j%s%0$N$H$-$O@h$K5a$a$?5wN%%F!<%V%k$r;HMQ$9$k!#(B
                        dist[ index1 ] = dist[ index2 ] = distTable[ idx1 * nAllTrj + idx2 ];
                    } else {
                        dist[ index1 ] = dist[ index2 ] = distanceTrajectory( tmpTrajectoriesClustered[ idx1 ]
                                                                            , tmpTrajectoriesClustered[ idx2 ] );
                    }
                    //TrajectoryElement_Distance distanceTrajectoryElement( clusteringParam.distanceLimit, clusteringParam.nLimit, clusteringParam.minCommonTimeRange );
                    //cerr << "idx1:" << tmpTrajectoriesClustered[ idx1 ].front().size() << ", idx2:" << tmpTrajectoriesClustered[ idx2 ].front().size() << ", ";
                    //cerr << "$B5wN%(B = ";
                    //dist[ index1 ] = dist[ index2 ] = distanceTrajectoryElement( tmpTrajectoriesClustered[ idx1 ].front()
                    //                                                           , tmpTrajectoriesClustered[ idx2 ].front() );
                    //cerr << dist[ index1 ] << endl;
                }
            }

#if 1

            // ƒNƒ‰ƒXƒ^‚ğŠÔˆø‚­
            cerr << "—˜—pƒNƒ‰ƒXƒ^‚Ì‘I’è...";
            vector<int> idxClusterUse;
            if( !tmpTrajectoriesClustered.empty() ) {
                vector<double> frequency( tmpTrajectoriesClustered.size(), 0.0 );
                CalcFrequency( (double*)&(frequency[ 0 ]), dist, tmpTrajectoriesClustered.size(), 0.1, clusteringParam.thDistance );
                ReduceTrajectory( &idxClusterUse, (double*)&(frequency[0]), frequency.size(), 55.0/*80.0*/ );
            }
            cerr << "Š®—¹i" << idxClusterUse.size() << "[ŒÂ]j...";

            // ‹——£ƒe[ƒuƒ‹Ä”z’u
            double* dist2 = new double[ idxClusterUse.size() * idxClusterUse.size() ];
            for( vector<int>::iterator itIdxCluster = idxClusterUse.begin(); itIdxCluster != idxClusterUse.end(); ++itIdxCluster ) {
                for( vector<int>::iterator it = idxClusterUse.begin(); it != idxClusterUse.end(); ++it ) {
                    size_t idxDst = distance( idxClusterUse.begin(), itIdxCluster ) * idxClusterUse.size()
                                    + distance( idxClusterUse.begin(), it );
                    size_t idxSrc = *itIdxCluster * nCluster + *it;
                    dist2[ idxDst ] = dist[ idxSrc ];
                }
            }

            vector<int> classID( idxClusterUse.size(), -1 );
            Clustering( &classID, dist2, idxClusterUse.size(), clusteringParam.distVerifyCluster );

            vector< vector<int> > tmp;
            Clustering2( &tmp, classID, dist2, idxClusterUse.size(), clusteringParam.thConnect, clusteringParam.thDistance );

            delete [] dist2;


            indexCluster.clear();
            for( vector< vector<int> >::iterator itClusterIdx = tmp.begin(); itClusterIdx != tmp.end(); ++itClusterIdx ) {
                vector<int> clusterIdx;
                for( vector<int>::iterator it = itClusterIdx->begin(); it != itClusterIdx->end(); ++it ) {
                    clusterIdx.insert( clusterIdx.end()
                                        , tmpIndexCluster[ idxClusterUse[ *it ] ].begin()
                                        , tmpIndexCluster[ idxClusterUse[ *it ] ].end() );
                }
                sort( clusterIdx.begin(), clusterIdx.end() );
                clusterIdx.erase( unique( clusterIdx.begin(), clusterIdx.end() ), clusterIdx.end() );
                indexCluster.push_back( clusterIdx );
            }

            prevNumOfCluster = nCluster;//idxClusterUse.size();// nCluster;
            nCluster = indexCluster.size();
            cerr << "I—¹iƒNƒ‰ƒXƒ^F" << nCluster << "[ŒÂ], trajectoriesClustered.size()=" << trajectoriesClustered.size() << "j";
            cerr << endl;

            //
            // Output process information of clustering
            if( flgOutputTrackingProcessData2Files ) {
                ++cnt_loop;
                if( trajectoriesClustered.size() < 30 ) {
                    ostringstream oss;
                    oss << cnt_loop;
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
			            , "/home/kumalab/project/HumanTracking/bin/tmp_trajectories/"
#endif
                                    , oss.str()
                                    , NULL
                                    , &plotParam );
                }
                
            }
#endif
            delete [] dist;
        } while( prevNumOfCluster != nCluster );

        logTracking.clustering( TrackingProcessLogger::End );

        logTracking.renovation( TrackingProcessLogger::Start );
        viewer.SetTrackingStatus( 2 );

        //
        // Renovate the trajectories
        TrajectoriesInfo infoTrj;
        infoTrj.section.resize( 1 );

        cerr << "Started renovation: [ " << timeTracking - commonParam.termTracking - timeEarliestPEPMap
                << ", " <<  timeTracking - timeEarliestPEPMap << " )" << endl;

        // ƒNƒ‰ƒXƒ^ƒŠƒ“ƒO‚µ‚½‹OÕ‚ğ•½‹Ï‚µ‚ÄinfoTrj‚ÉŠi”[‚·‚é
        infoTrj.trjElement.resize( trajectoriesClustered.size() );
        for( int i = 0; i < (int)trajectoriesClustered.size(); ++i ) {
            CTrajectory trj;
            trajectoriesClustered[ i ].Integrate( &trj );
            infoTrj.trjElement[ i ] = trj.front();
        }

//        if( myRank == 0 ) {
//            cout << "@ƒNƒ‰ƒXƒ^ƒŠƒ“ƒO‚µ‚½‹OÕ‚ğ’Ç‰Á: ‘Œv" << infoTrj.trjElement.size() << "[ŒÂ]" << endl;
//        }

        // ‘O‰ñ‚Ì’ÇÕŒ‹‰Ê(resultTrajectory)‚ğ[ timeTracking - commonParam.termTracking, timeTracking )‚Å
        // ƒNƒŠƒbƒv‚µ‚ÄinfoTrj‚ÉŠi”[‚·‚é
        int idx = (int)trajectoriesClustered.size();
        map<int,int> reserve; // ‹OÕ”Ô†‚ÆŠù‘¶‚ÌID‚Ì‘g‚İ‡‚í‚¹
        for( map<int,CTrajectory>::iterator itResult = resultTrajectory.begin(); itResult != resultTrajectory.end(); ++itResult ) {
            CTrajectory trj;
            trj = itResult->second;
            trj.Clip( timeTracking - commonParam.termTracking, timeTracking );
            if( !trj.empty() ) {
                infoTrj.trjElement.push_back( trj.front() );
                reserve[ idx ] = itResult->first;
                ++idx;
            }
        }

//        if( myRank == 0 ) {
//            cout << "@‘O‰ñ‚Ì’ÇÕŒ‹‰Ê‚ğ’Ç‰Á: ‘Œv" << infoTrj.trjElement.size() << "[ŒÂ]" << endl;
//        }

        //
        // ‹OÕ‚Ìo—Í
//            if( myRank == 0 ) {
//                vector<CTrajectory> vectrj;
//                for( vector<TrajectoryElement>::iterator it = infoTrj.trjElement.begin(); it != infoTrj.trjElement.end(); ++it ) {
//                    CTrajectory trj;
//                    trj.push_back( *it );
//                    vectrj.push_back( trj );
//                }
//                double sumValue = accumulate( sampler.begin(), sampler.end(), 0.0, PosXYTV_Sum() );
//                unsigned int nSample = (unsigned int)( 1.04e-4 * sumValue );
//                OutputProcess( timeTracking - commonParam.termTracking//tableLUMSlice.begin()->first
//                             , timeTracking//tableLUMSlice.rbegin()->first
//                             , timeEarliestPEPMap
//                             , &sampler
//                             , nSample
//                             , &vectrj
//                             , "./tmp_preoptimize/"
//                             , ""
//                             , NULL
//                             , &plotParam );
//            }

        // ƒZƒNƒVƒ‡ƒ“•ªŠ„
        map<int,int> pointIdxToTrjNo;
        DivideIntoSections( &infoTrj, &pointIdxToTrjNo, rnvtrjParam );

        // ŠeƒZƒNƒVƒ‡ƒ“‚ÅƒZƒbƒg‚ğì¬
        for( int i = 0; i < (int)infoTrj.section.size(); ++i ) {
            MakeSet( i, &infoTrj, &reserve );
//            cout << "@@ƒZƒNƒVƒ‡ƒ“" << i << ": " << infoTrj.section[ i ].trjSet.size() << "[ŒÂ]" << endl;
        }

//        start_d = MPI_Wtime();
//        cout << "Optimize()...";
        // Å“K‰ğ‚Ì’Tõ
        vector<TrajectoryElement> opt;
        vector<int> idOpt;
        double min = -1.0;
        //if( myRank == 0 ) {
        cerr << "  Optimum analysis: # of sections" << (int)infoTrj.section.size() << endl;
        //}
        for( int idxSec = 0; idxSec < (int)infoTrj.section.size(); ++idxSec ) {
            min = -1.0;
            int idxMinSet = -1;
            vector<TrajectoryElement> optOfSec;
            vector<int> idOptOfSec;
            for( int idxSet = 0; idxSet < (int)infoTrj.section[ idxSec ].trjSet.size(); ++idxSet ) {
                vector<TrajectoryElement> trjElement;
                vector<int> idTrjElement;
                double t1 = 0.0, t2 = 0.0, t3 = 0.0;
                double e = Optimize( &trjElement, &idTrjElement, &reserve, idxSec, idxSet, 20, &infoTrj, &pointIdxToTrjNo, rnvtrjParam, &t1, &t2, &t3 );
                if( e >= 0.0 && ( min < 0.0 || e < min ) ) {
                    optOfSec = trjElement;
                    idOptOfSec = idTrjElement;
                    idxMinSet = idxSet;
                    min = e;
                }
            }
            cerr << " Optimization result: Set" << idxMinSet << endl;
            cerr << "    # of trajectories " << optOfSec.size() << endl;

            opt.insert( opt.end(), optOfSec.begin(), optOfSec.end() );
            idOpt.insert( idOpt.end(), idOptOfSec.begin(), idOptOfSec.end() );
        }

//#define RE_RENOVATE
#ifdef RE_RENOVATE
        //
        // $B:F=$I|$r9T$&(B
        //if( myRank == 0 ) {
            cerr << "  Re-renovation";
        //}

        // $B5a$a$?:GE,2r$r%;%C%H(B
        infoTrj.trjElement = opt;

        // $BM=Ls50@W$r5a$a$k(B
        reserve.clear();
        for( int idx = 0; idx < (int)idOpt.size(); ++idx ) {
            if( idOpt[ idx ] != -1 ) {
                reserve[ idx ] = idOpt[ idx ];
            }
        }

        // $B%;%/%7%g%sJ,3d(B
        DivideIntoSections( &infoTrj, rnvtrjParam );

        // $B3F%;%/%7%g%s$G%;%C%H$r:n@.(B
        for( int i = 0; i < (int)infoTrj.section.size(); ++i ) {
            MakeSet( i, &infoTrj, &reserve );
        }

        // $B:GE,2r$NC5:w(B
        opt.clear();
        idOpt.clear();
        min = -1.0;
        for( int idxSec = 0; idxSec < (int)infoTrj.section.size(); ++idxSec ) {
            min = -1.0;
            int idxMinSet = -1;
            vector<TrajectoryElement> optOfSec;
            vector<int> idOptOfSec;
            for( int idxSet = 0; idxSet < (int)infoTrj.section[ idxSec ].trjSet.size(); ++idxSet ) {
                vector<TrajectoryElement> trjElement;
                vector<int> idTrjElement;
                double t1 = 0.0, t2 = 0.0, t3 = 0.0;
                double e = Optimize( &trjElement, &idTrjElement, &reserve, idxSec, idxSet, 100/*20*/, &infoTrj, rnvtrjParam, &t1, &t2, &t3 );
                if( e >= 0.0 && ( min < 0.0 || e < min ) ) {
                    optOfSec = trjElement;
                    idOptOfSec = idTrjElement;
                    idxMinSet = idxSet;
                    min = e;
                }
            }

            opt.insert( opt.end(), optOfSec.begin(), optOfSec.end() );
            idOpt.insert( idOpt.end(), idOptOfSec.begin(), idOptOfSec.end() );
        }

        //if( myRank == 0 ) {
            cerr << "  Done..." << endl;
        //}
#endif

	    logTracking.renovation( TrackingProcessLogger::End );
        viewer.SetTrackingStatus( 3 );

        logTracking.finishing( TrackingProcessLogger::Start );

        // ID–¢Š„‚è“–‚Ä‚Ì‹OÕ‚ÉV‚µ‚¢ID‚ğU‚é
        for( vector<int>::iterator itID = idOpt.begin(); itID != idOpt.end(); ++itID ) {
            if( *itID == -1 ) {
                *itID = idNext;
                ++idNext;
            }
        }

        // ‹OÕ‚Ì•âŠÔ‚ğs‚¤
        for( vector<TrajectoryElement>::iterator itTrj = opt.begin(); itTrj != opt.end(); ++itTrj ) {
            TrajectoryElement::iterator it = itTrj->begin();
            TrajectoryElement::iterator itNext = it;
            advance( itNext, 1 );
            while( itNext != itTrj->end() ) {
                while( it->t + commonParam.intervalTrajectory < itNext->t ) {
                    PosXYT pos;
                    pos.x = ( itNext->x - it->x ) / ( (double)( itNext->t - it->t ) * 1.0e-6 ) * ( (double)commonParam.intervalTrajectory * 1.0e-6 ) + it->x;
                    pos.y = ( itNext->y - it->y ) / ( (double)( itNext->t - it->t ) * 1.0e-6 ) * ( (double)commonParam.intervalTrajectory * 1.0e-6 ) + it->y;
                    pos.t = it->t + commonParam.intervalTrajectory;
                    it = itTrj->insert( it, pos );
                }
                ++it;
                ++itNext;
            }
        }

        // Œ‹‰Ê‚Ì•Û‘¶
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

        //map<unsigned long long, multimap<int,Point2d> > ext_result;

        unsigned long long time;
        p_ext_result->insert( remainedExtendedResult.begin(), remainedExtendedResult.end() );
        for( /*unsigned long long*/ time = max( timeTracking - commonParam.termTracking, timeEarliestPEPMap )
                ; time < timeTracking - ( commonParam.termTracking - commonParam.intervalTracking )
                ; time += commonParam.intervalTrajectory ) {

            (*p_result)[ time ];
            (*p_ext_result)[ time ];

            const set<TIME_MICRO_SEC>::iterator it_start_pepmap_time = time_of_received_pepmap.lower_bound( time );
            const set<TIME_MICRO_SEC>::iterator it_end_pepmap_time = time_of_received_pepmap.lower_bound( time + commonParam.intervalTrajectory );
            for( set<TIME_MICRO_SEC>::iterator it = it_start_pepmap_time; it != it_end_pepmap_time; ++it ) {
               (*p_ext_result)[ *it ]; 
            }

            map<int,CTrajectory>::const_iterator itResult = resultTrajectory.begin();
            for( ; itResult != resultTrajectory.end(); ++itResult ) {
                const CTrajectory& trajectory = itResult->second;
                if( trajectory.size() > 0 ) {
                    TrajectoryElement::iterator itPos = trajectory.front().begin();
                    for( ; itPos != trajectory.front().end(); ++itPos ) {
                        if( itPos->t == time ) {
                            (*p_result)[ time ][ itResult->first ] = Point2d( itPos->x, itPos->y );

                            const int trj_no = itPos->ID;
                            if( trj_no < trajectoriesClustered.size() ) {
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
                                                (*p_ext_result)[ *it ].insert( pair<int,Point2d>( itResult->first, Point2d( pos.x, pos.y ) ) );
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

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
        }


        viewer.SetResult( *p_result );

        //
        // Output process information of renovation
        //if( myRank == 0 ) {
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

        delete [] distTable;

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
    }

    return ret;
}
