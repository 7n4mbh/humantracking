#include <numeric>

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "track.h"


using namespace std;
using namespace cv;

PARAM_COMMON commonParam; // 共通パラメータ
PARAM_EXTRACTLUM extractlumParam; // 等速直線運動抽出パラメータ
PARAM_MKTRAJECTORY mktrajectoryParam; // 軌跡作成パラメータ
PARAM_CLUSTERING clusteringParam; // クラスタリングパラメータ
//PARAM_MAKERESULT makeresultParam; // 追跡結果作成パラメータ
PARAM_RENOVATE_TRAJECTORY rnvtrjParam; // 軌跡修復パラメータ
PARAM_PLOT plotParam; // 計算過程プロットパラメータ

bool flgFirst;

extern float roi_width, roi_height;
extern float roi_x, roi_y;

bool load_track_parameters( std::string strPath )
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
    extractlumParam.kLUM = 6.0e-1;//3.5e-1;//3.0e-2;
    extractlumParam.kVerifySample = 3.0e-1;//3.5e-1;//6.0e-2;
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
    plotParam.kSample = 1.0e-3;

    return true;
}

void initialize_tracker()
{
    flgFirst = true;
}

bool track( const Mat& occupancy, unsigned long long time_stamp )
{
    static TIME_MICRO_SEC timeTracking; // 追跡時刻[usec]
                                 // [timeTracking - commonParam.termTracking, timeTrackig) の範囲で追跡処理を行う事を意味する

    static SamplerPosXYTVID sampler; // 特徴量サンプラ

    // storageLUM
    // 時刻tk（キー）に対応するLUM集合へのマップ
    // 時刻tkに対応する等速直線運動集合とは、[tk - extractlumParam.term, tk)のPEPMapから作成した等速直線運動集合のことである。
    static LUMStorage storageLUM;

    // tableLUMSlice
    // LUMスライステーブル
    // 時刻tk（キー）におけるLUMスライスの配列へのマップ
    static LUMSliceTable tableLUMSlice;

    // storageTrajectory
    // 作成中の軌跡要素
    static vector<TrajectoryElement> storageTrajectoryElement;

    // resultTrajectory
    // 追跡結果（IDと軌跡のマップ）
    static map<int,CTrajectory> resultTrajectory;

    // idNext
    // 次に割り振るべきID番号
    static int idNext;

    static TIME_MICRO_SEC timeEarliestPEPMap;

    if( flgFirst ) {
        sampler.clear();
        storageLUM.clear();
        tableLUMSlice.clear();
        storageTrajectoryElement.clear();
        resultTrajectory.clear();
        idNext = 1;
        timeEarliestPEPMap = time_stamp;
        timeTracking = timeEarliestPEPMap + commonParam.termTracking;
        //timeTracking = commonParam.termTracking + 1000000000; // debug code
        flgFirst = false;
    }

    // debug code
    //time_stamp = time_stamp - timeEarliestPEPMap + 1000000000;
    
    // PEPMapをサンプラに追加する
    AddPEPMapToSampler( occupancy
                      , time_stamp
                        , &sampler
                        , extractlumParam.minPEPMapValue
                        , extractlumParam.maxPEPMapValue );

    //
    // LUM抽出
    vector<TIME_MICRO_SEC> addedTime;
    for( TIME_MICRO_SEC tk = timeTracking - commonParam.termTracking; tk <= time_stamp; tk += extractlumParam.interval ) {
        if( storageLUM.find( tk ) == storageLUM.end() ) {
            // 時刻tkのLUMを抽出
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
    // LUMスライステーブル作成
    if( !storageLUM.empty() ) {
        TIME_MICRO_SEC timeLatestLUM = storageLUM.rbegin()->first;
        addedTime.clear();
        // timeLatestLUMまでのLUMが得られているとき，LUMスライステーブルが作成できるのは
        // (timeLatestLUM - extractlumParam.term)まで。この範囲でLUMスライステーブルを作成する。
        for( TIME_MICRO_SEC tk = timeTracking - commonParam.termTracking
            ; tk <= timeLatestLUM - extractlumParam.term
            ; tk += commonParam.intervalTrajectory ) {
            if( tableLUMSlice.find( tk ) == tableLUMSlice.end() ) {
                // 時刻tkのLUMスライス作成
                //cerr << "LUMスライス追加: ";
                MakeLUMSlice( tk, &storageLUM, &tableLUMSlice[ tk ], &extractlumParam );
                addedTime.push_back( tk );
            }
        }
    }

    // tableLUMSliceに追加された各時刻に始点を定める。
    set<PosXYT,PosXYT_XYT_Less> originPos;
    const double intersticeOrigin = 1.0 / sqrt( mktrajectoryParam.densityOrigin ); // 軌跡の始点同士の間隔
    const double rangeOrigin = mktrajectoryParam.distanceImpact * 2.0; // 軌跡の情報X,Y座標の周りrangeOrigin四方の範囲に始点の候補を作成する
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

    // MPIにより始点を各プロセスに割り振る
    // 以下，仮記述
    vector<PosXYT> originPosPerProcess;
    originPosPerProcess.assign( originPos.begin(), originPos.end() );

    //
    // 受け取った始点を基に新たな軌跡を生成する
    int nNewTrj = 0;
    vector<PosXYT>::iterator itOrigin = originPosPerProcess.begin();
    for( int i = 0; itOrigin != originPosPerProcess.end(); ++itOrigin, ++i ) {
        TrajectoryElement trjElement;
        trjElement.insert( *itOrigin );
        storageTrajectoryElement.push_back( trjElement );
        ++nNewTrj;
    }

    //
    // tableLUMSliceに追加された各時刻について軌跡を延長する
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

    if( !tableLUMSlice.empty() && tableLUMSlice.rbegin()->first >= timeTracking ) {
        cout << "Done with making trajectories." << endl;

        // 計算過程をプロット
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
        cerr << "完了(nSample=" << nSample << ")" << endl;


        //
        // 軌跡のクラスタリング
        //
        cerr << "軌跡間の距離テーブル計算中..." << endl;

        //
        // クラスタリングに用いる軌跡（長さがclusterigParam.minLength以上）を取り出す
        vector<TrajectoryElement> trajectoryElementOfMyProc;
        vector<TrajectoryElement>::iterator it = storageTrajectoryElement.begin();
        for( ; it != storageTrajectoryElement.end(); ++it ) {
            if( it->rbegin()->t - it->begin()->t >= clusteringParam.minLength ) {
                trajectoryElementOfMyProc.push_back( *it );
            }
        }

        size_t nAllTrj; // 総軌跡数
        nAllTrj = trajectoryElementOfMyProc.size();
        map<int,CTrajectory> trajectoryForClustering;
        int iTrj = 0;
        for( vector<TrajectoryElement>::iterator it = trajectoryElementOfMyProc.begin(); it != trajectoryElementOfMyProc.end(); ++it ) {
            trajectoryForClustering[ iTrj ].push_back( *it );
            //(*pRecv)[ iTrj ].push_back( *it );
            ++iTrj;
        }


        // 距離テーブルを作成
        double* distTable = new double[ nAllTrj * nAllTrj ];
        for( size_t i = 0; i < nAllTrj * nAllTrj; ++i ) {
            distTable[ i ] = -2.0;
        }
        CTrajectory_Distance distanceTrajectory( clusteringParam.distanceLimit, clusteringParam.nLimit, clusteringParam.minCommonTimeRange );
        vector<size_t> iTrjToCol( nAllTrj ); // 自プロセスの距離テーブルにおいて軌跡番号と列番号の対応を示したもの
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


        // クラスタリング
        vector<CTrajectory> trajectoriesClustered;

        // 初期クラスタの情報を，受信した軌跡一つずつから成るクラスタが生成されるよう準備する。
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

            // 距離テーブル配置
            nCluster = trajectoriesClustered.size();
            dist = new double[ nCluster * nCluster ];

            //cerr << "再クラスタリング（現在のクラスタ：" << nCluster << "[個], 利用軌跡：" << usetrj.size() << "[本]）...";
            cerr << cnt_loop << "回目...";

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
                    //cerr << "idx1 = " << idx1 << ", 要素数 = " << tmpTrajectoriesClustered[ idx1 ].size() << endl;
                    //cerr << "idx2 = " << idx2 << ", 要素数 = " << tmpTrajectoriesClustered[ idx2 ].size() << endl;
                    if( false/*cnt_loop == 0*/ ) {
                        // 最初のクラスタリングのときは先に求めた距離テーブルを使用する。
                        dist[ index1 ] = dist[ index2 ] = distTable[ idx1 * nAllTrj + idx2 ];
                    } else {
                        dist[ index1 ] = dist[ index2 ] = distanceTrajectory( tmpTrajectoriesClustered[ idx1 ]
                                                                            , tmpTrajectoriesClustered[ idx2 ] );
                    }
                    //TrajectoryElement_Distance distanceTrajectoryElement( clusteringParam.distanceLimit, clusteringParam.nLimit, clusteringParam.minCommonTimeRange );
                    //cerr << "idx1:" << tmpTrajectoriesClustered[ idx1 ].front().size() << ", idx2:" << tmpTrajectoriesClustered[ idx2 ].front().size() << ", ";
                    //cerr << "距離 = ";
                    //dist[ index1 ] = dist[ index2 ] = distanceTrajectoryElement( tmpTrajectoriesClustered[ idx1 ].front()
                    //                                                           , tmpTrajectoriesClustered[ idx2 ].front() );
                    //cerr << dist[ index1 ] << endl;
                }
            }

#if 1

            // クラスタを間引く
            cerr << "利用クラスタの選定...";
            vector<double> frequency( tmpTrajectoriesClustered.size(), 0.0 );
            CalcFrequency( (double*)&(frequency[ 0 ]), dist, tmpTrajectoriesClustered.size(), 0.1, clusteringParam.thDistance );
            vector<int> idxClusterUse;
            ReduceTrajectory( &idxClusterUse, (double*)&(frequency[0]), frequency.size(), 55.0/*80.0*/ );
            cerr << "完了（" << idxClusterUse.size() << "[個]）...";

            // 距離テーブル再配置
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
            cerr << "終了（クラスタ：" << nCluster << "[個], trajectoriesClustered.size()=" << trajectoriesClustered.size() << "）";
            cerr << endl;

            //
            // 計算結果の出力
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
                                , "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_trajectories\\"
                                , oss.str()
                                , NULL
                                , &plotParam );

            }
#endif
            delete [] dist;
        } while( prevNumOfCluster != nCluster );



        //
        // 軌跡の修復
        TrajectoriesInfo infoTrj;
        infoTrj.section.resize( 1 );

        cerr << "軌跡の修復開始: [ " << timeTracking - commonParam.termTracking - timeEarliestPEPMap
                << ", " <<  timeTracking - timeEarliestPEPMap << " )" << endl;

        // クラスタリングした軌跡を平均してinfoTrjに格納する
        infoTrj.trjElement.resize( trajectoriesClustered.size() );
        for( int i = 0; i < (int)trajectoriesClustered.size(); ++i ) {
            CTrajectory trj;
            trajectoriesClustered[ i ].Integrate( &trj );
            infoTrj.trjElement[ i ] = trj.front();
        }

//        if( myRank == 0 ) {
//            cout << "　クラスタリングした軌跡を追加: 総計" << infoTrj.trjElement.size() << "[個]" << endl;
//        }

        // 前回の追跡結果(resultTrajectory)を[ timeTracking - commonParam.termTracking, timeTracking )で
        // クリップしてinfoTrjに格納する
        int idx = (int)trajectoriesClustered.size();
        map<int,int> reserve; // 軌跡番号と既存のIDの組み合わせ
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
//            cout << "　前回の追跡結果を追加: 総計" << infoTrj.trjElement.size() << "[個]" << endl;
//        }

        //
        // 軌跡の出力
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

        // セクション分割
        DivideIntoSections( &infoTrj, rnvtrjParam );

        // 各セクションでセットを作成
        for( int i = 0; i < (int)infoTrj.section.size(); ++i ) {
            MakeSet( i, &infoTrj, &reserve );
//            cout << "　　セクション" << i << ": " << infoTrj.section[ i ].trjSet.size() << "[個]" << endl;
        }

//        start_d = MPI_Wtime();
//        cout << "Optimize()...";
        // 最適解の探索
        vector<TrajectoryElement> opt;
        vector<int> idOpt;
        double min = -1.0;
        //if( myRank == 0 ) {
            cerr << "　最適解探索: セクション数" << (int)infoTrj.section.size() << endl;
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
                double e = Optimize( &trjElement, &idTrjElement, &reserve, idxSec, idxSet, 20, &infoTrj, rnvtrjParam, &t1, &t2, &t3 );
                if( e >= 0.0 && ( min < 0.0 || e < min ) ) {
                    optOfSec = trjElement;
                    idOptOfSec = idTrjElement;
                    idxMinSet = idxSet;
                    min = e;
                }
            }
            cerr << "　最適化結果: セット" << idxMinSet << endl;
            cerr << "　　軌跡数: " << optOfSec.size() << "[個]" << endl;

            opt.insert( opt.end(), optOfSec.begin(), optOfSec.end() );
            idOpt.insert( idOpt.end(), idOptOfSec.begin(), idOptOfSec.end() );
        }

#define RE_RENOVATE
#ifdef RE_RENOVATE
        //
        // 再修復を行う
        //if( myRank == 0 ) {
            cerr << "　最修復開始...";
        //}

        // 求めた最適解をセット
        infoTrj.trjElement = opt;

        // 予約軌跡を求める
        reserve.clear();
        for( int idx = 0; idx < (int)idOpt.size(); ++idx ) {
            if( idOpt[ idx ] != -1 ) {
                reserve[ idx ] = idOpt[ idx ];
            }
        }

        // セクション分割
        DivideIntoSections( &infoTrj, rnvtrjParam );

        // 各セクションでセットを作成
        for( int i = 0; i < (int)infoTrj.section.size(); ++i ) {
            MakeSet( i, &infoTrj, &reserve );
        }

        // 最適解の探索
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
            cerr << "　完了..." << endl;
        //}
#endif

        // ID未割り当ての軌跡に新しいIDを振る
        for( vector<int>::iterator itID = idOpt.begin(); itID != idOpt.end(); ++itID ) {
            if( *itID == -1 ) {
                *itID = idNext;
                ++idNext;
            }
        }

        // 軌跡の補間を行う
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

        // 結果の保存
        resultTrajectory.clear();
        vector<int>::iterator itID = idOpt.begin();
        for( vector<TrajectoryElement>::iterator itTrj = opt.begin(); itTrj != opt.end(); ++itTrj, ++itID ) {
            int id = ( *itID == -1 ) ? idNext++ : *itID;
            CTrajectory trj;
            trj.push_back( *itTrj );
            resultTrajectory[ id ] = trj;
        }

        //
        // 計算結果の出力
        //if( myRank == 0 ) {
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
        // [timeTracking - commonParam.termTracking, timeTracking]にクリッピング
        CTrajectory newStorageTrajectoryElement;
        newStorageTrajectoryElement.assign( storageTrajectoryElement.begin(), storageTrajectoryElement.end() );
        newStorageTrajectoryElement.Clip( timeTracking - ( commonParam.termTracking - commonParam.intervalTracking ), timeTracking );
        storageTrajectoryElement.assign( newStorageTrajectoryElement.begin(), newStorageTrajectoryElement.end() );

        timeTracking += commonParam.intervalTracking;
    }

    return true;
}
