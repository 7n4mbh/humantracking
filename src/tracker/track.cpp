#include <numeric>

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "track.h"


using namespace std;
using namespace cv;

PARAM_COMMON commonParam; // 共通パラメータ
PARAM_EXTRACTLUM extractlumParam; // 等速直線運動抽出パラメータ
PARAM_MKTRAJECTORY mktrajectoryParam; // 軌跡作成パラメータ
//PARAM_CLUSTERING clusteringParam; // クラスタリングパラメータ
//PARAM_MAKERESULT makeresultParam; // 追跡結果作成パラメータ
//PARAM_RENOVATE_TRAJECTORY rnvtrjParam; // 軌跡修復パラメータ
PARAM_PLOT plotParam; // 計算過程プロットパラメータ

bool flgFirst;

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
    extractlumParam.kLUM = 3.5e-1;//3.0e-2;
    extractlumParam.kVerifySample = 3.5e-1;//6.0e-2;
    extractlumParam.distVerifySample = 0.15;
    extractlumParam.thMean = 0.05;
    extractlumParam.thVariance = 0.4;
    extractlumParam.intervalVerify = 100000;
    extractlumParam.rangeVerifySample = 100000;
    extractlumParam.stDeviation = 0.05;
    mktrajectoryParam.distanceImpact = 0.07;;
    mktrajectoryParam.densityOrigin = 60.0;
    plotParam.rangeLeft = -2.0;
    plotParam.rangeRight = 2.0;
    plotParam.rangeTop = 8.0;
    plotParam.rangeBottom = 1.0;

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

    static TIME_MICRO_SEC timeEarliestPEPMap;

    if( flgFirst ) {
        sampler.clear();
        storageLUM.clear();
        tableLUMSlice.clear();
        storageTrajectoryElement.clear();
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
        unsigned int nSample = (unsigned int)( 3.0e-3/*1.04e-4*/ * sumValue );
        OutputProcess( timeTracking - commonParam.termTracking//tableLUMSlice.begin()->first
                        , timeTracking//tableLUMSlice.rbegin()->first
                        , timeEarliestPEPMap
                        , &sampler
                        , nSample
                        , &storageTrajectoryElement
                        , NULL//pipeGnuplot_Trajectory
                        , extractlumParam.stDeviation
                        , &plotParam );
        cerr << "完了" << endl;

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