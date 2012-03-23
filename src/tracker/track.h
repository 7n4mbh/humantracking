#ifndef __HEADER_TRACK__
#define __HEADER_TRACK__

#include <vector>
#include <map>
#include <string>

#include "ipp.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "PersonTrackingCommon.h"
#include "ProbSampleReplace.h"
#include "Trajectory.h"

typedef Vier::CProbSampleReplace<PosXYTVID> SamplerPosXYTVID; ///< 位置情報のサンプラー
typedef std::map< TIME_MICRO_SEC, std::vector<CLinearUniformMotion> > LUMStorage; ///< 等速直線運動ストレージ

typedef struct {
    TIME_MICRO_SEC termTracking; ///< １回の追跡処理が対象とする期間[usec]
    TIME_MICRO_SEC intervalTracking; ///< 追跡処理の間隔[usec]
    TIME_MICRO_SEC intervalTrajectory; ///< 軌跡の作成間隔[usec]
} PARAM_COMMON;

typedef struct {
    TIME_MICRO_SEC interval; ///< LUMの作成間隔[usec]
    TIME_MICRO_SEC term; ///< LUMを作成する期間[usec]
    double maxPEPMapValue; ///< PEPMapを閾値処理するときの最大値
    double minPEPMapValue; ///< PEPMapを閾値処理するときの最小値
    double maxSpeed; ///< 等速直線運動の最大の速さ[m/s]
    TIME_MICRO_SEC minDiffTime; ///< 抽出する等速直線運動の端点の最小の時刻差[usec]
    double kLUM; ///< 抽出する等速直線運動の個数を決めるための係数
    double kVerifySample; ///< 等速直線運動の検証に使うサンプルの個数を決めるための係数
    double distVerifySample; ///< 等速直線運動の検証に使うサンプルの範囲[m]
    double thMean; ///< 平均距離の閾値（等速直線運動の検証に利用）
    double thVariance; ///< 距離の分散の閾値（等速直線運動の検証に利用）
    TIME_MICRO_SEC intervalVerify; ///< 軌跡検証間隔[usec]
    TIME_MICRO_SEC rangeVerifySample; ///< 等速直線運動の検証に使うサンプルの範囲[usec]
    double stDeviation; ///< サンプル点にガウシアンノイズを加える場合の標準偏差（人物ID付きPEPMap利用時に適用される）
} PARAM_EXTRACTLUM;

typedef struct {
    double distanceImpact; ///< 軌跡の始点が影響を受ける等速直線運動との最長距離[m]
    double densityOrigin; ///< 軌跡の始点の密度[個・m^-2]
} PARAM_MKTRAJECTORY;

typedef struct {
    double thConnect; ///< 同一クラスタに分類される距離[m]の閾値
    double thDistance; ///< 同一クラスタに分類される距離[m]の閾値
    double minLength; ///< クラスタリングに用いる軌跡の最小の長さ[usec]
    double distanceLimit; ///< ２つの軌跡でdistanceLimit[m]以上の値をとる部分がnLimit[個]以上ある場合は，距離を無限大とする。
    unsigned int nLimit; ///< distanceLimitを参照
    TIME_MICRO_SEC minCommonTimeRange; ///< ２つの軌跡がminCommonTimeRange[usec]未満の共通部分しかない場合は距離を無限大とする。
    double distVerifyCluster; ///< クラスタに含まれる軌跡のまとまり具合を表す値。各時刻でクラスタの平均位置と構成軌跡の最短距離がdistVerifyCluster[m]以内であることを要求する。
} PARAM_CLUSTERING;

typedef struct {
    double thDistance; ///< 同一クラスタに分類される距離[m]の閾値
    double distanceLimit; ///< ２つの軌跡でdistanceLimit[m]以上の値をとる部分がnLimit[個]以上ある場合は，距離を無限大とする。
    unsigned int nLimit; ///< distanceLimitを参照
    TIME_MICRO_SEC minCommonTimeRange; ///< ２つの軌跡がminCommonTimeRange[usec]未満の共通部分しかない場合は距離を無限大とする。
} PARAM_MAKERESULT;

typedef struct {
    double rangeLeft; ///< 計算過程をGNUPLOTに表示するときの範囲（左端[m]）
    double rangeRight; ///< 計算過程をGNUPLOTに表示するときの範囲（右端[m]）
    double rangeTop; ///< 計算過程をGNUPLOTに表示するときの範囲（上端[m]）
    double rangeBottom; ///< 計算過程をGNUPLOTに表示するときの範囲（下端[m]）
} PARAM_PLOT;

bool load_track_parameters( std::string strPath );
void initialize_tracker();
bool track( const cv::Mat& occupancy, unsigned long long time_stamp );

void Gaussian( std::vector<PosXYTVID>* pPosVec, float stdev );

int ExtractLUM( SamplerPosXYTVID* pSampler
                , TIME_MICRO_SEC time
                , vector<CLinearUniformMotion>* pDst
                , const PARAM_COMMON* pCommonParam
                , const PARAM_EXTRACTLUM* pExtractlumParam
                , bool flgGaussian );

void AddPEPMapToSampler( /*CPEPMap* pPEPMap*/const cv::Mat& occupancy, unsigned long long time_stamp, SamplerPosXYTVID* pSampler, double minPEPMapValue, double maxPEPMapValue );
int MakeLUMSlice( TIME_MICRO_SEC tk, LUMStorage* pStorageLUM, std::vector<LUMSlice>* pDst, const PARAM_EXTRACTLUM* pExtractlumParam );
int ExtendTrajectories( TIME_MICRO_SEC startTime, TIME_MICRO_SEC endingTime, vector<TrajectoryElement>* pSrcDst, const vector<LUMSlice>* pLUMSlice, const PARAM_COMMON* pCommonParam, const PARAM_MKTRAJECTORY* pMkTrajectoryParam );

void OutputProcess( TIME_MICRO_SEC timeBegin
                     , TIME_MICRO_SEC timeEnd
                     , TIME_MICRO_SEC offset
                     , SamplerPosXYTVID* pSampler
                     , unsigned int nSample
                     , std::vector<TrajectoryElement>* pTrajectoryElementVec
                     , FILE* pipeGnuplot
                     , double stdDev
                     , const PARAM_PLOT* pPlotParam );
void OutputProcess( TIME_MICRO_SEC timeBegin
                     , TIME_MICRO_SEC timeEnd
                     , TIME_MICRO_SEC offset
                     , SamplerPosXYTVID* pSampler
                     , unsigned int nSample
                     , std::vector<CTrajectory>* pTrajectories
                     , string strDir
                     , string strExtention
                     , FILE* pipeGnuplot
                     , const PARAM_PLOT* pPlotParam );
void OutputProcess( TIME_MICRO_SEC timeBegin
                     , TIME_MICRO_SEC timeEnd
                     , TIME_MICRO_SEC offset
                     , SamplerPosXYTVID* pSampler
                     , unsigned int nSample
                     , std::map<int,CTrajectory>* pResult
                     , FILE* pipeGnuplot
                     , const PARAM_PLOT* pPlotParam );

#endif
