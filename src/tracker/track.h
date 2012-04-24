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
#include "RenovateTrajectory.h"

typedef Vier::CProbSampleReplace<PosXYTVID> SamplerPosXYTVID; ///< �ʒu���̃T���v���[
typedef std::map< TIME_MICRO_SEC, std::vector<CLinearUniformMotion> > LUMStorage; ///< ���������^���X�g���[�W

typedef struct {
    TIME_MICRO_SEC termTracking; ///< �P��̒ǐՏ������ΏۂƂ������[usec]
    TIME_MICRO_SEC intervalTracking; ///< �ǐՏ����̊Ԋu[usec]
    TIME_MICRO_SEC intervalTrajectory; ///< �O�Ղ̍쐬�Ԋu[usec]
} PARAM_COMMON;

typedef struct {
    TIME_MICRO_SEC interval; ///< LUM�̍쐬�Ԋu[usec]
    TIME_MICRO_SEC term; ///< LUM���쐬�������[usec]
    double maxPEPMapValue; ///< PEPMap��臒l��������Ƃ��̍ő�l
    double minPEPMapValue; ///< PEPMap��臒l��������Ƃ��̍ŏ��l
    double maxSpeed; ///< ���������^���̍ő�̑���[m/s]
    TIME_MICRO_SEC minDiffTime; ///< ���o���铙�������^���̒[�_�̍ŏ��̎�����[usec]
    double kLUM; ///< ���o���铙�������^���̌������߂邽�߂̌W��
    double kVerifySample; ///< ���������^���̌��؂Ɏg���T���v���̌������߂邽�߂̌W��
    double distVerifySample; ///< ���������^���̌��؂Ɏg���T���v���͈̔�[m]
    double thMean; ///< ���ϋ�����臒l�i���������^���̌��؂ɗ��p�j
    double thVariance; ///< �����̕��U��臒l�i���������^���̌��؂ɗ��p�j
    TIME_MICRO_SEC intervalVerify; ///< �O�Ռ��؊Ԋu[usec]
    TIME_MICRO_SEC rangeVerifySample; ///< ���������^���̌��؂Ɏg���T���v���͈̔�[usec]
    double stDeviation; ///< �T���v���_�ɃK�E�V�A���m�C�Y��������ꍇ�̕W���΍��i�l��ID�t��PEPMap���p���ɓK�p�����j
} PARAM_EXTRACTLUM;

typedef struct {
    double distanceImpact; ///< �O�Ղ̎n�_���e�����󂯂铙�������^���Ƃ̍Œ�����[m]
    double densityOrigin; ///< �O�Ղ̎n�_�̖��x[�Em^-2]
} PARAM_MKTRAJECTORY;

typedef struct {
    double thConnect; ///< ����N���X�^�ɕ��ނ���鋗��[m]��臒l
    double thDistance; ///< ����N���X�^�ɕ��ނ���鋗��[m]��臒l
    double minLength; ///< �N���X�^�����O�ɗp����O�Ղ̍ŏ��̒���[usec]
    double distanceLimit; ///< �Q�̋O�Ղ�distanceLimit[m]�ȏ�̒l���Ƃ镔����nLimit[��]�ȏ゠��ꍇ�́C�����𖳌���Ƃ���B
    unsigned int nLimit; ///< distanceLimit���Q��
    TIME_MICRO_SEC minCommonTimeRange; ///< �Q�̋O�Ղ�minCommonTimeRange[usec]�����̋��ʕ��������Ȃ��ꍇ�͋����𖳌���Ƃ���B
    double distVerifyCluster; ///< �N���X�^�Ɋ܂܂��O�Ղ̂܂Ƃ܂���\���l�B�e�����ŃN���X�^�̕��ψʒu�ƍ\���O�Ղ̍ŒZ������distVerifyCluster[m]�ȓ��ł��邱�Ƃ�v������B
} PARAM_CLUSTERING;

typedef struct {
    double thDistance; ///< ����N���X�^�ɕ��ނ���鋗��[m]��臒l
    double distanceLimit; ///< �Q�̋O�Ղ�distanceLimit[m]�ȏ�̒l���Ƃ镔����nLimit[��]�ȏ゠��ꍇ�́C�����𖳌���Ƃ���B
    unsigned int nLimit; ///< distanceLimit���Q��
    TIME_MICRO_SEC minCommonTimeRange; ///< �Q�̋O�Ղ�minCommonTimeRange[usec]�����̋��ʕ��������Ȃ��ꍇ�͋����𖳌���Ƃ���B
} PARAM_MAKERESULT;

typedef struct {
    double rangeLeft; ///< �v�Z�ߒ���GNUPLOT�ɕ\������Ƃ��͈̔́i���[[m]�j
    double rangeRight; ///< �v�Z�ߒ���GNUPLOT�ɕ\������Ƃ��͈̔́i�E�[[m]�j
    double rangeTop; ///< �v�Z�ߒ���GNUPLOT�ɕ\������Ƃ��͈̔́i��[[m]�j
    double rangeBottom; ///< �v�Z�ߒ���GNUPLOT�ɕ\������Ƃ��͈̔́i���[[m]�j
    double kSample; ///< ���o����T���v���������߂邽�߂̔��W��
} PARAM_PLOT;

bool load_track_parameters( std::string strPath, std::string strFileName );
void initialize_tracker();
bool track( std::map< unsigned long long, std::map<int,cv::Point2d> >* p_result, std::map<unsigned long long, std::multimap<int,cv::Point2d> >* p_ext_result, const cv::Mat& occupancy, unsigned long long time_stamp );

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
void CalculateDistanceTable( double* distTable
                                , std::vector<size_t>* pIdxTrjToCol
                                , size_t stepDistTable
                                , std::map<int,CTrajectory>::iterator itStart
                                , std::map<int,CTrajectory>::iterator itEnd
                                , std::map<int,CTrajectory>* pTrajectories
                                , CTrajectory_Distance& funcDist );
int Clustering( std::vector<int>* pIndexCluster, double* distTable, size_t nElements, double threshold );
int Clustering2( std::vector< std::vector<int> >* pDst, std::vector<int>& classID, double* distTable, size_t nElements, double thConnect, double thDiameter );
bool VerifyClusteredTrajectories( CTrajectory& trj, double threshold );
void CalcFrequency( double* pDst, double* distTable, int size, double sigma, double thDist );
void ReduceTrajectory( vector<int>* pDst, double* frequency, int size, double fval );
void DivideIntoSections( TrajectoriesInfo* pInfoTrj, std::map<int,int>* pPointIdxToTrjNo, PARAM_RENOVATE_TRAJECTORY param );
void MakeSet( int idxSection, TrajectoriesInfo* pInfoTrj, map<int,int>* pReserve );
double Optimize( std::vector<TrajectoryElement>* pDst, std::vector<int>* pDstID, std::map<int,int>* pReserve, int idxSection, int idxSet, int nStart, TrajectoriesInfo* pInfoTrj, std::map<int,int>* pPointIdxToTrjNo, PARAM_RENOVATE_TRAJECTORY param, double* p_t1 = NULL, double* p_t2 = NULL, double* p_t3 = NULL );

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
