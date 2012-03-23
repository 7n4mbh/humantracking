#include <numeric>

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "track.h"


using namespace std;
using namespace cv;

PARAM_COMMON commonParam; // ���ʃp�����[�^
PARAM_EXTRACTLUM extractlumParam; // ���������^�����o�p�����[�^
PARAM_MKTRAJECTORY mktrajectoryParam; // �O�Ս쐬�p�����[�^
//PARAM_CLUSTERING clusteringParam; // �N���X�^�����O�p�����[�^
//PARAM_MAKERESULT makeresultParam; // �ǐՌ��ʍ쐬�p�����[�^
//PARAM_RENOVATE_TRAJECTORY rnvtrjParam; // �O�ՏC���p�����[�^
PARAM_PLOT plotParam; // �v�Z�ߒ��v���b�g�p�����[�^

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
    static TIME_MICRO_SEC timeTracking; // �ǐՎ���[usec]
                                 // [timeTracking - commonParam.termTracking, timeTrackig) �͈̔͂ŒǐՏ������s�������Ӗ�����

    static SamplerPosXYTVID sampler; // �����ʃT���v��

    // storageLUM
    // ����tk�i�L�[�j�ɑΉ�����LUM�W���ւ̃}�b�v
    // ����tk�ɑΉ����铙�������^���W���Ƃ́A[tk - extractlumParam.term, tk)��PEPMap����쐬�������������^���W���̂��Ƃł���B
    static LUMStorage storageLUM;

    // tableLUMSlice
    // LUM�X���C�X�e�[�u��
    // ����tk�i�L�[�j�ɂ�����LUM�X���C�X�̔z��ւ̃}�b�v
    static LUMSliceTable tableLUMSlice;

    // storageTrajectory
    // �쐬���̋O�՗v�f
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
    
    // PEPMap���T���v���ɒǉ�����
    AddPEPMapToSampler( occupancy
                      , time_stamp
                        , &sampler
                        , extractlumParam.minPEPMapValue
                        , extractlumParam.maxPEPMapValue );

    //
    // LUM���o
    vector<TIME_MICRO_SEC> addedTime;
    for( TIME_MICRO_SEC tk = timeTracking - commonParam.termTracking; tk <= time_stamp; tk += extractlumParam.interval ) {
        if( storageLUM.find( tk ) == storageLUM.end() ) {
            // ����tk��LUM�𒊏o
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
    // LUM�X���C�X�e�[�u���쐬
    if( !storageLUM.empty() ) {
        TIME_MICRO_SEC timeLatestLUM = storageLUM.rbegin()->first;
        addedTime.clear();
        // timeLatestLUM�܂ł�LUM�������Ă���Ƃ��CLUM�X���C�X�e�[�u�����쐬�ł���̂�
        // (timeLatestLUM - extractlumParam.term)�܂ŁB���͈̔͂�LUM�X���C�X�e�[�u�����쐬����B
        for( TIME_MICRO_SEC tk = timeTracking - commonParam.termTracking
            ; tk <= timeLatestLUM - extractlumParam.term
            ; tk += commonParam.intervalTrajectory ) {
            if( tableLUMSlice.find( tk ) == tableLUMSlice.end() ) {
                // ����tk��LUM�X���C�X�쐬
                //cerr << "LUM�X���C�X�ǉ�: ";
                MakeLUMSlice( tk, &storageLUM, &tableLUMSlice[ tk ], &extractlumParam );
                addedTime.push_back( tk );
            }
        }
    }

    // tableLUMSlice�ɒǉ����ꂽ�e�����Ɏn�_���߂�B
    set<PosXYT,PosXYT_XYT_Less> originPos;
    const double intersticeOrigin = 1.0 / sqrt( mktrajectoryParam.densityOrigin ); // �O�Ղ̎n�_���m�̊Ԋu
    const double rangeOrigin = mktrajectoryParam.distanceImpact * 2.0; // �O�Ղ̏��X,Y���W�̎���rangeOrigin�l���͈̔͂Ɏn�_�̌����쐬����
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

    // MPI�ɂ��n�_���e�v���Z�X�Ɋ���U��
    // �ȉ��C���L�q
    vector<PosXYT> originPosPerProcess;
    originPosPerProcess.assign( originPos.begin(), originPos.end() );

    //
    // �󂯎�����n�_����ɐV���ȋO�Ղ𐶐�����
    int nNewTrj = 0;
    vector<PosXYT>::iterator itOrigin = originPosPerProcess.begin();
    for( int i = 0; itOrigin != originPosPerProcess.end(); ++itOrigin, ++i ) {
        TrajectoryElement trjElement;
        trjElement.insert( *itOrigin );
        storageTrajectoryElement.push_back( trjElement );
        ++nNewTrj;
    }

    //
    // tableLUMSlice�ɒǉ����ꂽ�e�����ɂ��ċO�Ղ���������
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

        // �v�Z�ߒ����v���b�g
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
        cerr << "����" << endl;

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
        // [timeTracking - commonParam.termTracking, timeTracking]�ɃN���b�s���O
        CTrajectory newStorageTrajectoryElement;
        newStorageTrajectoryElement.assign( storageTrajectoryElement.begin(), storageTrajectoryElement.end() );
        newStorageTrajectoryElement.Clip( timeTracking - ( commonParam.termTracking - commonParam.intervalTracking ), timeTracking );
        storageTrajectoryElement.assign( newStorageTrajectoryElement.begin(), newStorageTrajectoryElement.end() );

        timeTracking += commonParam.intervalTracking;
    }

    return true;
}