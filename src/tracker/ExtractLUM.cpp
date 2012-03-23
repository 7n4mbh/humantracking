/**
 * @file ExtractLUM.cpp
 * @brief 等速直線運動抽出処理に関する実装
 *
 * @author 福司 謙一郎
 * @date 2009
 */

//#include <mpi.h>
#include <utility>
#include <numeric>

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "track.h"
//#include "MersenneRandomizer.h"

using namespace std;
using namespace Vier;

extern float roi_width, roi_height;
extern float roi_x, roi_y;
extern float scale_m2px;

/**
 * 等速直線運動の検証
 * @param pSrc 検証対象の等速直線運動
 * @param pDst 検証対象の等速直線運動から不正な等速直線運動を消去したものを返す
 * @param pSamples 検証に使うサンプル
 * @param distVerifySample 等速直線運動の検証に使うサンプルの範囲[m]
 * @param thMean 距離の平均の閾値
 * @param thVariance 距離の分散の閾値
 * @param intervalVerify 軌跡検証間隔[usec]
 * @param rangeVerifySample 検証に使うサンプルの範囲[usec]
 */
void VerifyLUM( vector<CLinearUniformMotion>* pSrc, vector<CLinearUniformMotion>* pDst, vector<PosXYTID>* pSamples, double distVerifySample, double thMean, double thVariance, TIME_MICRO_SEC intervalVerify, TIME_MICRO_SEC rangeVerifySample );

/**
 * CalcSampleDistributionの準備
 * @param pSamples 検証に使うサンプル
 * @param pos_x time±rangeVerifySample[usec]にあるサンプルのX座標[m]
 * @param pos_y time±rangeVerifySample[usec]にあるサンプルのY座標[m]
 * @param pos_dt time±rangeVerifySample[usec]にあるサンプルの時刻[usec]
 */
unsigned int CalcSampleDistributionPrepare( vector<PosXYTID>* pSamples, Ipp32f* pos_x, Ipp32f* pos_y, Ipp32f* pos_dt, Ipp8u* pos_id, TIME_MICRO_SEC time, TIME_MICRO_SEC rangeVerifySample );

/**
 * 指定した等速直線情報の周りのサンプル分布を計算する
 * @param infoLUMSlice 等速直線情報
 * @param p_ex 分布のX方向に関する平均[m]
 * @param p_ey 分布のY方向に関する平均[m]
 * @param p_density 分布の密度
 * @param p_variance 分布の分散
 * @param distVerifySample サンプル分布計算に利用するサンプルの範囲[m]
 * @param nSamplesInRange サンプル分布計算に利用するサンプルの個数
 * @param pos_x サンプル分布計算に利用するサンプルのX座標[m]
 * @param pos_y サンプル分布計算に利用するサンプルのY座標[m]
 * @param pos_dt サンプル分布計算に利用するサンプルの時刻情報[usec]
 * @param tmpx,tmpz,tmp1,tmp2 計算用
 * @return 指定した等速直線運動情報の周りにあったサンプル数
 */
int CalcSampleDistribution( LUMSlice LUMSlice, double* p_ex, double* p_ey, double* p_density, vector<double>* p_variance/*double* p_variance1, double* p_variance2*/, double distVerifySample, int nSamplesInRange, Ipp32f* pos_x, Ipp32f* pos_y, Ipp32f* pos_dt, Ipp8u* pos_id, Ipp32f* tmpx, Ipp32f* tmpy, Ipp32f* tmp1, Ipp32f* tmp2 );

int ExtractLUM( SamplerPosXYTVID* pSrc
                , TIME_MICRO_SEC time
                , vector<CLinearUniformMotion>* pDst
                , const PARAM_COMMON* pCommonParam
                , const PARAM_EXTRACTLUM* pExtractlumParam
                , bool flgGaussian )
{
    TIME_MICRO_SEC timeBegin, timeEnd;
    SamplerPosXYTVID::iterator itBegin, itEnd;

    timeEnd = time;
    timeBegin = ( timeEnd > pExtractlumParam->term ) ? timeEnd - pExtractlumParam->term : 0;
    itBegin = lower_bound( pSrc->begin()
                         , pSrc->end()
                         , pair<PosXYT,unsigned long>( PosXYT( 0.0, 0.0, timeBegin ), 0 )
                         , PosXYT_T_Less() );
    itEnd = upper_bound( pSrc->begin()
                       , pSrc->end()
                       , pair<PosXYT,unsigned long>( PosXYT( 0.0, 0.0, timeEnd ), 0 )
                       , PosXYT_T_Less() );

    //
    // pSrcから，[timeBegin, timeEnd]の範囲にあるデータを取り出す。
    SamplerPosXYTVID sampler;
    SamplerPosXYTVID tmp;
    sampler.assign( itBegin, itEnd );
    tmp.assign( itBegin, itEnd );

    //
    // 抽出する等速直線運動数を計算する
    //
    double sumValue = accumulate( sampler.begin(), sampler.end(), 0.0, PosXYTV_Sum() );
    //cout << "sumValue=" << sumValue << endl;
    unsigned int nLUM = (unsigned int)( pExtractlumParam->kLUM * sumValue );
    tmp.erase( unique( tmp.begin(), tmp.end(), PosXYT_T_Equal() ), tmp.end() );
    if( tmp.size() > 0 ) {
        nLUM /= tmp.size();
    }

//    int myRank;
//    MPI_Comm_rank( MPI_COMM_WORLD, &myRank );
//
//    if( myRank == 0 ) {
//        cerr << "nLUM=" << nLUM << ", sumValue=" << sumValue << ", tmp.size()=" << tmp.size()
//             << ", pSrc->size()=" << pSrc->size() << " ";
//             //<< ", timeBegin=" << timeBegin << ", itEnd=" << timeEnd << " ";
//    }

    //
    // 等速直線運動を確率的に生成する
    //
    vector<CLinearUniformMotion> lumVec;
    lumVec.reserve( nLUM );
    unsigned int cntLUM = 0;
    if( nLUM > 0 ) {
        //
        // サンプラーからnLUMの２倍のサンプルを取り出す。２倍なのは，１つの等速直線運動が２つのサンプルから成るため。
        vector<PosXYTVID> posVec( 2 * nLUM );
        sampler.Execute( (PosXYTVID*)&(*posVec.begin()), 2 * nLUM );
        if( flgGaussian ) {
            Gaussian( &posVec, pExtractlumParam->stDeviation );
        }

        //
        // 等速直線運動の作成
        CLinearUniformMotion lum;
        for( size_t i = 0; i < 2 * nLUM; i += 2 ) {
            //if( true ) {
            if( ( posVec[ i ].ID == 0 ) || ( posVec[ i + 1 ].ID == 0 ) || ( posVec[ i ].ID == posVec[ i + 1 ].ID ) ) {
            //if( ( posVec[ i ].ID != 0 ) && ( posVec[ i + 1 ].ID != 0 ) && ( posVec[ i ].ID == posVec[ i + 1 ].ID ) ) {
                // 同一人物IDを結ぶ等速直線運動のみ作成する
                // ただしID=0を少なくとも一つの端点に持つ線分は許可する
                lum = CLinearUniformMotion( posVec[ i ], posVec[ i + 1 ] );
                if( lum.GetTimeDifference() >= pExtractlumParam->minDiffTime && lum.GetSpeed() <= pExtractlumParam->maxSpeed ) {
                    lumVec.push_back( lum );
                    cntLUM++;
                }
            }
        }
    }

    // サンプルの少ない部分を通る等速直線運動を消去する
    vector<CLinearUniformMotion> newLUM;
    newLUM.reserve( lumVec.size() );
    if( tmp.size() > 0 ) {
        vector<PosXYTVID> posvVec( (size_t)( pExtractlumParam->kVerifySample * sumValue ) / tmp.size() );
        //cerr << "verifySamle=" << posvVec.size() << " ";
        vector<PosXYTID> posVec( posvVec.size() );
        if( posVec.size() > 0 ) {
            sampler.Execute( (PosXYTVID*)&(*posvVec.begin()), posvVec.size() );
            if( flgGaussian ) {
                Gaussian( &posvVec, pExtractlumParam->stDeviation );
            }
            posVec.assign( posvVec.begin(), posvVec.end() );
            VerifyLUM( &lumVec
                     , &newLUM
                     , &posVec
                     , pExtractlumParam->distVerifySample
                     , pExtractlumParam->thMean
                     , pExtractlumParam->thVariance
                     , pExtractlumParam->intervalVerify
                     , pExtractlumParam->rangeVerifySample );
        }
    }

    // 今回作成した等速直線運動をstorageLUMに追加する
    pDst->insert( pDst->end(), newLUM.begin(), newLUM.end() );
    sort( pDst->begin(), pDst->end(), LUM_EndTime_Less() );

    return TRUE;
}

void Gaussian( vector<PosXYTVID>* pPosVec, float stdev )
{
    //static nyx::util::random::MersenneRandomizer mr( 0, RAND_MAX );
    Ipp32f* gaussrand = new Ipp32f[ pPosVec->size() * 2 ];
    static unsigned int seed;
    static IppsRandGaussState_32f* pRandGaussState = NULL;
    static float _stdev = 0.0f;


    if( _stdev != stdev ) {
        _stdev = stdev;
        if( pRandGaussState ) {
            ippsRandGaussFree_32f( pRandGaussState );
            pRandGaussState = NULL;
        }
    }

    if( !pRandGaussState ) {
        //seed = mr.getNumber();
        seed = rand();
        ippsRandGaussInitAlloc_32f( &pRandGaussState, 0.0f, (float)_stdev, seed );
    }

    ippsRandGauss_32f( gaussrand, pPosVec->size() * 2, pRandGaussState );

//    cerr << "Gaussianノイズをかけます(seed=" << seed << ", stdev=" << _stdev << "): ";
//    for( int i = 0; i < min( pPosVec->size() * 2, 10 ); ++i ) {
//        cerr << gaussrand[ i ] << " ";
//    }
//    cerr << endl;

    int i = 0;
    for( vector<PosXYTVID>::iterator it = pPosVec->begin(); it != pPosVec->end(); ++it ) {
        it->x += gaussrand[ i++ ];
        it->y += gaussrand[ i++ ];
    }

    delete [] gaussrand;
}

void AddPEPMapToSampler( const cv::Mat& occupancy
                        , unsigned long long time_stamp
                           , SamplerPosXYTVID* pSampler
                           , double minPEPMapValue
                           , double maxPEPMapValue )
{
    //IppiData_32f PEPMapMat;
    //IppiData_8u PIDMapMat;
    //RECTROI rectROI;
    double resX, resZ;
    int nRow, nCol;

    // PEPMapの諸情報取得
    nRow = occupancy.rows;
    nCol = occupancy.cols;

    for( int row = 0; row < nRow; ++row ) {
        for( int col = 0; col < nCol; ++col ) {
            //double value = *DATA_PTR_32F( PEPMapMat.pData, col, row, PEPMapMat.step ); // 座標(col,row)でのPEPMapの値を取得
            //int id = *DATA_PTR_8U( PIDMapMat.pData, col, row, PIDMapMat.step ); // 座標(col,row)での人物IDの値を取得
            float value = occupancy.at<unsigned short>( row, col );
            if( value > minPEPMapValue ) {
                // minPEPMapValueより大きい値を持つ座標のみ追加する

                // valueを上限値maxPEPmapValueで頭打ちにする
                if( value > maxPEPMapValue )
                    value = maxPEPMapValue;

                // 座標(col,row)を実空間の座標(x[m],y[m])に変換する
                PosXYTVID pos;
                //pos.x = rectROI.left + (double)col / resX;
                //pos.y = rectROI.bottom + (double)row / resZ;
                //pos.t = pPEPMap->GetTime();
                //pos.value = value;
                //pos.ID = id;
                pos.x = roi_x - roi_width / 2.0f + (float)row / scale_m2px;
                pos.y = roi_y - roi_height / 2.0f + (float)col / scale_m2px;
                pos.t = time_stamp;
                pos.value = value;
                pos.ID = 1;//id;
                // 仮
                // 四隅のノイズを除去する
                bool flgOutOfROI = false;
                if( ( pos.x < 0.5 && pos.y > 6.7 ) || ( pos.x < 0.5 && pos.y < 3.5 ) ) {
                    flgOutOfROI = true;
                }
                if( ( pos.x > 1.1 && pos.y > 6.7 ) || ( pos.x > 1.1 && pos.y < 3.5 ) ) {
                    flgOutOfROI = true;
                }

                if( !flgOutOfROI ) {
                    // 復元抽出時の重みweightをvalueから計算する
                    unsigned long weight = (unsigned long)( value - minPEPMapValue );

                    // サンプラーに追加する
                    pSampler->push_back( pair<PosXYTVID,unsigned long>( pos, weight ) );
                }
            }
        }
    }


    //ippiFree( PEPMapMat.pData );
}

void VerifyLUM( vector<CLinearUniformMotion>* pSrc, vector<CLinearUniformMotion>* pDst, vector<PosXYTID>* pSamples, double distVerifySample, double thMean, double thVariance, TIME_MICRO_SEC intervalVerify, TIME_MICRO_SEC rangeVerifySample )
{
    size_t nSamples = pSamples->size();
    Ipp32f* pos_x = new Ipp32f[ nSamples ];
    Ipp32f* pos_y = new Ipp32f[ nSamples ];
    Ipp32f* pos_dt = new Ipp32f[ nSamples ];
    Ipp8u* pos_id = new Ipp8u[ nSamples ];
    Ipp32f* tmpx = new Ipp32f[ nSamples ];
    Ipp32f* tmpz = new Ipp32f[ nSamples ];
    Ipp32f* tmp1 = new Ipp32f[ nSamples ];
    Ipp32f* tmp2 = new Ipp32f[ nSamples ];
    vector<bool> valid( pSrc->size(), true );
    size_t cnt = pSrc->size();

    TIME_MICRO_SEC timeBegin, timeEnd;
    timeBegin = min_element( pSamples->begin(), pSamples->end(), PosXYT_T_Less() )->t;
    timeEnd = max_element( pSamples->begin(), pSamples->end(), PosXYT_T_Less() )->t;

    for( TIME_MICRO_SEC time = timeBegin; time <= timeEnd; time += intervalVerify ) {
        size_t nSamplesInRange = CalcSampleDistributionPrepare( pSamples, pos_x, pos_y, pos_dt, pos_id, time, rangeVerifySample );

        vector<CLinearUniformMotion>::iterator itLUM = pSrc->begin();
        vector<bool>::iterator itValid = valid.begin();
        for( ; itLUM != pSrc->end(); itLUM++, itValid++ ) {
            LUMSlice lumSlice;
            // 始点・終点のいずれかが色情報に因らない（IDが0）LUMについてチェックを行う。
            if( *itValid == true && /*!itLUM->IsColoredLUM() &&*/ itLUM->GetLUMSlice( time, &lumSlice ) ) {
                int ret;
                double ex, ey, den, var1, var2;
                vector<double> var;
//                for( double v = 0.45; v >= 0.15; v -= 0.01 ) {
//                    var.push_back( v );
//                }
                var.push_back( 0.32 );
                //var.push_back( 0.20 );
                var.push_back( 0.15 );
                var.push_back( 0.025 );
                ret = CalcSampleDistribution( lumSlice
                                            , &ex
                                            , &ey
                                            , &den
                                            //, &var1
                                            //, &var2
                                            , &var
                                            , distVerifySample
                                            , nSamplesInRange
                                            , pos_x
                                            , pos_y
                                            , pos_dt
                                            , pos_id
                                            , tmpx
                                            , tmpz
                                            , tmp1
                                            , tmp2 );
                //if( ret == 0 || ( ( ex * ex + ey * ey ) > thMean * thMean ) || ( ( var1 > /*0.49*//*0.47*//*0.4*//*0.43*/thVariance ) && ( var2 > /*0.36*//*0.49*//*0.47*/thVariance ) ) ) {
                if( ret == 0 || ( ( ex * ex + ey * ey ) > thMean * thMean ) || ( *min_element( var.begin(), var.end() - 1 ) > thVariance ) ) {
                //if( ret == 0 || ( ( ex * ex + ey * ey ) > thMean * thMean ) || ( ( var[ 0 ] > thVariance ) && ( var[ 1 ] > thVariance ) ) ) {
                    *itValid = false;
                    cnt--;
                }
                //cerr << "ret=" << ret << " ";

            }
        }
    }

    //vector<CLinearUniformMotion> retLUM( cnt );
    pDst->resize( cnt );
    vector<CLinearUniformMotion>::iterator itLUM = pSrc->begin(), itRetLUM = pDst->begin();
    vector<bool>::iterator itValid = valid.begin();
    for( ; itLUM != pSrc->end(); itLUM++, itValid++ ) {
        if( *itValid ) {
            *itRetLUM = *itLUM;
            itRetLUM++;
        }
    }

    delete [] pos_x;
    delete [] pos_y;
    delete [] pos_dt;
    delete [] pos_id;
    delete [] tmpx;
    delete [] tmpz;
    delete [] tmp1;
    delete [] tmp2;
}

unsigned int CalcSampleDistributionPrepare( vector<PosXYTID>* pSamples, Ipp32f* pos_x, Ipp32f* pos_y, Ipp32f* pos_dt, Ipp8u* pos_id, TIME_MICRO_SEC time, TIME_MICRO_SEC rangeVerifySample )
{
    vector<PosXYTID>::iterator itBegin, itEnd;
    TIME_MICRO_SEC timeBegin, timeEnd;
    unsigned int cnt = 0;

    timeBegin = ( time > rangeVerifySample ) ? time - rangeVerifySample : 0;
    timeEnd = time + rangeVerifySample;

    // pSamplesの内，time±rangeVerifySample[usec]にあるサンプルの範囲を求める
    sort( pSamples->begin(), pSamples->end(), PosXYT_T_Less() );
    itBegin = lower_bound( pSamples->begin(), pSamples->end(), PosXYT( 0.0, 0.0, timeBegin ), PosXYT_T_Less() );
    itEnd = upper_bound( pSamples->begin(), pSamples->end(), PosXYT( 0.0, 0.0, timeEnd ), PosXYT_T_Less() );

    for( ; itBegin != itEnd; itBegin++ ) {
        *pos_x = (float)itBegin->x;
        *pos_y = (float)itBegin->y;
        *pos_dt = ( itBegin->t > time ) ? (float)( itBegin->t - time ) : -(float)( time - itBegin->t );
        *pos_id = itBegin->ID;
        pos_x++;
        pos_y++;
        pos_dt++;
        pos_id++;
        cnt++;
    }

    return cnt;
}

int CalcSampleDistribution( LUMSlice lumSlice, double* p_ex, double* p_ey, double* p_density, vector<double>* p_variance/*double* p_variance1, double* p_variance2*/, double distVerifySample, int nSamplesInRange, Ipp32f* pos_x, Ipp32f* pos_y, Ipp32f* pos_dt, Ipp8u* pos_id, Ipp32f* tmpx, Ipp32f* tmpy, Ipp32f* tmp1, Ipp32f* tmp2 )
{
    if( nSamplesInRange == 0 )
        return 0;

    int cnt;
    const double r = distVerifySample;
    IppiSize roiSize;
    const int step = nSamplesInRange * sizeof( Ipp32f );
    Ipp32f ret_ex, ret_ey, ret_variance;

    roiSize.width = nSamplesInRange;
    roiSize.height = 1;

    // x, z方向の差分をとる
    ippiSubC_32f_C1R( pos_x
                    , step
                    , (Ipp32f)lumSlice.pos.x
                    , tmpx
                    , step
                    , roiSize );
    ippiSubC_32f_C1R( pos_y
                    , step
                    , (Ipp32f)lumSlice.pos.y
                    , tmpy
                    , step
                    , roiSize );

    // lumSlice.gradX, lumSlice.gradYを考慮して補正
    ippiMulC_32f_C1R( pos_dt
                    , step
                    , (Ipp32f)-lumSlice.gradX
                    , tmp1
                    , step
                    , roiSize );
    ippiMulC_32f_C1R( pos_dt
                    , step
                    , (Ipp32f)-lumSlice.gradY
                    , tmp2
                    , step
                    , roiSize );
    ippiAdd_32f_C1IR( tmp1
                    , step
                    , tmpx
                    , step
                    , roiSize );
    ippiAdd_32f_C1IR( tmp2
                    , step
                    , tmpy
                    , step
                    , roiSize );

    // tmpx, tmpyの各要素を２乗し，和を取ってtmp1に代入する
    // tmp1はposと各サンプルの距離の２乗になる
    ippiSqr_32f_C1R( tmpx
                   , step
                   , tmp1
                   , step
                   , roiSize );
    ippiSqr_32f_C1R( tmpy
                   , step
                   , tmp2
                   , step
                   , roiSize );
    ippiAdd_32f_C1IR( tmp2
                   , step
                   , tmp1
                   , step
                   , roiSize );

    // tmp1中で，lumSlice.IDとIDが一致しないサンプルに対応する距離を( 2 * r * r )に書き換える。
    // （２倍するのは(r*r)より十分大きくするため）
    // ただし，ID=0のサンプルはlumSlice.IDの値に関わらず一致するとみなす。
    //if( false ) {
    if( ( lumSlice.pos.ID != 0 ) ) {
        for( int i = 0; i < roiSize.width; ++i ) {
            if( ( pos_id[ i ] != 0 ) && ( pos_id[ i ] != lumSlice.pos.ID ) ) {
                tmp1[ i ] = 2.0 * r * r;
            }
        }
    }

    // posとの距離がr以下のサンプルの個数を数える
    ippiCountInRange_32f_C1R( tmp1
                            , step
                            , roiSize
                            , &cnt
                            , 0.0f
                            , (Ipp32f)( r * r ) );

    // posとの距離がrより大きい要素をゼロ，それ以外を１としたtmp2を作る
    // 下記の計算だと最初にゼロだった要素は１となるべきだがゼロになる。
    // しかし，平均を取るのに問題は無い。
    ippiThreshold_GTVal_32f_C1R( tmp1
                               , step
                               , tmp2
                               , step
                               , roiSize
                               , (Ipp32f)( r * r )
                               , 0.0f );
    ippiThreshold_GTVal_32f_C1IR( tmp2
                                , step
                                , roiSize
                                , 0.0f
                                , 1.0f );

    // x方向の平均計算
    ippmDotProduct_vv_32f( tmpx
                         , sizeof( Ipp32f )
                         , tmp2
                         , sizeof( Ipp32f )
                         , &ret_ex
                         , nSamplesInRange );
    *p_ex = ret_ex / (float)cnt;

    // z方向の平均計算
    ippmDotProduct_vv_32f( tmpy
                         , sizeof( Ipp32f )
                         , tmp2
                         , sizeof( Ipp32f )
                         , &ret_ey
                         , nSamplesInRange );
    *p_ey = ret_ey / (float)cnt;

    // 分散計算
    //ippmDotProduct_vv_32f( tmp1
    //                     , sizeof( Ipp32f )
    //                     , tmp2
    //                     , sizeof( Ipp32f )
    //                     , &ret_variance
    //                     , nSamplesInRange );
    //*p_variance = ret_variance / (float)cnt;

    // 分散計算（パターン２）
    //ippiSubC_32f_C1IR( (Ipp32f)*p_ex
    //                 , tmpx
    //                 , step
    //                 , roiSize );
    //ippiSqr_32f_C1IR( tmpx
    //                , step
    //                , roiSize );
    //ippmDotProduct_vv_32f( tmpx
    //                     , sizeof( Ipp32f )
    //                     , tmp2
    //                     , sizeof( Ipp32f )
    //                     , &ret_variance
    //                     , nSamplesInRange );
    //*p_variance = ret_variance / (float)cnt;
    //ippiSubC_32f_C1IR( (Ipp32f)*p_ey
    //                 , tmpy
    //                 , step
    //                 , roiSize );
    //ippiSqr_32f_C1IR( tmpy
    //                , step
    //                , roiSize );
    //ippmDotProduct_vv_32f( tmpy
    //                     , sizeof( Ipp32f )
    //                     , tmp2
    //                     , sizeof( Ipp32f )
    //                     , &ret_variance
    //                     , nSamplesInRange );
    //*p_variance += ret_variance / (float)cnt;

#define VAR_ARRAY
#ifndef VAR_ARRAY
    // 分散計算
    // 実際には 半径r/(√2)[m] 以外にある点の数を cntで割ったもの。
    {
        int cnt2;
        const double r2 = 0.32;//0.27;//0.3;//0.27;//0.3;//r * 3.0;//4.0;
        const double rdiv = r2 / sqrt( 2.0 );
        ippiCountInRange_32f_C1R( tmp1
                                , step
                                , roiSize
                                , &cnt
                                , 0.0f
                                , (Ipp32f)( r2 * r2 ) );
        ippiCountInRange_32f_C1R( tmp1
                                , step
                                , roiSize
                                , &cnt2
                                , 0.0f
                                , (Ipp32f)( rdiv * rdiv ) );
        *p_variance1 = 1.0f - (Ipp32f)cnt2 / (Ipp32f)cnt;
    }
    {
        int cnt2;
        const double r2 = 0.15;//0.17;//0.20;//0.15;//r * 3.0;//4.0;
        const double rdiv = r2 / sqrt( 2.0 );
        ippiCountInRange_32f_C1R( tmp1
                                , step
                                , roiSize
                                , &cnt
                                , 0.0f
                                , (Ipp32f)( r2 * r2 ) );
        ippiCountInRange_32f_C1R( tmp1
                                , step
                                , roiSize
                                , &cnt2
                                , 0.0f
                                , (Ipp32f)( rdiv * rdiv ) );
        *p_variance2 = 1.0f - (Ipp32f)cnt2 / (Ipp32f)cnt;
    }
#else
    for( vector<double>::iterator itVar = p_variance->begin(); itVar != p_variance->end(); ++itVar ) {
        int cnt2;
        const double r2 = *itVar;//0.15 + (double)i * 0.01;
        const double rdiv = r2 / sqrt( 2.0 );
        //cerr << "itVar=" << *itVar;
        ippiCountInRange_32f_C1R( tmp1
                                , step
                                , roiSize
                                , &cnt
                                , 0.0f
                                , (Ipp32f)( r2 * r2 ) );
        ippiCountInRange_32f_C1R( tmp1
                                , step
                                , roiSize
                                , &cnt2
                                , 0.0f
                                , (Ipp32f)( rdiv * rdiv ) );
        *itVar = 1.0f - (Ipp32f)cnt2 / (Ipp32f)cnt;
        //cerr << " -> " << *itVar << endl;
    }

#endif
    // 密度計算
    *p_density = 0.0f;//(float)( (float)cnt / ( M_PI * r * r ) );

    return cnt;
}
