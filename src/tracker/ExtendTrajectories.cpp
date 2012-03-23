/**
 * @file ExtendTrajectories
 * @brief 軌跡の延長処理に関する実装
 *
 * @author 福司 謙一郎
 * @date 2009
 */

//#include <mpi.h>
#include <vector>
#include <stdlib.h>
#include <ipp.h>
#include "track.h"

using namespace std;

/**
 * 複数の位置・速度情報
 */
typedef struct _PositionVelocity {
public:
    vector<Ipp32f> PositionX; // X座標
    vector<Ipp32f> PositionY; // Y座標
    vector<Ipp32f> VelocityX; // 速度のX成分
    vector<Ipp32f> VelocityY; // 速度のY成分
    vector<unsigned int> ID; // 色情報に基づくID
public:
    _PositionVelocity& operator=( const vector<LUMSlice>& obj ) {
        size_t sizeLUM = obj.size();
        PositionX.resize( sizeLUM );
        PositionY.resize( sizeLUM );
        VelocityX.resize( sizeLUM );
        VelocityY.resize( sizeLUM );
        ID.resize( sizeLUM );
        vector<Ipp32f>::iterator itPosX, itPosY, itVelX, itVelY;
        vector<unsigned int>::iterator itID;
        itPosX = PositionX.begin();
        itPosY = PositionY.begin();
        itVelX = VelocityX.begin();
        itVelY = VelocityY.begin();
        itID = ID.begin();
        vector<LUMSlice>::const_iterator itLUMSlice = obj.begin();
        for( ; itLUMSlice != obj.end(); ++itLUMSlice, ++itPosX, ++itPosY, ++itVelX, ++itVelY, ++itID ) {
            *itPosX = (Ipp32f)itLUMSlice->pos.x;
            *itPosY = (Ipp32f)itLUMSlice->pos.y;
            *itVelX = (Ipp32f)itLUMSlice->gradX;
            *itVelY = (Ipp32f)itLUMSlice->gradY;
            *itID = (unsigned int)itLUMSlice->pos.ID;
        }

        return *this;
    }
} PositionVelocity;

/**
 * 指定した位置から半径distanceImpact[m]以内にある等速直線運動を取り出す
 * @param pSrc 参照元の等速直線運動情報
 * @param pDst 計算結果の格納先
 * @param x X座標[m]
 * @param y Y座標[m]
 * @param distance 指定範囲[m]
 * @param pBuffer バッファへのポインタ（バッファサイズはGetCloseLUMBufferSizeで取得）
 * @param 取り出した等速直線運動の数
 */
unsigned int GetCloseLUM( PositionVelocity* pSrc
                            , PositionVelocity* pDst
                            , Ipp32f x
                            , Ipp32f y
                            , unsigned int ID
                            , Ipp32f distance
                            , unsigned char* buffer );

/**
 * 入力元の位置･速度情報から次の時刻での位置を計算する
 */
void CalcNextPosition( PositionVelocity* pSrc, PositionVelocity* pDst, TIME_MICRO_SEC dt );

/**
 * GetCloseLUMの計算用に必要なバッファサイズを返す
 * @param nLUM 参照元の等速直線運動の情報数
 * @return バッファサイズ[byte]
 */
unsigned int GetCloseLUMBufferSize( size_t nLUM );

int ExtendTrajectories( TIME_MICRO_SEC startTime
                         , TIME_MICRO_SEC endingTime
                         , vector<TrajectoryElement>* pSrcDst
                         , const vector<LUMSlice>* pLUMSlice
                         , const PARAM_COMMON* pCommonParam
                         , const PARAM_MKTRAJECTORY* pMkTrajectoryParam )
{
    PositionVelocity lum; // 等速直線運動の情報を，各成分（メンバ変数）ごとの配列に展開したもの
    PositionVelocity lumClose; // 速度ベクトルの計算に利用する等速直線運動の情報
    PositionVelocity trajectoryPVAry; // 各軌跡要素の時刻startTimeでのXY座標と勾配を配列に展開したもの
    PositionVelocity trajectoryNextPVAry; // 各軌跡要素の時刻endingTimeでのXY座標と勾配を配列に展開したもの
    vector<bool> flgExtentionAllowed; // 軌跡の延長を許可するフラグ配列

    vector<double> vx, vy;
    vector<bool> flgStartPoint;

    //
    // LUMスライスをXY座標と勾配ごとの配列に展開
    lum = *pLUMSlice;

    //
    // 各軌跡要素の時刻startTimeでのXY座標と勾配ごとの配列に展開
    trajectoryPVAry.PositionX.reserve( pSrcDst->size() );
    trajectoryPVAry.PositionY.reserve( pSrcDst->size() );
    trajectoryPVAry.ID.reserve( pSrcDst->size() );
    vx.reserve( pSrcDst->size() );
    vy.reserve( pSrcDst->size() );
    flgStartPoint.reserve( pSrcDst->size() );
    vector<TrajectoryElement>::iterator itTrjElement = pSrcDst->begin();
    for( ; itTrjElement != pSrcDst->end(); ++itTrjElement ) {
        TrajectoryElement::iterator itPosXYTID;
        if( ( itPosXYTID = itTrjElement->find( PosXYT( 0.0, 0.0, startTime ) ) ) != itTrjElement->end() ) {
            trajectoryPVAry.PositionX.push_back( (Ipp32f)itPosXYTID->x );
            trajectoryPVAry.PositionY.push_back( (Ipp32f)itPosXYTID->y );
            trajectoryPVAry.ID.push_back( (unsigned int)itPosXYTID->ID );
            if( itPosXYTID != itTrjElement->begin() ) {
                TrajectoryElement::iterator itPrevPosXYT = itPosXYTID;
                advance( itPrevPosXYT, -1 );
                flgStartPoint.push_back( false );
                vx.push_back( ( itPosXYTID->x - itPrevPosXYT->x ) / ( (double)( itPosXYTID->t - itPrevPosXYT->t ) * 1.0e-6 ) );
                vy.push_back( ( itPosXYTID->y - itPrevPosXYT->y ) / ( (double)( itPosXYTID->t - itPrevPosXYT->t ) * 1.0e-6 ) );
            } else {
                flgStartPoint.push_back( true );
                vx.push_back( 0.0 );
                vy.push_back( 0.0 );
            }
        }
    }
    trajectoryPVAry.VelocityX.resize( trajectoryPVAry.PositionX.size() );
    trajectoryPVAry.VelocityY.resize( trajectoryPVAry.PositionY.size() );
    flgExtentionAllowed.resize( trajectoryPVAry.PositionY.size() );

    //cout << "　　　" << trajectoryPVAry.PositionY.size() << "[個]の軌跡の延長します" << endl << flush;

    //
    // 各軌跡要素の時刻startTimeでの位置に近い等速直線運動を用いて移動量（XY方向の勾配）を決める
    vector<Ipp32f>::iterator itPosX, itPosY, itVelX, itVelY;
    vector<unsigned int>::iterator itID;
    vector<bool>::iterator itFlgAllowed;
    vector<double>::iterator itVX, itVY;
    vector<bool>::iterator itFlgStartPoint;
    itPosX = trajectoryPVAry.PositionX.begin();
    itPosY = trajectoryPVAry.PositionY.begin();
    itVelX = trajectoryPVAry.VelocityX.begin();
    itVelY = trajectoryPVAry.VelocityY.begin();
    itID = trajectoryPVAry.ID.begin();
    itFlgAllowed = flgExtentionAllowed.begin();
    itVX = vx.begin();
    itVY = vy.begin();
    itFlgStartPoint = flgStartPoint.begin();
    unsigned char* buffer = new unsigned char[ GetCloseLUMBufferSize( pLUMSlice->size() ) ]; // 計算用バッファ
    for( ; itPosX != trajectoryPVAry.PositionX.end(); ++itPosX, ++itPosY, ++itVelX, ++itVelY, ++itID, ++itFlgAllowed, ++itVX, ++itVY, ++itFlgStartPoint ) {
        // 位置(*itPosX,*itPosY)から半径distanceImpact[m]以内にある等速直線運動の情報を取り出す
        size_t nLUMClose = GetCloseLUM( &lum, &lumClose, *itPosX, *itPosY, *itID, (Ipp32f)pMkTrajectoryParam->distanceImpact, buffer );

        if( nLUMClose > 0 ) {
            vector<int> idx( nLUMClose/*lumClose.PositionX.size()*/ );
            for( int i = 0; i < (int)idx.size(); ++i ) {
                idx[ i ] = i;
            }
            random_shuffle( idx.begin(), idx.end() );

            *itVelX = *itVelY = 0.0;
            *itFlgAllowed = false;
            for( vector<int>::iterator itIdx = idx.begin(); itIdx != idx.end(); ++itIdx ) {
                // 近傍に等速直線運動が存在する場合は，
                // 上で求めた等速直線運動からランダムに１つ選び出す

                double dvx = *itVX - lumClose.VelocityX[ *itIdx ];
                double dvy = *itVY - lumClose.VelocityY[ *itIdx ];
                if( !(*itFlgStartPoint) && sqrt( dvx * dvx + dvy * dvy ) > 1.5/*2.0*/ ) {
                    //cerr << "■違反方向！(" << sqrt( dvx * dvx + dvy * dvy ) << ")";
                    continue;
                }

                *itVelX = lumClose.VelocityX[ *itIdx ];
                *itVelY = lumClose.VelocityY[ *itIdx ];
                if( *itID == 0 ) {
                    *itID = lumClose.ID[ *itIdx ];
                }

                // 軌跡の延長が可能なので，flgExtentionAllowedをTRUEに設定する。
                *itFlgAllowed = true;
                break;

    //            cout << "　　　軌跡（"
    //                 << distance( flgExtentionAllowed.begin(), itFlgAllowed )
    //                 << "）を延長します。近傍に"
    //                 << nLUMClose
    //                 << "[個]の等速直線運動が存在します。"
    //                 << endl << flush;
            }
        } else {
            // 近傍に等速直線運動が１つも無いときは軌跡の延長を不許可
            *itVelX = *itVelY = 0.0;
            *itID = 0;
            *itFlgAllowed = false;
//            cout << "　　　軌跡（" << distance( flgExtentionAllowed.begin(), itFlgAllowed ) << "）を延長しません" << endl << flush;
        }
    }

    //
    // 計算した速度ベクトルに基づいて次の時刻での位置を計算する
    size_t nPosVec = trajectoryPVAry.PositionX.size();
    trajectoryNextPVAry.PositionX.resize( nPosVec );
    trajectoryNextPVAry.PositionY.resize( nPosVec );
    trajectoryNextPVAry.VelocityX.resize( nPosVec );
    trajectoryNextPVAry.VelocityY.resize( nPosVec );
    trajectoryNextPVAry.ID.resize( nPosVec );
    CalcNextPosition( &trajectoryPVAry, &trajectoryNextPVAry, endingTime - startTime );

    //
    // 新しい位置を各軌跡要素に保存する
    itPosX = trajectoryNextPVAry.PositionX.begin();
    itPosY = trajectoryNextPVAry.PositionY.begin();
    itID = trajectoryNextPVAry.ID.begin();
    itFlgAllowed = flgExtentionAllowed.begin();
    for( itTrjElement = pSrcDst->begin(); itTrjElement != pSrcDst->end(); ++itTrjElement ) {
        if( itTrjElement->find( PosXYT( 0.0, 0.0, startTime ) ) != itTrjElement->end() ) {
            if( *itFlgAllowed ) {
                itTrjElement->insert( PosXYTID( *itPosX, *itPosY, endingTime, *itID ) );
            }
            ++itFlgAllowed;
            ++itPosX;
            ++itPosY;
            ++itID;
        }
    }

    delete [] buffer;

    return TRUE;
}

unsigned int GetCloseLUMBufferSize( size_t nLUM ) {
    return nLUM * 4 * sizeof( Ipp32f );
}

unsigned int GetCloseLUM( PositionVelocity* pSrc
                            , PositionVelocity* pDst
                            , Ipp32f x
                            , Ipp32f y
                            , unsigned int ID
                            , Ipp32f distance
                            , unsigned char* buffer )
{
    size_t nLUM = pSrc->PositionX.size();

    pDst->PositionX.clear();
    pDst->PositionY.clear();
    pDst->VelocityX.clear();
    pDst->VelocityY.clear();
    pDst->ID.clear();

    if( nLUM == 0 ) {
        return 0;
    }

    int step = nLUM * sizeof( Ipp32f );
    Ipp32f* posX = (Ipp32f*)&(*pSrc->PositionX.begin());
    Ipp32f* posY = (Ipp32f*)&(*pSrc->PositionY.begin());
    Ipp32f* gradX = (Ipp32f*)&(*pSrc->VelocityX.begin());
    Ipp32f* gradY = (Ipp32f*)&(*pSrc->VelocityY.begin());
    unsigned int* lumID = (unsigned int*)&(*pSrc->ID.begin());
    Ipp32f* dx_sq = (Ipp32f*)buffer;
    Ipp32f* dy_sq = (Ipp32f*)( buffer + 1 * step );
    Ipp32f* d_sq = (Ipp32f*)( buffer + 2 * step );
    Ipp8u* IsInRange = (Ipp8u*)( buffer + 3 * step );
    IppiSize size;
    size.width = nLUM;
    size.height = 1;

    // d_sqに点(x,y)から各等速直線運動への距離の２乗が代入される
    ippiSubC_32f_C1R( posX
                    , step
                    , x
                    , dx_sq
                    , step
                    , size );
    ippiSubC_32f_C1R( posY
                    , step
                    , y
                    , dy_sq
                    , step
                    , size );
    ippiSqr_32f_C1IR( dx_sq
                    , step
                    , size );
    ippiSqr_32f_C1IR( dy_sq
                    , step
                    , size );
    ippiAdd_32f_C1R( dx_sq
                   , step
                   , dy_sq
                   , step
                   , d_sq
                   , step
                   , size );

    // 距離の２乗がdistance^2以下のものを取り出してpDstに格納する
    ippiCompareC_32f_C1R( d_sq
                        , step
                        , distance * distance
                        , IsInRange
                        , nLUM * sizeof( Ipp8u )
                        , size
                        , ippCmpLessEq );
    Ipp64f nLUMInRange;
    ippiSum_8u_C1R( IsInRange, step, size, &nLUMInRange );
    nLUMInRange /= (Ipp64f)IPP_MAX_8U;
    size_t cntLUM = 0;
    if( pDst ) {
//        pDst->PositionX.resize( (size_t)nLUMInRange );
//        pDst->PositionY.resize( (size_t)nLUMInRange );
//        pDst->VelocityX.resize( (size_t)nLUMInRange );
//        pDst->VelocityY.resize( (size_t)nLUMInRange );
//        pDst->ID.resize( (size_t)nLUMInRange );
//        vector<Ipp32f>::iterator itPosX, itPosY, itVelX, itVelY;
//        vector<unsigned int>::iterator itID;
//        itPosX = pDst->PositionX.begin();
//        itPosY = pDst->PositionY.begin();
//        itVelX = pDst->VelocityX.begin();
//        itVelY = pDst->VelocityY.begin();
//        itID = pDst->ID.begin();
//        for( cntLUM = 0; cntLUM < nLUM; ++cntLUM, ++posX, ++posY, ++gradX, ++gradY, ++lumID ) {
//            if( IsInRange[ cntLUM  ] && ( ( lumID == 0 ) || ( *lumID == ID ) ) ) {
//                *itPosX++ = *posX;
//                *itPosY++ = *posY;
//                *itVelX++ = *gradX;
//                *itVelY++ = *gradY;
//                *itID++ = *lumID;
//            }
//        }
        pDst->PositionX.reserve( (size_t)nLUM );
        pDst->PositionY.reserve( (size_t)nLUM );
        pDst->VelocityX.reserve( (size_t)nLUM );
        pDst->VelocityY.reserve( (size_t)nLUM );
        pDst->ID.reserve( (size_t)nLUM );
        for( size_t i = 0; i < nLUM; ++i, ++posX, ++posY, ++gradX, ++gradY, ++lumID ) {
            if( IsInRange[ i  ] && ( ( *lumID == 0 ) || ( ID == 0 ) || ( *lumID == ID ) ) ) {
                pDst->PositionX.push_back( *posX );
                pDst->PositionY.push_back( *posY );
                pDst->VelocityX.push_back( *gradX );
                pDst->VelocityY.push_back( *gradY );
                pDst->ID.push_back( *lumID );
                ++cntLUM;
            }
        }
    }

    return cntLUM;
}

void CalcNextPosition( PositionVelocity* pSrc, PositionVelocity* pDst, TIME_MICRO_SEC dt )
{
    size_t nPosVec = pSrc->PositionX.size();

    if( nPosVec == 0 ) {
        return;
    }

    int step = nPosVec * sizeof( Ipp32f );
    int stride = sizeof( Ipp32f );
    Ipp32f* posCurrentX = (Ipp32f*)&(*pSrc->PositionX.begin());
    Ipp32f* posCurrentY = (Ipp32f*)&(*pSrc->PositionY.begin());
    Ipp32f* gradCurrentX = (Ipp32f*)&(*pSrc->VelocityX.begin());
    Ipp32f* gradCurrentY = (Ipp32f*)&(*pSrc->VelocityY.begin());
    Ipp32f* posNextX = (Ipp32f*)&(*pDst->PositionX.begin());
    Ipp32f* posNextY = (Ipp32f*)&(*pDst->PositionY.begin());
    IppiSize size;
    size.width = nPosVec;
    size.height = 1;

    ippmMul_vc_32f( gradCurrentX
                  , stride
                  , (Ipp32f)dt
                  , posNextX
                  , stride
                  , nPosVec );
    ippmMul_vc_32f( gradCurrentY
                  , stride
                  , (Ipp32f)dt
                  , posNextY
                  , stride
                  , nPosVec );
    ippiAdd_32f_C1IR( posCurrentX
                    , step
                    , posNextX
                    , step
                    , size );
    ippiAdd_32f_C1IR( posCurrentY
                    , step
                    , posNextY
                    , step
                    , size );
    memcpy( (unsigned int*)&(*pDst->ID.begin())
          , (unsigned int*)&(*pSrc->ID.begin())
          , sizeof( unsigned int ) * pSrc->ID.size() );
}
