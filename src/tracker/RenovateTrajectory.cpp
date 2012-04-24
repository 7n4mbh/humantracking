/**
 * @file RenovateTrajectory.cpp
 * @brief 軌跡修復に関する処理
 *
 * @author 福司 謙一郎
 * @date 2009
 */

//#include <mpi.h>
#include <deque>
#include <algorithm>
#include <numeric>
#include <iterator>
#include <ipp.h>
#include "track.h"
#include "RenovateTrajectory.h"
//#include "PerformanceCounter.h"
//#include "MersenneRandomizer.h"

using namespace std;
//using namespace nyx::util::random;
//using namespace nyx::util::research;

void DivideIntoSections( TrajectoriesInfo* pInfoTrj, std::map<int,int>* pPointIdxToTrjNo, PARAM_RENOVATE_TRAJECTORY param )
{
    TrajectoriesInfo& infoTrj = *pInfoTrj;

    // 全てのセクションの軌跡を１つにまとめる
    //vector<Section>::iterator itSection = pSrcDst->begin();
    //for( ++itSection; itSection != pSrcDst->end(); ++itSection ) {
    //    section.trjElement.insert( section.trjElement.end()
    //                             , itSection->trjElement.begin()
    //                             , itSection->trjElement.end() );
    //}
    //pSrcDst->resize( 1 );

    // 軌跡情報の初期化
    infoTrj.section.clear();
    infoTrj.points.clear();
    infoTrj.trjPointIdx.clear();

    if( pInfoTrj->trjElement.empty() ) {
        return;
    }

    // 全ての軌跡に含まれる点をpointsに格納する
    int idxPoint = 0, idxTrj = 0;
    vector<TrajectoryElement>::iterator itTrj = infoTrj.trjElement.begin();
    for( ; itTrj != infoTrj.trjElement.end(); ++itTrj, ++idxTrj ) {
        TrajectoryWithPointIndex trjPointIdx;
        TrajectoryElement::iterator itPos = itTrj->begin();
        for( ; itPos != itTrj->end(); ++itPos ) {
            PosXYTID pos;
            pos.ID = idxPoint;
            pos.t = itPos->t;
            pos.x = itPos->x;
            pos.y = itPos->y;
            infoTrj.points.push_back( pos );
            trjPointIdx.push_back( PointIndex( itPos->t, idxPoint ) );
            (*pPointIdxToTrjNo)[ idxPoint ] = idxTrj;
            ++idxPoint;
        }
        infoTrj.trjPointIdx.push_back( trjPointIdx );
    }

    // pointsに含まれる点を時刻順にソート
    // trjPointIdxの点番号がpointsの添字に一致しなくなることに注意
    sort( infoTrj.points.begin(), infoTrj.points.end(), PosXYT_T_Less() );

    // trjPointIdxの点番号がpointsの添字に一致するよう補正
    vector<int> index( infoTrj.points.size() );
    for( size_t i = 0; i < infoTrj.points.size(); ++i ) {
        index[ infoTrj.points[ i ].ID ] = (int)i;
    }
    /*int*/ idxTrj = 0;
    vector<TrajectoryWithPointIndex>::iterator itTrjPointIdx = infoTrj.trjPointIdx.begin();
    for( ; itTrjPointIdx != infoTrj.trjPointIdx.end(); ++itTrjPointIdx ) {
        TrajectoryWithPointIndex::iterator itPointIdx = itTrjPointIdx->begin();
        for( ; itPointIdx != itTrjPointIdx->end(); ++ itPointIdx ) {
            itPointIdx->second = index[ itPointIdx->second ];
            infoTrj.points[ itPointIdx->second ].ID = idxTrj;
        }
        ++idxTrj;
    }

//    //
//    // ハッシュテーブル作成
//    infoTrj.tableHash1.clear();
//    infoTrj.tableHash2.clear();
//    infoTrj.tableHash3.clear();
//    infoTrj.tableHash4.clear();
//    for( size_t i = 0; i < infoTrj.points.size(); ++i ) {
//        long hash;
//
//        hash = hash1( infoTrj.points[ i ].x, infoTrj.points[ i ].y, param.territory );
//        infoTrj.tableHash1[ infoTrj.points[ i ].t ][ hash ].push_back( (int)i );
//
//        hash = hash2( infoTrj.points[ i ].x, infoTrj.points[ i ].y, param.territory );
//        infoTrj.tableHash2[ infoTrj.points[ i ].t ][ hash ].push_back( (int)i );
//
//        hash = hash3( infoTrj.points[ i ].x, infoTrj.points[ i ].y, param.territory );
//        infoTrj.tableHash3[ infoTrj.points[ i ].t ][ hash ].push_back( (int)i );
//
//        hash = hash4( infoTrj.points[ i ].x, infoTrj.points[ i ].y, param.territory );
//        infoTrj.tableHash4[ infoTrj.points[ i ].t ][ hash ].push_back( (int)i );
//    }
//
//    //
//    // 近接テーブル作成
//    infoTrj.tableNear.clear();
//    infoTrj.tableNear.SetSize( (int)infoTrj.trjPointIdx.size() );
//    for( size_t iTrj1 = 0; iTrj1 < infoTrj.trjPointIdx.size(); ++iTrj1 ) {
//        vector<int> idxNearPoint;
//        TrajectoryWithPointIndex::iterator it = infoTrj.trjPointIdx[ iTrj1 ].begin();
//        idxNearPoint.clear();
//        for( ; it != infoTrj.trjPointIdx[ iTrj1 ].end(); ++it ) {
//            // iTrj番目の軌跡の全点で，近傍にある点の番号をidxNearPointに列挙する
//            long hash;
//            PosXYTID pos = infoTrj.points[ it->second ];
//
//            hash = hash1( pos.x, pos.y, param.territory );
//            idxNearPoint.insert( idxNearPoint.end(), infoTrj.tableHash1[ pos.t ][ hash ].begin(), infoTrj.tableHash1[ pos.t ][ hash ].end() );
//
//            hash = hash2( pos.x, pos.y, param.territory );
//            idxNearPoint.insert( idxNearPoint.end(), infoTrj.tableHash2[ pos.t ][ hash ].begin(), infoTrj.tableHash2[ pos.t ][ hash ].end() );
//
//            hash = hash3( pos.x, pos.y, param.territory );
//            idxNearPoint.insert( idxNearPoint.end(), infoTrj.tableHash3[ pos.t ][ hash ].begin(), infoTrj.tableHash3[ pos.t ][ hash ].end() );
//
//            hash = hash4( pos.x, pos.y, param.territory );
//            idxNearPoint.insert( idxNearPoint.end(), infoTrj.tableHash4[ pos.t ][ hash ].begin(), infoTrj.tableHash4[ pos.t ][ hash ].end() );
//        }
//
//        // idxNearPointの点を含む軌跡全てに対しNearフラグを立てる
//        for( vector<int>::iterator itIdxNear = idxNearPoint.begin(); itIdxNear != idxNearPoint.end(); ++itIdxNear ) {
//            int iTrj2 = infoTrj.points[ *itIdxNear ].ID;
//            if( iTrj1 != iTrj2 ) {
//                //cout << "near! 時刻: " << infoTrj.points[ *itIdxNear ].t << endl;
//                infoTrj.tableNear[ iTrj1 ].set( iTrj2, true );
//                infoTrj.tableNear[ iTrj2 ].set( (int)iTrj1, true );
//            }
//        }
//    }

    //
    // ハッシュテーブル作成
    infoTrj.tableHash1.clear();
    infoTrj.tableHash2.clear();
    infoTrj.tableHash3.clear();
    infoTrj.tableHash4.clear();
    for( size_t i = 0; i < infoTrj.points.size(); ++i ) {
        long hash;

        hash = hash1( infoTrj.points[ i ].x, infoTrj.points[ i ].y, param.territory );
        infoTrj.tableHash1[ infoTrj.points[ i ].t ][ hash ].push_back( (int)i );

        hash = hash2( infoTrj.points[ i ].x, infoTrj.points[ i ].y, param.territory );
        infoTrj.tableHash2[ infoTrj.points[ i ].t ][ hash ].push_back( (int)i );

        hash = hash3( infoTrj.points[ i ].x, infoTrj.points[ i ].y, param.territory );
        infoTrj.tableHash3[ infoTrj.points[ i ].t ][ hash ].push_back( (int)i );

        hash = hash4( infoTrj.points[ i ].x, infoTrj.points[ i ].y, param.territory );
        infoTrj.tableHash4[ infoTrj.points[ i ].t ][ hash ].push_back( (int)i );
    }

    infoTrj.tableHashVeryNear1.clear();
    infoTrj.tableHashVeryNear2.clear();
    infoTrj.tableHashVeryNear3.clear();
    infoTrj.tableHashVeryNear4.clear();
    for( size_t i = 0; i < infoTrj.points.size(); ++i ) {
        long hash;

        hash = hash1( infoTrj.points[ i ].x, infoTrj.points[ i ].y, param.territoryVeryNear );
        infoTrj.tableHashVeryNear1[ infoTrj.points[ i ].t ][ hash ].push_back( (int)i );

        hash = hash2( infoTrj.points[ i ].x, infoTrj.points[ i ].y, param.territoryVeryNear );
        infoTrj.tableHashVeryNear2[ infoTrj.points[ i ].t ][ hash ].push_back( (int)i );

        hash = hash3( infoTrj.points[ i ].x, infoTrj.points[ i ].y, param.territoryVeryNear );
        infoTrj.tableHashVeryNear3[ infoTrj.points[ i ].t ][ hash ].push_back( (int)i );

        hash = hash4( infoTrj.points[ i ].x, infoTrj.points[ i ].y, param.territoryVeryNear );
        infoTrj.tableHashVeryNear4[ infoTrj.points[ i ].t ][ hash ].push_back( (int)i );
    }

    //
    // 近接テーブル作成
    infoTrj.tableNear.clear();
    infoTrj.tableNear.SetSize( (int)infoTrj.trjPointIdx.size() );
    for( size_t iTrj1 = 0; iTrj1 < infoTrj.trjPointIdx.size(); ++iTrj1 ) {
        vector<int> idxNearPoint;
        TrajectoryWithPointIndex::iterator it = infoTrj.trjPointIdx[ iTrj1 ].begin();
        idxNearPoint.clear();
        for( ; it != infoTrj.trjPointIdx[ iTrj1 ].end(); ++it ) {
            // iTrj番目の軌跡の全点で，近傍にある点の番号をidxNearPointに列挙する
            long hash;
            PosXYTID pos = infoTrj.points[ it->second ];

            hash = hash1( pos.x, pos.y, param.territory );
            idxNearPoint.insert( idxNearPoint.end(), infoTrj.tableHash1[ pos.t ][ hash ].begin(), infoTrj.tableHash1[ pos.t ][ hash ].end() );

            hash = hash2( pos.x, pos.y, param.territory );
            idxNearPoint.insert( idxNearPoint.end(), infoTrj.tableHash2[ pos.t ][ hash ].begin(), infoTrj.tableHash2[ pos.t ][ hash ].end() );

            hash = hash3( pos.x, pos.y, param.territory );
            idxNearPoint.insert( idxNearPoint.end(), infoTrj.tableHash3[ pos.t ][ hash ].begin(), infoTrj.tableHash3[ pos.t ][ hash ].end() );

            hash = hash4( pos.x, pos.y, param.territory );
            idxNearPoint.insert( idxNearPoint.end(), infoTrj.tableHash4[ pos.t ][ hash ].begin(), infoTrj.tableHash4[ pos.t ][ hash ].end() );
        }

        // idxNearPointの点を含む軌跡全てに対しNearフラグを立てる
        for( vector<int>::iterator itIdxNear = idxNearPoint.begin(); itIdxNear != idxNearPoint.end(); ++itIdxNear ) {
            int iTrj2 = infoTrj.points[ *itIdxNear ].ID;
            if( iTrj1 != iTrj2 ) {
                infoTrj.tableNear[ iTrj1 ].set( iTrj2, true );
                infoTrj.tableNear[ iTrj2 ].set( (int)iTrj1, true );
            }
        }
    }

    infoTrj.tableVeryNear.clear();
    infoTrj.tableVeryNear.SetSize( (int)infoTrj.trjPointIdx.size() );
    for( size_t iTrj1 = 0; iTrj1 < infoTrj.trjPointIdx.size(); ++iTrj1 ) {
        vector<int> idxNearPoint;
        TrajectoryWithPointIndex::iterator it = infoTrj.trjPointIdx[ iTrj1 ].begin();
        idxNearPoint.clear();
        for( ; it != infoTrj.trjPointIdx[ iTrj1 ].end(); ++it ) {
            // iTrj番目の軌跡の全点で，近傍にある点の番号をidxNearPointに列挙する
            long hash;
            PosXYTID pos = infoTrj.points[ it->second ];

            hash = hash1( pos.x, pos.y, param.territoryVeryNear );
            idxNearPoint.insert( idxNearPoint.end(), infoTrj.tableHashVeryNear1[ pos.t ][ hash ].begin(), infoTrj.tableHashVeryNear1[ pos.t ][ hash ].end() );

            hash = hash2( pos.x, pos.y, param.territoryVeryNear );
            idxNearPoint.insert( idxNearPoint.end(), infoTrj.tableHashVeryNear2[ pos.t ][ hash ].begin(), infoTrj.tableHashVeryNear2[ pos.t ][ hash ].end() );

            hash = hash3( pos.x, pos.y, param.territoryVeryNear );
            idxNearPoint.insert( idxNearPoint.end(), infoTrj.tableHashVeryNear3[ pos.t ][ hash ].begin(), infoTrj.tableHashVeryNear3[ pos.t ][ hash ].end() );

            hash = hash4( pos.x, pos.y, param.territoryVeryNear );
            idxNearPoint.insert( idxNearPoint.end(), infoTrj.tableHashVeryNear4[ pos.t ][ hash ].begin(), infoTrj.tableHashVeryNear4[ pos.t ][ hash ].end() );
        }

        // idxNearPointの点を含む軌跡全てに対しNearフラグを立てる
        for( vector<int>::iterator itIdxNear = idxNearPoint.begin(); itIdxNear != idxNearPoint.end(); ++itIdxNear ) {
            int iTrj2 = infoTrj.points[ *itIdxNear ].ID;
            if( iTrj1 != iTrj2 ) {
                //cout << "near! 時刻: " << infoTrj.points[ *itIdxNear ].t << endl;
                infoTrj.tableVeryNear[ iTrj1 ].set( iTrj2, true );
                infoTrj.tableVeryNear[ iTrj2 ].set( (int)iTrj1, true );
            }
        }
    }


    //
    // 各点に対し，接続可能な点を列挙する
    TIME_MICRO_SEC t = 0;
    vector<PosXYTID>::iterator itConnectFirst, itConnectLast;
    infoTrj.connectable.clear();
    infoTrj.connectable.resize( infoTrj.points.size() );
    for( vector<PosXYTID>::iterator itPoint = infoTrj.points.begin(); itPoint != infoTrj.points.end(); ++itPoint ) {
        size_t index = distance( infoTrj.points.begin(), itPoint );

        //
        // itPointからrange[usec]以内にある点を接続候補とする
        if( itPoint->t != t || itPoint == infoTrj.points.begin() ) {
            itConnectFirst = infoTrj.points.begin();
            itConnectLast = upper_bound( infoTrj.points.begin()
                                       , infoTrj.points.end()
                                       , PosXYTID( 0.0, 0.0, itPoint->t + param.termConnect, 0 )
                                       , PosXYT_T_Less() );
            t = itConnectFirst->t;
        }

        // itPointの後に同一IDの点があれば最も近い点とitPoint間の速度を求める
        ULONGLONG t_back = 0;
        double vx1 = 0.0, vy1 = 0.0;
        for( int idx = (int)index - 1; idx >= 0; --idx ) {
            if( infoTrj.points[ idx ].ID == itPoint->ID ) {
                t_back = infoTrj.points[ idx ].t;
                const double dt = (double)( itPoint->t - t_back );
                vx1 = ( itPoint->x - infoTrj.points[ idx ].x ) / dt;
                vy1 = ( itPoint->y - infoTrj.points[ idx ].y ) / dt;
                break;
            }
        }

        // itPointの前に接続できる点を列挙
        TIME_MICRO_SEC t_nearest = 0; // 同一IDをもつ接続先のうち最も近い点の時刻
        for( vector<PosXYTID>::iterator itConnect = itPoint; itConnect != itConnectLast; ++itConnect ) {
            if( itConnect->t > itPoint->t ) {
                bool flgConnectable;
                if(  t_back > 0 ) {
                    double v = sqrt( ( itPoint->x - itConnect->x ) * ( itPoint->x - itConnect->x )
                                   + ( itPoint->y - itConnect->y ) * ( itPoint->y - itConnect->y ) )
                             / ( (double)( itConnect->t - itPoint->t ) * 1.0e-6 );
                    flgConnectable = v < 5.0/*5.0*//*3.0*/;
                } else {
                    const double dt = (double)( itConnect->t - itPoint->t );
                    double dv, vx2, vy2;
                    vx2 = ( itConnect->x - itPoint->x ) / dt;
                    vy2 = ( itConnect->y - itPoint->y ) / dt;
                    dv = sqrt( ( vx2 - vx1 ) * ( vx2 - vx1 ) + ( vy2 - vy1 ) * ( vy2 - vy1 ) ); // should be corrected. Sometimes vx is used without initialization
                    flgConnectable = dv < 1.5;
                }
                if( flgConnectable ) {
                    infoTrj.connectable[ index ].push_back( (int)distance( infoTrj.points.begin(), itConnect ) );
                    if( itConnect->ID == itPoint->ID ) {
                        t_nearest = ( t_nearest != 0 ) ? min( t_nearest, itConnect->t ) : itConnect->t;
                    }
                }
            }
        }
        if( t_nearest > 0 ) {
            for( list<int>::iterator itIdx = infoTrj.connectable[ index ].begin(); itIdx != infoTrj.connectable[ index ].end(); ) {
                if( infoTrj.points[ *itIdx ].t > t_nearest ) {
                    itIdx = infoTrj.connectable[ index ].erase( itIdx );
                } else {
                    ++itIdx;
                }
            }
        }/* else {
            infoTrj.connectable[ index ].push_front( -1 );
        }*/
    }

    //
    // グループ化可能テーブル作成
    infoTrj.tableGroupable.clear();
    infoTrj.tableGroupable.SetSize( (int)infoTrj.trjPointIdx.size() );
    for( size_t iTrj1 = 0; iTrj1 < infoTrj.trjPointIdx.size(); ++iTrj1 ) {
        for( size_t iTrj2 = iTrj1 + 1; iTrj2 < infoTrj.trjPointIdx.size(); ++iTrj2 ) {
            TIME_MICRO_SEC minTime = max( infoTrj.trjPointIdx[ iTrj1 ].front().first, infoTrj.trjPointIdx[ iTrj2 ].front().first );
            TIME_MICRO_SEC maxTime = min( infoTrj.trjPointIdx[ iTrj1 ].back().first, infoTrj.trjPointIdx[ iTrj2 ].back().first );
            if( minTime > maxTime ) {
                TIME_MICRO_SEC tmp = minTime;
                minTime = maxTime;
                maxTime = tmp;
            }

            // 区間[minTime, maxTime)で軌跡間の移動が常に可能かどうか検証する
            TrajectoryWithPointIndex::iterator it, itLast;
            bool flgMovable = true;

            // iTrj1からiTrj2への移動可能性の検証
            it = lower_bound( infoTrj.trjPointIdx[ iTrj1 ].begin()
                            , infoTrj.trjPointIdx[ iTrj1 ].end()
                            , PointIndex( minTime, 0 )
                            , PointIndex_Less() );
            itLast = lower_bound( infoTrj.trjPointIdx[ iTrj1 ].begin()
                                , infoTrj.trjPointIdx[ iTrj1 ].end()
                                , PointIndex( maxTime, 0 )
                                , PointIndex_Less() );
            for( ; it != itLast && flgMovable; ++it ) {
                list<int>::iterator itIdx = infoTrj.connectable[ it->second ].begin();
                flgMovable = false;
                for( ; itIdx != infoTrj.connectable[ it->second ].end(); ++itIdx ) {
                    if( infoTrj.points[ *itIdx ].ID == iTrj2 ) {
                        flgMovable = true;
                        break;
                    }
                }
            }

            // iTrj1からiTrj2への移動可能性の検証
            it = lower_bound( infoTrj.trjPointIdx[ iTrj2 ].begin()
                            , infoTrj.trjPointIdx[ iTrj2 ].end()
                            , PointIndex( minTime, 0 )
                            , PointIndex_Less() );
            itLast = lower_bound( infoTrj.trjPointIdx[ iTrj2 ].begin()
                                , infoTrj.trjPointIdx[ iTrj2 ].end()
                                , PointIndex( maxTime, 0 )
                                , PointIndex_Less() );
            for( ; it != itLast && flgMovable; ++it ) {
                list<int>::iterator itIdx = infoTrj.connectable[ it->second ].begin();
                flgMovable = false;
                for( ; itIdx != infoTrj.connectable[ it->second ].end(); ++itIdx ) {
                    if( infoTrj.points[ *itIdx ].ID == iTrj1 ) {
                        flgMovable = true;
                        break;
                    }
                }
            }

            // グループ化可能テーブル更新
            if( flgMovable ) {
                infoTrj.tableGroupable[ iTrj1 ].set( (int)iTrj2, true );
                infoTrj.tableGroupable[ iTrj2 ].set( (int)iTrj1, true );
            }
        }
    }

    //
    // グループ化不可能テーブル作成
    infoTrj.tableNotGroupable.clear();
    infoTrj.tableNotGroupable.SetSize( (int)infoTrj.trjPointIdx.size() );
    for( size_t iTrj1 = 0; iTrj1 < infoTrj.trjPointIdx.size(); ++iTrj1 ) {
        for( size_t iTrj2 = iTrj1 + 1; iTrj2 < infoTrj.trjPointIdx.size(); ++iTrj2 ) {
            TIME_MICRO_SEC minTime = max( infoTrj.trjPointIdx[ iTrj1 ].front().first, infoTrj.trjPointIdx[ iTrj2 ].front().first );
            TIME_MICRO_SEC maxTime = min( infoTrj.trjPointIdx[ iTrj1 ].back().first, infoTrj.trjPointIdx[ iTrj2 ].back().first );

            if( minTime < maxTime ) {
                // 区間[minTime, maxTime)で軌跡間の移動が１か所でも不可能な部分があるかどうか検証する
                TrajectoryWithPointIndex::iterator it, itLast;
                bool flgImmovable = false;

                // iTrj1からiTrj2への移動可能性の検証
                it = lower_bound( infoTrj.trjPointIdx[ iTrj1 ].begin()
                                , infoTrj.trjPointIdx[ iTrj1 ].end()
                                , PointIndex( minTime, 0 )
                                , PointIndex_Less() );
                itLast = lower_bound( infoTrj.trjPointIdx[ iTrj1 ].begin()
                                    , infoTrj.trjPointIdx[ iTrj1 ].end()
                                    , PointIndex( maxTime, 0 )
                                    , PointIndex_Less() );
                for( ; it != itLast && !flgImmovable; ++it ) {
                    list<int>::iterator itIdx = infoTrj.connectable[ it->second ].begin();
                    flgImmovable = true;
                    for( ; itIdx != infoTrj.connectable[ it->second ].end(); ++itIdx ) {
                        if( infoTrj.points[ *itIdx ].ID == iTrj2 ) {
                            flgImmovable = false;
                            break;
                        }
                    }
                }

                // iTrj1からiTrj2への移動可能性の検証
                it = lower_bound( infoTrj.trjPointIdx[ iTrj2 ].begin()
                                , infoTrj.trjPointIdx[ iTrj2 ].end()
                                , PointIndex( minTime, 0 )
                                , PointIndex_Less() );
                itLast = lower_bound( infoTrj.trjPointIdx[ iTrj2 ].begin()
                                    , infoTrj.trjPointIdx[ iTrj2 ].end()
                                    , PointIndex( maxTime, 0 )
                                    , PointIndex_Less() );
                for( ; it != itLast && !flgImmovable; ++it ) {
                    list<int>::iterator itIdx = infoTrj.connectable[ it->second ].begin();
                    flgImmovable = true;
                    for( ; itIdx != infoTrj.connectable[ it->second ].end(); ++itIdx ) {
                        if( infoTrj.points[ *itIdx ].ID == iTrj1 ) {
                            flgImmovable = false;
                            break;
                        }
                    }
                }

                // グループ化可能テーブル更新
                if( flgImmovable ) {
                    infoTrj.tableNotGroupable[ iTrj1 ].set( (int)iTrj2, true );
                    infoTrj.tableNotGroupable[ iTrj2 ].set( (int)iTrj1, true );
                }
            }
        }
    }

    //
    // セクション分割
    // 隣接行列の作成
    CStatusTable adjmat;
    adjmat.SetSize( (int)infoTrj.trjElement.size() );
    IppiSize size;
    size.width = (int)adjmat[ 0 ].size();
    size.height = 1;
    for( int i = 0; i < (int)adjmat.size(); ++i ) {
        ippiOr_8u_C1R( &(infoTrj.tableGroupable[ i ][ 0 ])
                     , (int)infoTrj.tableGroupable[ i ].size()
                     , &(infoTrj.tableNear[ i ][ 0 ])
                     , (int)infoTrj.tableNear[ i ].size()
                     , &(adjmat[ i ][ 0 ])
                     , (int)adjmat[ i ].size()
                     , size );
    }

    // クラスタリング
    vector<int> listClass( adjmat.size(), -1 );
    int label = 0;
    for( int i = 0; i < (int)adjmat.size(); ++i ) {
        if( listClass[ i ] == -1 ) {
            // 未使用のラベルを選択する
            clustering( i, &listClass, label, adjmat );
            label++;
        }
    }

    infoTrj.section.resize( label );
    for( int i = 0; i < label; ++i ) {
        for( int iTrj = 0; iTrj < (int)infoTrj.trjElement.size(); ++iTrj ) {
            if( listClass[ iTrj ] == i ) {
                infoTrj.section[ i ].idxTrajectory.push_back( iTrj );
            }
        }
    }
}

/*
void MakeSet( int idxSection, TrajectoriesInfo* pInfoTrj, map<int,int>* pReserve )
{
    TrajectoriesInfo& infoTrj = *pInfoTrj;
    Section& section = infoTrj.section[ idxSection ];

    //
    // セクション情報の初期化
    section.trjGroup.clear();
    section.trjSet.clear();

    //
    // セクションに含まれる各軌跡についてグループを作成
    vector<int>::iterator itIdxTrj = section.idxTrajectory.begin();
    for( ; itIdxTrj != section.idxTrajectory.end(); ++itIdxTrj ) {
#if 0
        TrajectoryGroup grp;
        vector<TrajectoryGroup> vecgrp;
        vector<CStatusList> vecGrpBits;
        grouping_prepare( *itIdxTrj, *itIdxTrj, grp, NULL, &vecGrpBits, section.idxTrajectory, infoTrj.tableGroupable, infoTrj.tableNotGroupable );

        sort( vecGrpBits.begin(), vecGrpBits.end() );
        vecGrpBits.erase( unique( vecGrpBits.begin(), vecGrpBits.end() ), vecGrpBits.end() );

        CStatusList grpbit;
        grpbit.SetSize( (int)infoTrj.tableGroupable.size() );

        grouping( 0, grpbit, &vecgrp, vecGrpBits, infoTrj.tableGroupable, infoTrj.tableNotGroupable );

        section.trjGroup.insert( section.trjGroup.end(), vecgrp.begin(), vecgrp.end() );
#else
        // grouping_prepare -> gourpingが正しい極大なグループの作成方法だが，
        // grouping_prepareで作成されるグループvecgrpもあまり変わらない場合が多く，計算時間も短い。
        // しかし6near2_100msec.trjやmany1_100msec.trjのような軌跡では極大なグループが作成できず，
        // セットの数が多くなる。
        // 以下，grouping_prepareのみでグループを作る時の記述法を一応残しておく。

        TrajectoryGroup grp;
        vector<TrajectoryGroup> vecgrp;
        vector<CStatusList> vecGrpBits;
        grouping_prepare( *itIdxTrj, *itIdxTrj, grp, &vecgrp, &vecGrpBits, section.idxTrajectory, infoTrj.tableGroupable, infoTrj.tableNotGroupable, pReserve );

        vector<TrajectoryGroup>::iterator it = vecgrp.begin();
        for( ; it != vecgrp.end(); ++it ) {
            sort( it->begin(), it->end() );
            section.trjGroup.push_back( *it );
        }
#endif
    }

    //
    // 重複するグループを消去
    sort( section.trjGroup.begin(), section.trjGroup.end() );
    section.trjGroup.erase( unique( section.trjGroup.begin(), section.trjGroup.end() ), section.trjGroup.end() );

    // 各グループに含まれる点を列挙する
    section.groupToPoints.resize( section.trjGroup.size() );
    for( int i = 0; i < (int)section.trjGroup.size(); ++i ) {
        TrajectoryGroup::iterator it = section.trjGroup[ i ].begin();
        for( ; it != section.trjGroup[ i ].end(); ++it ) {
            section.groupToPoints[ i ].insert( section.groupToPoints[ i ].end(), infoTrj.trjPointIdx[ *it ].begin(), infoTrj.trjPointIdx[ *it ].end() );
        }
    }

    //
    // 軌跡の組み合わせを作成
//    cout << "　makeset()...";
//    double start_d, end_d;
//    start_d = MPI_Wtime();
    int nMinGrp = 0;
    for( int i = 0; i < (int)section.trjGroup.size(); ++i ) {
        TrajectorySet trjSet;
        vector<TrajectorySet> vecset;
        vector<int> idxTrj;
        makeset( i, trjSet, &vecset, idxTrj, section.trjGroup, infoTrj.trjPointIdx, nMinGrp );
        section.trjSet.insert( section.trjSet.end(), vecset.begin(), vecset.end() );

        //
        // テスト
        // 最小のグループ数で構成しているセットのみを残す
        vector<TrajectorySet> tmpSet = section.trjSet;
        section.trjSet.clear();
//        for( int i = 0; i < (int)tmpSet.size(); ++i ) {
//            if( nMinGrp <= 0 || (int)tmpSet[ i ].size() < nMinGrp ) {
//                nMinGrp = (int)tmpSet[ i ].size();
//            }
//        }
        for( int i = 0; i < (int)tmpSet.size(); ++i ) {
            if( (int)tmpSet[ i ].size() == nMinGrp ) {
                section.trjSet.push_back( tmpSet[ i ] );
            }
        }
    }
//    end_d = MPI_Wtime();
//    cout << ( end_d - start_d ) * 1.0e3 << "[msec]" << endl;


    //
    // 重複する組み合わせを消去
    sort( section.trjSet.begin(), section.trjSet.end() );
    section.trjSet.erase( unique( section.trjSet.begin(), section.trjSet.end() ), section.trjSet.end() );
}
*/

void MakeSet( int idxSection, TrajectoriesInfo* pInfoTrj, map<int,int>* pReserve )
{
    TrajectoriesInfo& infoTrj = *pInfoTrj;
    Section& section = infoTrj.section[ idxSection ];

    //
    // セクション情報の初期化
    section.trjGroup.clear();
    section.trjSet.clear();

    vector<TrajectoryGroup> subgroup;
//#define SUBGROUPING
#ifdef SUBGROUPING
    //cerr << "　　サブグループ作成...";
    //
    // セクションに含まれる各軌跡についてサブグループを作成
    vector<int>::iterator itIdxTrj = section.idxTrajectory.begin();
    for( ; itIdxTrj != section.idxTrajectory.end(); ++itIdxTrj ) {
        TrajectoryGroup grp;
        vector<TrajectoryGroup> vecgrp;
        vector<CStatusList> vecGrpBits;
        subgrouping( *itIdxTrj, *itIdxTrj, grp, &vecgrp, &vecGrpBits, section.idxTrajectory, infoTrj, pReserve );

        vector<TrajectoryGroup>::iterator it = vecgrp.begin();
        for( ; it != vecgrp.end(); ++it ) {
            sort( it->begin(), it->end() );
            subgroup.push_back( *it );
        }
    }

    // 重複するサブグループを消去
    sort( subgroup.begin(), subgroup.end() );
    subgroup.erase( unique( subgroup.begin(), subgroup.end() ), subgroup.end() );

    //cerr << "　　完了（" << subgroup.size() << "[個]）" << endl;
#else
    vector<int>::iterator itIdxTrj = section.idxTrajectory.begin();
    for( ; itIdxTrj != section.idxTrajectory.end(); ++itIdxTrj ) {
        TrajectoryGroup grp;
        grp.push_back( *itIdxTrj );
        subgroup.push_back( grp );
    }
#endif

    cerr << "　　グループ作成...";
    //
    // サブグループを統合してグループを作成
    for( int idxSubGrp = 0; idxSubGrp < (int)subgroup.size(); ++idxSubGrp ) {
        vector<int> subgrps;
        vector<TrajectoryGroup> vecgrp;
        vector<CStatusList> vecGrpBits;
        grouping( idxSubGrp, idxSubGrp, subgrps, &vecgrp, &vecGrpBits, subgroup, infoTrj.tableGroupable, infoTrj.tableNotGroupable, pReserve );

        vector<TrajectoryGroup>::iterator it = vecgrp.begin();
        for( ; it != vecgrp.end(); ++it ) {
            sort( it->begin(), it->end() );
            section.trjGroup.push_back( *it );
        }
    }

#define CHOOSE_GROUP
#ifdef CHOOSE_GROUP
    //
    // 重複するグループを消去
    sort( section.trjGroup.begin(), section.trjGroup.end() );
    section.trjGroup.erase( unique( section.trjGroup.begin(), section.trjGroup.end() ), section.trjGroup.end() );

    //cerr << "　　完了（" << section.trjGroup.size() << "[個]）" << endl;


    //
    // 各グループの最大の速度変化を求める
    vector<double> max_acc;
    int idxGrp = 0;
    for( vector<TrajectoryGroup>::iterator itTrjGrp = section.trjGroup.begin(); itTrjGrp != section.trjGroup.end(); ++itTrjGrp, ++idxGrp ) {
        multiset<PosXYT,PosXYT_T_Less> elements;
        //CTrajectory trj;
        for( TrajectoryGroup::iterator itIdxTrj = itTrjGrp->begin(); itIdxTrj != itTrjGrp->end(); ++itIdxTrj ) {
            elements.insert( infoTrj.trjElement[ *itIdxTrj ].begin(), infoTrj.trjElement[ *itIdxTrj ].end() );
            //trj.push_back( infoTrj.trjElement[ *itIdxTrj ] );
        }

        //TrajectoryElement trjElement;
        deque<PosXYT> trjPos;
        double maxacc = -1.0;
        multiset<PosXYT,PosXYT_T_Less>::iterator itBegin, itEnd;
        itBegin = elements.begin();
        while( itBegin != elements.end() ) {
            itEnd = elements.upper_bound( PosXYT( 0.0, 0.0, itBegin->t ) );
            PosXYTID pos( 0.0, 0.0, itBegin->t, 0 );
            unsigned int n = 0;
            for( ; itBegin != itEnd; ++itBegin, ++n ) {
                pos.x += itBegin->x;
                pos.y += itBegin->y;
            }
            pos.x /= n;
            pos.y /= n;
            //trjElement.insert( pos );
            trjPos.push_back( pos );
            if( trjPos.size() == 3 ) {
                double dx1 = trjPos[ 0 ].x - trjPos[ 1 ].x;
                double dy1 = trjPos[ 0 ].y - trjPos[ 1 ].y;
                double dx2 = trjPos[ 1 ].x - trjPos[ 2 ].x;
                double dy2 = trjPos[ 1 ].y - trjPos[ 2 ].y;
                double dv1 = sqrt( dx1 * dx1 + dy1 * dy1 );
                double dv2 = sqrt( dx2 * dx2 + dy2 * dy2 );
                double acc = abs( dv1 - dv2 );
                if( maxacc < 0.0 || acc > maxacc ) {
                    maxacc = acc;
                }
                trjPos.pop_front();
            }
        }

        //max_acc.push_back( maxacc );
        const double ratioUnusedTrj = 1.0 - (double)itTrjGrp->size() / (double)infoTrj.trjElement.size();
        const double ratioRange = 1.0 - ( ( (double)( elements.rbegin()->t - elements.begin()->t ) * 1.0e-6 ) / 10.0/*8.5*/ );
        max_acc.push_back( 1.5/*2.0*//*0.75*/ * maxacc /*+ 0.85 * ratioUnusedTrj*/ + ratioRange );
    }

    //
    // グループの滑らかさ（ここでは最大の速度変化）でグループを絞り込む
    map< int,multimap<double,int> > idxTrjToIdxGrp; // 「軌跡番号」から「その軌跡を含むグループの滑らかさ及び番号」へのマップ
    idxGrp = 0;
    for( vector<TrajectoryGroup>::iterator itTrjGrp = section.trjGroup.begin(); itTrjGrp != section.trjGroup.end(); ++itTrjGrp, ++idxGrp ) {
        for( TrajectoryGroup::iterator itIdxTrj = itTrjGrp->begin(); itIdxTrj != itTrjGrp->end(); ++itIdxTrj ) {
            idxTrjToIdxGrp[ *itIdxTrj ].insert( pair<double,int>( max_acc[ idxGrp ], idxGrp ) );
        }
    }

    vector<TrajectoryGroup> newTrjGrp;
    for( map< int,multimap<double,int> >::iterator itIdxGrp = idxTrjToIdxGrp.begin(); itIdxGrp != idxTrjToIdxGrp.end(); ++itIdxGrp ) {
        const int idxTrj = itIdxGrp->first;
        int cnt = 0;
        for( multimap<double,int>::iterator itAcc = itIdxGrp->second.begin(); itAcc != itIdxGrp->second.end() && cnt < 10/*5*/; ++itAcc, ++cnt ) {
            const int idxGrp = itAcc->second;
            newTrjGrp.push_back( section.trjGroup[ idxGrp ] );
        }
    }

    section.trjGroup = newTrjGrp;
#endif

    // 最小単位のグループ（サブグループ単体）を追加
    for( vector<TrajectoryGroup>::iterator itSubGrp = subgroup.begin(); itSubGrp != subgroup.end(); ++itSubGrp ) {
        section.trjGroup.push_back( *itSubGrp );
    }

    //
    // 重複するグループを消去
    sort( section.trjGroup.begin(), section.trjGroup.end() );
    section.trjGroup.erase( unique( section.trjGroup.begin(), section.trjGroup.end() ), section.trjGroup.end() );

    //cerr << "　　完了（" << section.trjGroup.size() << "[個]）" << endl;

    //
    // 各グループに含まれる点を列挙する
    section.groupToPoints.resize( section.trjGroup.size() );
    for( int i = 0; i < (int)section.trjGroup.size(); ++i ) {
        TrajectoryGroup::iterator it = section.trjGroup[ i ].begin();
        for( ; it != section.trjGroup[ i ].end(); ++it ) {
            section.groupToPoints[ i ].insert( section.groupToPoints[ i ].end(), infoTrj.trjPointIdx[ *it ].begin(), infoTrj.trjPointIdx[ *it ].end() );
        }
    }

    //cerr << "　　包含関係の調査...";
    //
    // 包含関係を調べる
    vector<CStatusList> grpBits;
    for( int idxGrp = 0; idxGrp < (int)section.trjGroup.size(); ++idxGrp ) {
        CStatusList bit;
        bit.SetSize( (int)infoTrj.trjElement.size() );
        for( TrajectoryGroup::iterator it = section.trjGroup[ idxGrp ].begin(); it != section.trjGroup[ idxGrp ].end(); ++it ) {
            bit.set( *it, true );
        }
        grpBits.push_back( bit );
    }
    vector< vector<GroupCombiniation> > idxGrpToCmb( section.trjGroup.size() );
    vector< vector<CStatusList> > idxGrpToCmbBit( section.trjGroup.size() );
    GroupCombiniation grpCmb;
    CStatusList bit;
    bit.SetSize( (int)infoTrj.trjElement.size() );
    for( int idxGrp = 0; idxGrp < (int)section.trjGroup.size(); ++idxGrp ) {
        inclusive_relation( &idxGrpToCmb[ idxGrp ]
                          , idxGrp
                          , grpCmb
                          , bit
                          , section.trjGroup
                          , grpBits );
        sort( idxGrpToCmb[ idxGrp ].begin(), idxGrpToCmb[ idxGrp ].end() );
        idxGrpToCmb[ idxGrp ].erase( unique( idxGrpToCmb[ idxGrp ].begin(), idxGrpToCmb[ idxGrp ].end() )
                                   , idxGrpToCmb[ idxGrp ].end() );
        vector<GroupCombiniation>::iterator itCmb = idxGrpToCmb[ idxGrp ].begin();
        for( ; itCmb != idxGrpToCmb[ idxGrp ].end(); ++itCmb ) {
            CStatusList cmbBit;
            cmbBit.SetSize( (int)section.trjGroup.size() );
            for( int i = 0; i < (int)itCmb->size(); ++i ) {
                cmbBit.set( (*itCmb)[ i ], true );
            }
            idxGrpToCmbBit[ idxGrp ].push_back( cmbBit );
        }
    }
    //cerr << "　　完了" << endl;

    //cerr << "　　組み合わせ作成...";
    //
    // 軌跡の組み合わせを作成
    vector<TrajectorySet> trjSet;
    TrajectorySet tmpset;
    CStatusList tmpsetbit;
    set<CStatusList> setSearched;
    tmpsetbit.SetSize( (int)section.trjGroup.size() );
    for( int idxGrp = 0; idxGrp < (int)section.trjGroup.size(); ++idxGrp ) {
        if( section.trjGroup[ idxGrp ].size() == 1 ) {
            tmpset.push_back( idxGrp );
            tmpsetbit.set( idxGrp, true );
        }
    }
    makeset( &trjSet, 0, tmpset, tmpsetbit, setSearched, idxGrpToCmb, idxGrpToCmbBit, section.trjGroup );


    // 重複する組み合わせを消去
    sort( trjSet.begin(), trjSet.end() );
    trjSet.erase( unique( trjSet.begin(), trjSet.end() ), trjSet.end() );
    //cerr << "　　完了（" << trjSet.size() << "[個]）" << endl;


    //cerr << "　　最小のグループ数のセット抽出...";
    //
    // 最小のグループ数のセットを取り出す
    int min;
    vector<int> nGrp;
    for( int i = 0; i < (int)trjSet.size(); ++i ) {
        nGrp.push_back( (int)trjSet[ i ].size() );
    }
    min = *min_element( nGrp.begin(), nGrp.end() );

    vector<TrajectorySet> proSet;
    for( vector<TrajectorySet>::iterator itSet = trjSet.begin(); itSet != trjSet.end(); ++itSet ) {
        if( itSet->size() == min ) {
            proSet.push_back( *itSet );
        }
    }
    //cerr << "　　完了（" << proSet.size() << "[個]）" << endl;

#define RECTIFY_SET
#ifdef RECTIFY_SET
//    //
//    // 不正なグループの組み合わせを除去し，セットを完成させる
//    section.trjSet.clear();
//    for( vector<TrajectorySet>::iterator itSet = proSet.begin(); itSet != proSet.end(); ++itSet ) {
//        rectifyset( &section.trjSet, *itSet, section.trjGroup, infoTrj.tableNotGroupable, infoTrj.tableNear, *pReserve );
//    }
//
//    // 重複する組み合わせを消去
//    sort( section.trjSet.begin(), section.trjSet.end() );
//    section.trjSet.erase( unique( section.trjSet.begin(), section.trjSet.end() ), section.trjSet.end() );


    cerr << "　　rectifyset...";
    //
    // 不正なグループの組み合わせを除去し，セットを完成させる
    trjSet.clear();
    for( vector<TrajectorySet>::iterator itSet = proSet.begin(); itSet != proSet.end(); ++itSet ) {
        TrajectorySet initSet;
        rectifyset( &trjSet, initSet, *itSet, section.trjGroup, infoTrj.tableNotGroupable, infoTrj.tableNear, infoTrj.tableVeryNear, *pReserve, idxGrpToCmb, idxGrpToCmbBit );
    }

    if( trjSet.empty() ) {
        for( vector<TrajectorySet>::iterator itSet = proSet.begin(); itSet != proSet.end(); ++itSet ) {
            TrajectorySet initSet;
            rectifyset( &trjSet, initSet, *itSet, section.trjGroup, infoTrj.tableNotGroupable, infoTrj.tableNear, infoTrj.tableVeryNear, *pReserve, idxGrpToCmb, idxGrpToCmbBit, false );
        }
    }

    // 重複する組み合わせを消去
    sort( trjSet.begin(), trjSet.end() );
    trjSet.erase( unique( trjSet.begin(), trjSet.end() ), trjSet.end() );
    //
    // 最大のグループ数のセットを取り出す
    int max;
    nGrp.clear();
    for( int i = 0; i < (int)trjSet.size(); ++i ) {
        nGrp.push_back( (int)trjSet[ i ].size() );
    }
    max = *max_element( nGrp.begin(), nGrp.end() );

    section.trjSet.clear();
    for( vector<TrajectorySet>::iterator itSet = trjSet.begin(); itSet != trjSet.end(); ++itSet ) {
        if( itSet->size() == max ) {
            section.trjSet.push_back( *itSet );
        }
    }

#else
    section.trjSet = proSet;
#endif

//        end_d = MPI_Wtime();
//        cout << ( end_d - start_d ) * 1.0e3 << "[msec]" << endl;
}

//double Optimize( vector<TrajectoryElement>* pDst, std::vector<int>* pDstID, std::map<int,int>* pReserve, int idxSection, int idxSet, int nStart, TrajectoriesInfo* pInfoTrj, PARAM_RENOVATE_TRAJECTORY param, double* p_t1, double* p_t2, double* p_t3 )
//{
//    TrajectoriesInfo& infoTrj = *pInfoTrj;
//    Section& section = infoTrj.section[ idxSection ];
//    MersenneRandomizer mr( 0, RAND_MAX );
//    //PerformanceCounter cpc;
//
//    vector<TIME_MICRO_SEC> time;
//
//    // cout << "　　セット: " << idxSet << endl;
//
//    //
//    // idxSet番目のセットに含まれない軌跡の中に，pReserveで指定した軌跡が１つでもあれば解なし。
//
//    // idxSet番目のセットに含まれるグループ中の軌跡(idxTrjInThisSet)と予約軌跡idxRsvTrjを列挙。
//    vector<int> idxRsvTrj, idxTrjInThisSet;
//    for( map<int,int>::iterator itRsvTrj = pReserve->begin(); itRsvTrj != pReserve->end(); ++itRsvTrj ) {
//        idxRsvTrj.push_back( itRsvTrj->first );
//    }
//    for( vector<int>::iterator itIdxGrp = section.trjSet[ idxSet ].begin(); itIdxGrp != section.trjSet[ idxSet ].end(); ++itIdxGrp ) {
//        TrajectoryGroup::iterator itIdxTrj = section.trjGroup[ *itIdxGrp ].begin();
//        for( ; itIdxTrj != section.trjGroup[ *itIdxGrp ].end(); ++itIdxTrj ) {
//            idxTrjInThisSet.push_back( *itIdxTrj );
//        }
//    }
//    sort( idxTrjInThisSet.begin(), idxTrjInThisSet.end() );
//
//    // idxSection番目のセクションの軌跡のうち，セットに含まれない軌跡(idxTrjOutOfThisSet)を列挙
//    vector<int> idxTrjOutOfThisSet;
//    set_difference( section.idxTrajectory.begin()
//                  , section.idxTrajectory.end()
//                  , idxTrjInThisSet.begin()
//                  , idxTrjInThisSet.end()
//                  , inserter( idxTrjOutOfThisSet, idxTrjOutOfThisSet.begin() ) );
//    // cout << "　　セットに含まれない軌跡: ";
//    for( vector<int>::iterator it = idxTrjOutOfThisSet.begin(); it != idxTrjOutOfThisSet.end(); ++it ) {
//        // cout << *it << " ";
//    }
//    // cout << endl;
//
//    // idxTrjOutOfThisSetの中にidxRsvTrjが１つでもあれば解なし。
//    vector<int> idxRsvTrjOutOfThisSet;
//    set_intersection( idxTrjOutOfThisSet.begin()
//                    , idxTrjOutOfThisSet.end()
//                    , idxRsvTrj.begin()
//                    , idxRsvTrj.end()
//                    , inserter( idxRsvTrjOutOfThisSet, idxRsvTrjOutOfThisSet.begin() ) );
//    if( !idxRsvTrjOutOfThisSet.empty() ) {
//        // cout << "　　解なし(セクション: " << idxSection << ", セット: " << idxSet << ")" << endl;
//        return -1.0;
//    }
//
//    // 予約軌跡の点番号からIDへのマップを作成
//    // cout << "　　idxPointToID: ";
//    map<int,int> idxPointToID;
//    for( map<int,int>::iterator itRsvTrj = pReserve->begin(); itRsvTrj != pReserve->end(); ++itRsvTrj ) {
//        int idxTrj = itRsvTrj->first;
//        TrajectoryWithPointIndex::iterator it = infoTrj.trjPointIdx[ idxTrj ].begin();
//        for( ; it !=  infoTrj.trjPointIdx[ idxTrj ].end(); ++it ) {
//            int idxPoint = it->second;
//            int id = itRsvTrj->second;
//            idxPointToID[ idxPoint ] = id;
//            // cout << "( " << idxPoint << ", " << id << " ), ";
//        }
//    }
//    // cout << endl;
//
//    //
//    // 各グループにおける予約軌跡の点番号を求める。
//    // cout << "　　予約軌跡: ";
//    for( vector<int>::iterator it = idxRsvTrj.begin(); it != idxRsvTrj.end(); ++it ) {
//        // cout << *it << " ";
//    }
//    // cout << endl;
//    //map<int,int> grpToID;
//    map<int,TimeToPointIndex> grpToIdxRsvPoint;
//    for( vector<int>::iterator itIdxGrp = section.trjSet[ idxSet ].begin(); itIdxGrp != section.trjSet[ idxSet ].end(); ++itIdxGrp ) {
//        // cout << "　　グループ" << *itIdxGrp << "の軌跡: ";
//        for( vector<int>::iterator it = section.trjGroup[ *itIdxGrp ].begin(); it != section.trjGroup[ *itIdxGrp ].end(); ++it ) {
//            // cout << *it << " ";
//        }
//        // cout << endl;
//        vector<int> idxRsvTrjOfThisGrp; // itIdxGrp番目のグループに含まれている予約軌跡
//        set_intersection( section.trjGroup[ *itIdxGrp ].begin()
//                        , section.trjGroup[ *itIdxGrp ].end()
//                        , idxRsvTrj.begin()
//                        , idxRsvTrj.end()
//                        , inserter( idxRsvTrjOfThisGrp, idxRsvTrjOfThisGrp.begin() ) );
//        // 予約軌跡は同一グループに入らないので，idxRsvTrjOfThisGrpの要素数は0または1
//        if( idxRsvTrjOfThisGrp.empty() ) {
//            //grpToID[ *itIdxGrp ] = -1;
//        } else {
//            //grpToID[ *itIdxGrp ] = (*pReserve)[ idxRsvTrjOfThisGrp.front() ];
//            grpToIdxRsvPoint[ *itIdxGrp ].insert( infoTrj.trjPointIdx[ idxRsvTrjOfThisGrp.front() ].begin()
//                                                , infoTrj.trjPointIdx[ idxRsvTrjOfThisGrp.front() ].end() );
//            //// cout << "　　ID割り当て: グループ" << (int)(*itIdxGrp) << " -> " << (int)( (*pReserve)[ idxRsvTrjOfThisGrp.front() ] ) << endl;
//        }
//    }
//
//
//    //
//    // idxSet番目のセットに含まれない軌跡(idxTrjOutOfThisSet)の点の分のエネルギーを計算
//    double energyExPoints;
//    size_t nExPoints = 0;
//    for( vector<int>::iterator itIdxTrj = idxTrjOutOfThisSet.begin(); itIdxTrj != idxTrjOutOfThisSet.end(); ++itIdxTrj ) {
//        nExPoints = infoTrj.trjElement[ *itIdxTrj ].size();
//    }
//    energyExPoints = (double)nExPoints * param.lambda3;
//
//
//    //
//    // idxSet番目のセット内のグループの，各時刻での選択肢（点番号）を列挙
////    // cout << "　点番号列挙...";
//    map<int,TimeToPointChoice> choiceOfGroup;
//    TrajectorySet::iterator itIdxGrp = section.trjSet[ idxSet ].begin();
//    for( ; itIdxGrp != section.trjSet[ idxSet ].end(); ++itIdxGrp ) {
//        TrajectoryWithPointIndex::iterator itIdxPoint = section.groupToPoints[ *itIdxGrp ].begin();
//        for( ; itIdxPoint != section.groupToPoints[ *itIdxGrp ].end(); ++itIdxPoint ) {
//            choiceOfGroup[ *itIdxGrp ].insert( *itIdxPoint );
//            time.push_back( itIdxPoint->first );
//        }
//    }
////    // cout << "完了" << endl;
//
//    sort( time.begin(), time.end() );
//    time.erase( unique( time.begin(), time.end() ), time.end() );
//
//    //
//    // choiceOfGroupを用いて，全ての時刻における各グループの点番号の組み合わせを作成する。
//    // 組み合わせの表現は，選択肢リストと選択肢ベクトルの２通りありともに同一の内容を表す。
//    //map<TIME_MICRO_SEC,ChoiceList> timeToChoiceList;
//    vector<ChoiceList> idxTimeToChoiceList( time.size() ); // 各時刻に対する選択肢リスト
//    vector<ChoiceVector> idxTimeToPointChoiceVector( time.size() ); // 各時刻に対する選択肢ベクトル
//    //vector<TIME_MICRO_SEC>::iterator itTime = time.begin();
//    //for( int i = 0; itTime != time.end(); ++itTime, ++i ) {
//    for( int i = 0; i < (int)time.size(); ++i ) {
//        map<int,int> grpToPointIdx;
//        HashTable tblHash;
//        makechoicelist( choiceOfGroup.begin()
//                      , choiceOfGroup.end()
//                      , time[ i ] //*itTime
//                      , &idxTimeToChoiceList[ i ]
//                      , grpToPointIdx
//                      , tblHash
//                      , tblHash
//                      , tblHash
//                      , tblHash
//                      , *pInfoTrj
//                      , grpToIdxRsvPoint
//                      , param.territory );
//
//        if( ( idxTimeToPointChoiceVector[ i ].nChoice = (int)idxTimeToChoiceList[ i ].size() ) != 0 ) {
//            for( map<int,int>::iterator it = idxTimeToChoiceList[ i ].front().begin(); it != idxTimeToChoiceList[ i ].front().end(); ++it ) {
//                idxTimeToPointChoiceVector[ i ].modifyGroup.push_back( it->first );
//            }
//
//            for( ChoiceList::iterator itChoice = idxTimeToChoiceList[ i ].begin(); itChoice != idxTimeToChoiceList[ i ].end(); ++itChoice ) {
//                vector<int>::iterator itIdxGrp = idxTimeToPointChoiceVector[ i ].modifyGroup.begin();
//                for( ; itIdxGrp != idxTimeToPointChoiceVector[ i ].modifyGroup.end(); ++itIdxGrp ) {
//                    idxTimeToPointChoiceVector[ i ].data.push_back( PointIndex( time[ i ], (*itChoice)[ *itIdxGrp ] ) );
//                }
//            }
//        }
//    }
//
//    //
//    // connectableテーブル作成
//    CStatusTable tblConnectable;
//    tblConnectable.SetSize( (int)infoTrj.points.size() );
//    for( int i = 0; i < (int)infoTrj.points.size(); ++i ) {
//        for( list<int>::iterator it = infoTrj.connectable[ i ].begin(); it != infoTrj.connectable[ i ].end(); ++it ) {
//            tblConnectable[ i ].set( *it, true );
//        }
//    }
//
////    // cout << "　初期状態作成...";
//
//    // nStart個の初期状態を生成する
//    PARAM_CALC_ENERGY param_calc_energy;
//    BUFFER_CALC_ENERGY buffer;
//    vector<OptVar> optvar( nStart );
//    for( vector<OptVar>::iterator itOptVar = optvar.begin(); itOptVar != optvar.end(); ++itOptVar ) {
//        //
//        // 各時刻の点番号の組み合わせをランダムに設定する
//        //map<int,TimeToPointIndex> grpToStatus; // 各グループの点番号の選択状況
//        vector<ChoiceList>::iterator itListChoice = idxTimeToChoiceList.begin();
//        for( ; itListChoice != idxTimeToChoiceList.end(); ++itListChoice ) {
//            TIME_MICRO_SEC t = time[ distance( idxTimeToChoiceList.begin(), itListChoice ) ];
//            ChoiceList::iterator itChoice = itListChoice->begin();
//            advance( itChoice, ( mr.getNumber() % itListChoice->size() ) );
//            map<int,int>::iterator itGrpToPointIdx = itChoice->begin();
//            for( ; itGrpToPointIdx != itChoice->end(); ++itGrpToPointIdx ) {
//                int idxGrp = itGrpToPointIdx->first;
//                itOptVar->grpToStatus[ idxGrp ][ t ] = itGrpToPointIdx->second;
//            }
//        }
//
//        // エネルギー計算
//        itOptVar->energy.resize( section.trjSet[ idxSet ].size() * time.size(), 0.0 );
//        vector<PointIndex> data( itOptVar->energy.size() );
//        vector<int> modifyGroup( itOptVar->energy.size() );
//        for( TrajectorySet::iterator itIdxGrp = section.trjSet[ idxSet ].begin(); itIdxGrp != section.trjSet[ idxSet ].end(); ++itIdxGrp ) {
//            size_t offset_idxGrp = distance( section.trjSet[ idxSet ].begin(), itIdxGrp ) * time.size();
//            for( vector<TIME_MICRO_SEC>::iterator itTime = time.begin(); itTime != time.end(); ++itTime ) {
//                size_t idxTime = distance( time.begin(), itTime );
//                size_t idx = idxTime + offset_idxGrp;
//                data[ idx ].first = *itTime;
//                modifyGroup[ idx ] = *itIdxGrp;
//                if( itOptVar->grpToStatus[ *itIdxGrp ].find( *itTime ) != itOptVar->grpToStatus[ *itIdxGrp ].end() ) {
//                    data[ idx ].second = itOptVar->grpToStatus[ *itIdxGrp ][ *itTime ];
//                } else {
//                    data[ idx ].second = -2;
//                }
//            }
//        }
//        vector<int> idxStart( 1, (int)distance( optvar.begin(), itOptVar ) );
//        energy2_init( &param_calc_energy
//                    , data
//                    , modifyGroup
//                    , 1
//                    , idxStart
//                    , optvar
//                    , tblConnectable
//                    , time );
//
//        int lenVec = (int)param_calc_energy.idxCur.size();
//
//        energy2( (Ipp32f*)&(itOptVar->energy[ 0 ])
//               , param_calc_energy.idxNext1
//               , param_calc_energy.idxCur
//               , param_calc_energy.idxPrev1
//               , lenVec
//               , modifyGroup
//               , infoTrj
//               , param
//               , &buffer );
//    }
////    // cout << "完了" << endl;
//
//    //
//    // エネルギー最小化処理
//    vector<Ipp32f> evec, evec1, evec2, evec3;
//    vector<int> listIdxStart( nStart );
//    for( int i = 0; i < (int)listIdxStart.size(); ++i ) {
//        listIdxStart[ i ] = i;
//    }
//    const int N_LOOP = 1000;
//    for( int loop = 0; loop < N_LOOP; ++loop ) {
//        // 時刻をランダムに選択
//        int idxTime = (int)( mr.getNumber() % time.size() );
//        TIME_MICRO_SEC t = time[ idxTime ];
//
//        //// cout << "energy2_init...";
//        energy2_init( &param_calc_energy
//                    , idxTimeToPointChoiceVector[ idxTime ].data
//                    , idxTimeToPointChoiceVector[ idxTime ].modifyGroup
//                    , idxTimeToPointChoiceVector[ idxTime ].nChoice
//                    , listIdxStart
//                    , optvar
//                    , tblConnectable
//                    , time );
//        //// cout << "done" << endl;
//
//        int lenVec = (int)param_calc_energy.idxCur.size();
//        evec.resize( lenVec );
//        evec1.resize( lenVec );
//        evec2.resize( lenVec );
//        evec3.resize( lenVec );
//
//        //// cout << "energy...";
//
//        energy2( (Ipp32f*)&evec2[ 0 ]
//               , param_calc_energy.idxNext1
//               , param_calc_energy.idxCur
//               , param_calc_energy.idxPrev1
//               , lenVec
//               , idxTimeToPointChoiceVector[ idxTime ].modifyGroup
//               , infoTrj
//               , param
//               , &buffer );
//        energy2( (Ipp32f*)&evec1[ 0 ]
//               , param_calc_energy.idxNext2
//               , param_calc_energy.idxNext1
//               , param_calc_energy.idxCur
//               , lenVec
//               , idxTimeToPointChoiceVector[ idxTime ].modifyGroup
//               , infoTrj
//               , param
//               , &buffer );
//        energy2( (Ipp32f*)&evec3[ 0 ]
//               , param_calc_energy.idxCur
//               , param_calc_energy.idxPrev1
//               , param_calc_energy.idxPrev2
//               , lenVec
//               , idxTimeToPointChoiceVector[ idxTime ].modifyGroup
//               , infoTrj
//               , param
//               , &buffer );
//
//        //// cout << "done" << endl;
//
//        ippsAdd_32f( (Ipp32f*)&evec3[ 0 ]
//                   , (Ipp32f*)&evec2[ 0 ]
//                   , (Ipp32f*)&evec[ 0 ]
//                   , lenVec );
//        ippsAdd_32f_I( (Ipp32f*)&evec1[ 0 ]
//                     , (Ipp32f*)&evec[ 0 ]
//                     , lenVec );
//
////        // cout << "　山登り法開始...";
//        // 各パターンごとに山登り法を行う
//        int idxStart = 0;
//        int nChoice = idxTimeToPointChoiceVector[ idxTime ].nChoice;
//        vector<int>& modifyGroup = idxTimeToPointChoiceVector[ idxTime ].modifyGroup;
//        for( vector<OptVar>::iterator itOptVar = optvar.begin(); itOptVar != optvar.end(); ++itOptVar, ++idxStart ) {
//            // 選択肢ごとのエネルギーを計算
//            //// cout << "　　選択肢ごとのエネルギーを計算(nChoice=" << nChoice << ")...";
//            vector<Ipp32f> choiceToEnergy( nChoice );
//            for( int idxChoice = 0; idxChoice < nChoice; ++idxChoice ) {
//                int idx = idxStart * nChoice * (int)modifyGroup.size() + idxChoice * (int)modifyGroup.size();
//                ippsSum_32f( (Ipp32f*)&evec[ idx ]
//                           , (int)modifyGroup.size()
//                           , (Ipp32f*)&choiceToEnergy[ idxChoice ]
//                           , ippAlgHintAccurate /*ippAlgHintFast*/ );
//            }
//            //// cout << "完了" << endl;
//
//            // エネルギーの最小値を計算
//            //// cout << "　　エネルギーの最小値を計算...(";
////            for( int i = 0; i < (int)choiceToEnergy.size(); ++i ) {
////                // cout << choiceToEnergy[ i ] << ", ";
////            }
//            //// cout << ")...";
//            Ipp32f min;
//            ippsMin_32f( (Ipp32f*)&choiceToEnergy[ 0 ]
//                       , (int)choiceToEnergy.size()
//                       , &min );
//            //// cout << min << "...完了" << endl;
//
//            int idxOptChoice;
//#if 1
//            if( ( mr.getNumber() % 100 ) < ( ( loop > N_LOOP / 2 ) ? 1 : 10 ) ) {
//#else
//            bool flgRand;
//            if( loop > ( 3 * N_LOOP / 5 ) ) {
//                flgRand = false;
//            } else if( loop > N_LOOP / 2 ) {
//                flgRand = ( mr.getNumber() % 100 ) < 1;
//            } else {
//                flgRand = ( mr.getNumber() % 100 ) < 10;
//            }
//            if( flgRand ) {
//#endif
//                // 選択肢のインデックスをランダムに選ぶ
//                //// cout << "　　選択肢のインデックスをランダムに選ぶ...";
//                idxOptChoice = mr.getNumber() % nChoice;
//                //// cout << endl;
//            } else {
//                // 最小のエネルギーを与える選択肢のインデックスをランダムに選ぶ
//                //// cout << "　　最小のエネルギーを与える選択肢のインデックスをランダムに選ぶ...";
//                vector<int> idxMin;
//                for( int i = 0; i < (int)choiceToEnergy.size(); ++i ) {
//                    if( choiceToEnergy[ i ] == min ) {
//                        idxMin.push_back( i );
//                    }
//                }
//                idxOptChoice = idxMin[ mr.getNumber() % idxMin.size() ];
//                //// cout << endl;
//            }
//
//            // 現在の状態を最適な選択肢で置き換える
//            //// cout << "　　現在の状態を最適な選択肢で置き換える...";
//            ChoiceList::iterator itOptChoice = idxTimeToChoiceList[ idxTime ].begin();
//            advance( itOptChoice, idxOptChoice );
//
//            map<int,int>::iterator itGrpToPointIdx = itOptChoice->begin();
//            for( ; itGrpToPointIdx != itOptChoice->end(); ++itGrpToPointIdx ) {
//                int idxGrp = itGrpToPointIdx->first;
//                itOptVar->grpToStatus[ idxGrp ][ t ] = itGrpToPointIdx->second;
//            }
//            //// cout << "完了" << endl;
//
//            // エネルギーを更新する
//            //// cout << "　　エネルギーを更新する...";
//            int idxBegin, idxEnd;
//            idxBegin = idxStart * nChoice * (int)modifyGroup.size() + idxOptChoice * (int)modifyGroup.size();
//            idxEnd = idxBegin + (int)modifyGroup.size();
//            for( int idx = idxBegin; idx < idxEnd; ++idx ) {
//                int idxEnergy, offset_t;
//                if( param_calc_energy.idxNext1[ idx ].second >= -1 ) {
//                    offset_t = (int)distance( time.begin(), find( time.begin(), time.end(), param_calc_energy.idxNext1[ idx ].first ) );
//                    idxEnergy = ( idx - idxBegin ) * (int)time.size() + offset_t;
//                    itOptVar->energy[ idxEnergy ] = evec1[ idx ];
//                }
//                if( param_calc_energy.idxCur[ idx ].second >= -1 ) {
//                    offset_t = idxTime;
//                    idxEnergy = ( idx - idxBegin ) * (int)time.size() + offset_t;
//                    itOptVar->energy[ idxEnergy ] = evec2[ idx ];
//                }
//                if( param_calc_energy.idxPrev1[ idx ].second >= -1 ) {
//                    offset_t = (int)distance( time.begin(), find( time.begin(), time.end(), param_calc_energy.idxPrev1[ idx ].first ) );
//                    idxEnergy = ( idx - idxBegin ) * (int)time.size() + offset_t;
//                    itOptVar->energy[ idxEnergy ] = evec3[ idx ];
//                }
//            }
//            //// cout << "完了" << endl;
//        }
////        // cout << "完了" << endl;
//    }
//
//
////    // cout << "　エネルギー最小状態探索...";
//
//    // エネルギー最小な状態を求める
//    map<int,TimeToPointIndex> optGrpToStatus; // 最適な各グループの点番号の選択状況
//    vector<Ipp32f> e( optvar.size() );
//    for( int i = 0; i < (int)optvar.size(); ++i ) {
//        ippsSum_32f( (Ipp32f*)&(optvar[ i ].energy[ 0 ])
//                   , (int)optvar[ i ].energy.size()
//                   , (Ipp32f*)&e[ i ]
//                   , ippAlgHintAccurate );
//    }
//
//    int idxMin;
//    Ipp32f min_energy;
//    ippsMinIndx_32f( (Ipp32f*)&e[ 0 ]
//                   , (int)e.size()
//                   , &min_energy
//                   , &idxMin );
//    optGrpToStatus = optvar[ idxMin ].grpToStatus;
//
////    // cout << endl;
//
//    //
//    // 最適な選択結果から軌跡を生成する
//    // cout << "　　軌跡生成" << endl;
//    pDst->clear();
//    map<int,TimeToPointIndex>::iterator itStatus = optGrpToStatus.begin();
//    for( ; itStatus != optGrpToStatus.end(); ++itStatus ) {
//        // cout << "　　　グループ" << itStatus->first;
//        int idxGrp = itStatus->first;
//        TrajectoryElement trj;
//        bool flgMaking = false;
//        int idxPrevPoint = -1;
//        int id = -1;
//        TimeToPointIndex::iterator itIdxPoint = itStatus->second.begin();
//        for( ; itIdxPoint != itStatus->second.end(); ++itIdxPoint ) {
//            int idxPoint = itIdxPoint->second;
//            map<int,int>::iterator itID;
//            if( ( itID = idxPointToID.find( idxPoint ) ) != idxPointToID.end() ) {
//                id = itID->second;
//            }
//            if( idxPoint != -1 ) {
//                PosXYTID pos = infoTrj.points[ idxPoint ];
//                if( idxPrevPoint != -1 ) {
//                    if( find( infoTrj.connectable[ idxPrevPoint ].begin()
//                            , infoTrj.connectable[ idxPrevPoint ].end()
//                            , idxPoint ) != infoTrj.connectable[ idxPrevPoint ].end() ) {
//                        trj.insert( PosXYT( pos.x, pos.y, pos.t ) );
//                    } else {
//                        if( trj.size() >= 3 ) {
//                            pDst->push_back( trj );
//                            pDstID->push_back( id );
//                        }
//                        id = -1;
//                        trj.clear();
//                        trj.insert( PosXYT( pos.x, pos.y, pos.t ) );
//                    }
//                } else {
//                        trj.insert( PosXYT( pos.x, pos.y, pos.t ) );
//                }
//                flgMaking = true;
//            } else {
//                if( flgMaking ) {
//                    if( trj.size() >= 3 ) {
//                        pDst->push_back( trj );
//                        pDstID->push_back( id );
//                    }
//                    id = -1;
//                    trj.clear();
//                    flgMaking = false;
//                }
//            }
//            idxPrevPoint = idxPoint;
//        }
//        if( trj.size() >= 3 ) {
//            pDst->push_back( trj );
//            pDstID->push_back( id );
//        }
//        id = -1;
//        // cout << ", 現時点での軌跡数: " << pDst->size() << endl;
//    }
//
//    // cout << "　　エネルギー: " << min_energy + energyExPoints
//    //     << ", min_energy=" << min_energy
//    //     << ", energyExPoints=" << energyExPoints << endl;
//
//    return min_energy + energyExPoints;
//}

double Optimize( vector<TrajectoryElement>* pDst, std::vector<int>* pDstID, std::map<int,int>* pReserve, int idxSection, int idxSet, int nStart, TrajectoriesInfo* pInfoTrj, std::map<int,int>* pPointIdxToTrjNo, PARAM_RENOVATE_TRAJECTORY param, double* p_t1, double* p_t2, double* p_t3 )
{
    TrajectoriesInfo& infoTrj = *pInfoTrj;
    Section& section = infoTrj.section[ idxSection ];
    //MersenneRandomizer mr( 0, RAND_MAX );
    //PerformanceCounter cpc;

    vector<TIME_MICRO_SEC> time;

//    cerr << "　　セット: " << idxSet << ", 構成 ";
//    for( vector<int>::iterator itIdxGrp = section.trjSet[ idxSet ].begin(); itIdxGrp != section.trjSet[ idxSet ].end(); ++itIdxGrp ) {
//        cerr << *itIdxGrp << " ";
//    }
//    cerr << endl;

    //
    // idxSet番目のセットに含まれない軌跡の中に，pReserveで指定した軌跡が１つでもあれば解なし。

    // idxSet番目のセットに含まれるグループ中の軌跡(idxTrjInThisSet)と予約軌跡idxRsvTrjを列挙。
    vector<int> idxRsvTrj, idxTrjInThisSet;
    for( map<int,int>::iterator itRsvTrj = pReserve->begin(); itRsvTrj != pReserve->end(); ++itRsvTrj ) {
        idxRsvTrj.push_back( itRsvTrj->first );
    }
    for( vector<int>::iterator itIdxGrp = section.trjSet[ idxSet ].begin(); itIdxGrp != section.trjSet[ idxSet ].end(); ++itIdxGrp ) {
        TrajectoryGroup::iterator itIdxTrj = section.trjGroup[ *itIdxGrp ].begin();
        for( ; itIdxTrj != section.trjGroup[ *itIdxGrp ].end(); ++itIdxTrj ) {
            idxTrjInThisSet.push_back( *itIdxTrj );
        }
    }
    sort( idxTrjInThisSet.begin(), idxTrjInThisSet.end() );

    // idxSection番目のセクションの軌跡のうち，セットに含まれない軌跡(idxTrjOutOfThisSet)を列挙
    vector<int> idxTrjOutOfThisSet;
    set_difference( section.idxTrajectory.begin()
                  , section.idxTrajectory.end()
                  , idxTrjInThisSet.begin()
                  , idxTrjInThisSet.end()
                  , inserter( idxTrjOutOfThisSet, idxTrjOutOfThisSet.begin() ) );
    // cout << "　　セットに含まれない軌跡: ";
    for( vector<int>::iterator it = idxTrjOutOfThisSet.begin(); it != idxTrjOutOfThisSet.end(); ++it ) {
        // cout << *it << " ";
    }
    // cout << endl;

    // idxTrjOutOfThisSetの中にidxRsvTrjが１つでもあれば解なし。
    vector<int> idxRsvTrjOutOfThisSet;
    set_intersection( idxTrjOutOfThisSet.begin()
                    , idxTrjOutOfThisSet.end()
                    , idxRsvTrj.begin()
                    , idxRsvTrj.end()
                    , inserter( idxRsvTrjOutOfThisSet, idxRsvTrjOutOfThisSet.begin() ) );
    if( !idxRsvTrjOutOfThisSet.empty() ) {
        // cout << "　　解なし(セクション: " << idxSection << ", セット: " << idxSet << ")" << endl;
        return -1.0;
    }

    // 予約軌跡の点番号からIDへのマップを作成
    // cout << "　　idxPointToID: ";
    map<int,int> idxPointToID;
    for( map<int,int>::iterator itRsvTrj = pReserve->begin(); itRsvTrj != pReserve->end(); ++itRsvTrj ) {
        int idxTrj = itRsvTrj->first;
        TrajectoryWithPointIndex::iterator it = infoTrj.trjPointIdx[ idxTrj ].begin();
        for( ; it !=  infoTrj.trjPointIdx[ idxTrj ].end(); ++it ) {
            int idxPoint = it->second;
            int id = itRsvTrj->second;
            idxPointToID[ idxPoint ] = id;
            // cout << "( " << idxPoint << ", " << id << " ), ";
        }
    }
    // cout << endl;

    //
    // 各グループにおける予約軌跡の点番号を求める。
    // cout << "　　予約軌跡: ";
    for( vector<int>::iterator it = idxRsvTrj.begin(); it != idxRsvTrj.end(); ++it ) {
        // cout << *it << " ";
    }
    // cout << endl;
    //map<int,int> grpToID;
    map<int,TimeToPointIndex> grpToIdxRsvPoint;
    for( vector<int>::iterator itIdxGrp = section.trjSet[ idxSet ].begin(); itIdxGrp != section.trjSet[ idxSet ].end(); ++itIdxGrp ) {
        // cout << "　　グループ" << *itIdxGrp << "の軌跡: ";
        for( vector<int>::iterator it = section.trjGroup[ *itIdxGrp ].begin(); it != section.trjGroup[ *itIdxGrp ].end(); ++it ) {
            // cout << *it << " ";
        }
        // cout << endl;
        vector<int> idxRsvTrjOfThisGrp; // itIdxGrp番目のグループに含まれている予約軌跡
        set_intersection( section.trjGroup[ *itIdxGrp ].begin()
                        , section.trjGroup[ *itIdxGrp ].end()
                        , idxRsvTrj.begin()
                        , idxRsvTrj.end()
                        , inserter( idxRsvTrjOfThisGrp, idxRsvTrjOfThisGrp.begin() ) );
        // 予約軌跡は同一グループに入らないので，idxRsvTrjOfThisGrpの要素数は0または1
        if( idxRsvTrjOfThisGrp.empty() ) {
            //grpToID[ *itIdxGrp ] = -1;
        } else {
            //grpToID[ *itIdxGrp ] = (*pReserve)[ idxRsvTrjOfThisGrp.front() ];
            grpToIdxRsvPoint[ *itIdxGrp ].insert( infoTrj.trjPointIdx[ idxRsvTrjOfThisGrp.front() ].begin()
                                                , infoTrj.trjPointIdx[ idxRsvTrjOfThisGrp.front() ].end() );
            //// cout << "　　ID割り当て: グループ" << (int)(*itIdxGrp) << " -> " << (int)( (*pReserve)[ idxRsvTrjOfThisGrp.front() ] ) << endl;
        }
    }


    //
    // idxSet番目のセットに含まれない軌跡(idxTrjOutOfThisSet)の点の分のエネルギーを計算
    double energyExPoints;
    size_t nExPoints = 0;
    for( vector<int>::iterator itIdxTrj = idxTrjOutOfThisSet.begin(); itIdxTrj != idxTrjOutOfThisSet.end(); ++itIdxTrj ) {
        nExPoints += infoTrj.trjElement[ *itIdxTrj ].size();
    }
    energyExPoints = (double)nExPoints * param.lambda3;


    //
    // idxSet番目のセット内のグループの，各時刻での選択肢（点番号）を列挙
//    // cout << "　点番号列挙...";
    map<int,TimeToPointChoice> choiceOfGroup;
    TrajectorySet::iterator itIdxGrp = section.trjSet[ idxSet ].begin();
    for( ; itIdxGrp != section.trjSet[ idxSet ].end(); ++itIdxGrp ) {
        TrajectoryWithPointIndex::iterator itIdxPoint = section.groupToPoints[ *itIdxGrp ].begin();
        for( ; itIdxPoint != section.groupToPoints[ *itIdxGrp ].end(); ++itIdxPoint ) {
            choiceOfGroup[ *itIdxGrp ].insert( *itIdxPoint );
            time.push_back( itIdxPoint->first );
        }
    }
//    // cout << "完了" << endl;

    sort( time.begin(), time.end() );
    time.erase( unique( time.begin(), time.end() ), time.end() );

//    cerr << "　　点の存在する時刻：[" << time.front() << ", " << time.back() << "]" << endl;

    //
    // choiceOfGroupを用いて，全ての時刻における各グループの点番号の組み合わせを作成する。
    // 組み合わせの表現は，選択肢リストと選択肢ベクトルの２通りありともに同一の内容を表す。
    //map<TIME_MICRO_SEC,ChoiceList> timeToChoiceList;
    vector<ChoiceList> idxTimeToChoiceList( time.size() ); // 各時刻に対する選択肢リスト
    vector<ChoiceVector> idxTimeToPointChoiceVector( time.size() ); // 各時刻に対する点番号の選択肢ベクトル
    //vector<TIME_MICRO_SEC>::iterator itTime = time.begin();
    //for( int i = 0; itTime != time.end(); ++itTime, ++i ) {
    for( int i = 0; i < (int)time.size(); ++i ) {
        map<int,int> grpToPointIdx;
        HashTable tblHash;
        makechoicelist( choiceOfGroup.begin()
                      , choiceOfGroup.end()
                      , time[ i ] //*itTime
                      , &idxTimeToChoiceList[ i ]
                      , grpToPointIdx
                      , tblHash
                      , tblHash
                      , tblHash
                      , tblHash
                      , *pInfoTrj
                      , grpToIdxRsvPoint
                      , param.territoryVeryNear/*param.territory*/ );

        if( idxTimeToChoiceList[ i ].empty() ) {
            cerr << "　　選択肢リスト異常！！　";
        }

        idxTimeToPointChoiceVector[ i ].nChoice = (int)idxTimeToChoiceList[ i ].size();
        //if( ( idxTimeToPointChoiceVector[ i ].nChoice = (int)idxTimeToChoiceList[ i ].size() ) != 0 ) {
            map<int,TimeToPointChoice>::iterator itTimeToPoint = choiceOfGroup.begin();
            int offset = 0;
            for( ; itTimeToPoint != choiceOfGroup.end(); ++itTimeToPoint ) {
                int idxGrp = itTimeToPoint->first;
                TimeToPointChoice::iterator it = itTimeToPoint->second.lower_bound( time[ i ] );
                TimeToPointChoice::iterator itEnd = itTimeToPoint->second.upper_bound( time[ i ] );
                if( it != itEnd ) {
                    idxTimeToPointChoiceVector[ i ].modifyGroup.push_back( idxGrp );
                    idxTimeToPointChoiceVector[ i ].data.push_back( PointIndex( time[ i ], -1 ) );
                    idxTimeToPointChoiceVector[ i ].idxGrpIdxPointToOffset[ idxGrp ][ -1 ] = offset;
                    ++offset;
                }
                for( ; it != itEnd; ++it ) {
                    idxTimeToPointChoiceVector[ i ].modifyGroup.push_back( idxGrp );
                    idxTimeToPointChoiceVector[ i ].data.push_back( PointIndex( time[ i ], it->second ) );
                    idxTimeToPointChoiceVector[ i ].idxGrpIdxPointToOffset[ idxGrp ][ it->second ] = offset;
                    ++offset;
                }
            }
        //}
    }

    //
    // connectableテーブル作成
    CStatusTable tblConnectable;
    tblConnectable.SetSize( (int)infoTrj.points.size() );
    for( int i = 0; i < (int)infoTrj.points.size(); ++i ) {
        for( list<int>::iterator it = infoTrj.connectable[ i ].begin(); it != infoTrj.connectable[ i ].end(); ++it ) {
            tblConnectable[ i ].set( *it, true );
        }
    }

//    cerr << "　　初期状態作成...";

    // nStart個の初期状態を生成する
    PARAM_CALC_ENERGY param_calc_energy;
    BUFFER_CALC_ENERGY buffer;
    vector<OptVar> optvar( nStart );
//    cerr << "nStart=" << nStart << "...";
    for( vector<OptVar>::iterator itOptVar = optvar.begin(); itOptVar != optvar.end(); ++itOptVar ) {
        //
        // 各時刻の点番号の組み合わせをランダムに設定する
        //map<int,TimeToPointIndex> grpToStatus; // 各グループの点番号の選択状況
        vector<ChoiceList>::iterator itListChoice = idxTimeToChoiceList.begin();
        for( ; itListChoice != idxTimeToChoiceList.end(); ++itListChoice ) {
            TIME_MICRO_SEC t = time[ distance( idxTimeToChoiceList.begin(), itListChoice ) ];
//            cerr << "t=" << t << "...";
            ChoiceList::iterator itChoice = itListChoice->begin();
//            cerr << "ランダムに選択（選択肢：" << itListChoice->size() << "[個]）...";
            advance( itChoice, ( rand()/*mr.getNumber()*/ % itListChoice->size() ) );
//            cerr << "完了...";
            map<int,int>::iterator itGrpToPointIdx = itChoice->begin();
            for( ; itGrpToPointIdx != itChoice->end(); ++itGrpToPointIdx ) {
                int idxGrp = itGrpToPointIdx->first;
                itOptVar->grpToStatus[ idxGrp ][ t ] = itGrpToPointIdx->second;
            }
        }

        // エネルギー計算
//        cerr << "エネルギー計算...";
        itOptVar->energy.resize( section.trjSet[ idxSet ].size() * time.size(), 0.0 );
        vector<PointIndex> data( itOptVar->energy.size() );
        vector<int> modifyGroup( itOptVar->energy.size() );
        for( TrajectorySet::iterator itIdxGrp = section.trjSet[ idxSet ].begin(); itIdxGrp != section.trjSet[ idxSet ].end(); ++itIdxGrp ) {
            size_t offset_idxGrp = distance( section.trjSet[ idxSet ].begin(), itIdxGrp ) * time.size();
            for( vector<TIME_MICRO_SEC>::iterator itTime = time.begin(); itTime != time.end(); ++itTime ) {
                size_t idxTime = distance( time.begin(), itTime );
                size_t idx = idxTime + offset_idxGrp;
                data[ idx ].first = *itTime;
                modifyGroup[ idx ] = *itIdxGrp;
                if( itOptVar->grpToStatus[ *itIdxGrp ].find( *itTime ) != itOptVar->grpToStatus[ *itIdxGrp ].end() ) {
                    data[ idx ].second = itOptVar->grpToStatus[ *itIdxGrp ][ *itTime ];
                } else {
                    data[ idx ].second = -2;
                }
            }
        }
        vector<int> idxStart( 1, (int)distance( optvar.begin(), itOptVar ) );
        energy2_init( &param_calc_energy
                    , data
                    , modifyGroup
                    , 1
                    , idxStart
                    , optvar
                    , tblConnectable
                    , time );

        int lenVec = (int)param_calc_energy.idxCur.size();

        energy2( (Ipp32f*)&(itOptVar->energy[ 0 ])
               , param_calc_energy.idxNext1
               , param_calc_energy.idxCur
               , param_calc_energy.idxPrev1
               , lenVec
               , modifyGroup
               , infoTrj
               , param
               , &buffer );

//        cerr << "完了...";
    }
//    cerr << "完了" << endl;

//    cerr << "　　エネルギー最小化...";

    //
    // エネルギー最小化処理
    vector<Ipp32f> evec, evec1, evec2, evec3;
    vector<int> listIdxStart( nStart );
    for( int i = 0; i < (int)listIdxStart.size(); ++i ) {
        listIdxStart[ i ] = i;
    }
    const int N_LOOP = 3000;
    for( int loop = 0; loop < N_LOOP; ++loop ) {
        // 時刻をランダムに選択
        int idxTime = (int)( rand()/*mr.getNumber()*/ % time.size() );
        TIME_MICRO_SEC t = time[ idxTime ];

        //// cout << "energy2_init...";
        energy2_init( &param_calc_energy
                    , idxTimeToPointChoiceVector[ idxTime ].data
                    , idxTimeToPointChoiceVector[ idxTime ].modifyGroup
                    , ( idxTimeToPointChoiceVector[ idxTime ].nChoice > 0 ) ? 1 : 0
                    , listIdxStart
                    , optvar
                    , tblConnectable
                    , time );
        //// cout << "done" << endl;

        int lenVec = (int)param_calc_energy.idxCur.size();
        evec.resize( lenVec );
        evec1.resize( lenVec );
        evec2.resize( lenVec );
        evec3.resize( lenVec );

        //// cout << "energy...";

        energy2( (Ipp32f*)&evec2[ 0 ]
               , param_calc_energy.idxNext1
               , param_calc_energy.idxCur
               , param_calc_energy.idxPrev1
               , lenVec
               , idxTimeToPointChoiceVector[ idxTime ].modifyGroup
               , infoTrj
               , param
               , &buffer );
        energy2( (Ipp32f*)&evec1[ 0 ]
               , param_calc_energy.idxNext2
               , param_calc_energy.idxNext1
               , param_calc_energy.idxCur
               , lenVec
               , idxTimeToPointChoiceVector[ idxTime ].modifyGroup
               , infoTrj
               , param
               , &buffer );
        energy2( (Ipp32f*)&evec3[ 0 ]
               , param_calc_energy.idxCur
               , param_calc_energy.idxPrev1
               , param_calc_energy.idxPrev2
               , lenVec
               , idxTimeToPointChoiceVector[ idxTime ].modifyGroup
               , infoTrj
               , param
               , &buffer );

        //// cout << "done" << endl;

        ippsAdd_32f( (Ipp32f*)&evec3[ 0 ]
                   , (Ipp32f*)&evec2[ 0 ]
                   , (Ipp32f*)&evec[ 0 ]
                   , lenVec );
        ippsAdd_32f_I( (Ipp32f*)&evec1[ 0 ]
                     , (Ipp32f*)&evec[ 0 ]
                     , lenVec );

//        // cout << "　山登り法開始...";
        // 各パターンごとに山登り法を行う
        int idxStart = 0;
        int nChoice = (int)idxTimeToChoiceList[ idxTime ].size(); //idxTimeToPointChoiceVector[ idxTime ].nChoice;
        vector<int>& modifyGroup = idxTimeToPointChoiceVector[ idxTime ].modifyGroup;
        for( vector<OptVar>::iterator itOptVar = optvar.begin(); itOptVar != optvar.end(); ++itOptVar, ++idxStart ) {
            // 選択肢ごとのエネルギーを計算
            vector<Ipp32f> choiceToEnergy( nChoice );
            ChoiceList::iterator itChoice = idxTimeToChoiceList[ idxTime ].begin();
            for( int idxChoice = 0 ; itChoice != idxTimeToChoiceList[ idxTime ].end(); ++itChoice, ++idxChoice ) {
                choiceToEnergy[ idxChoice ] = 0;
                for( map<int,int>::iterator it = itChoice->begin(); it != itChoice->end(); ++it ) {
                    int idx = idxStart * (int)modifyGroup.size() + idxTimeToPointChoiceVector[ idxTime ].idxGrpIdxPointToOffset[ it->first ][ it->second ];
                    choiceToEnergy[ idxChoice ] += evec[ idx ];
                }
            }

            //// cout << "完了" << endl;

            // エネルギーの最小値を計算
            //// cout << "　　エネルギーの最小値を計算...(";
//            for( int i = 0; i < (int)choiceToEnergy.size(); ++i ) {
//                // cout << choiceToEnergy[ i ] << ", ";
//            }
            //// cout << ")...";
            Ipp32f min;
            ippsMin_32f( (Ipp32f*)&choiceToEnergy[ 0 ]
                       , (int)choiceToEnergy.size()
                       , &min );
            //// cout << min << "...完了" << endl;

            int idxOptChoice;
#if 1
            if( ( rand()/*mr.getNumber()*/ % 100 ) < ( ( loop > N_LOOP / 2 ) ? 0 : 10 ) ) {
#else
            bool flgRand;
            if( loop > ( 3 * N_LOOP / 5 ) ) {
                flgRand = false;
            } else if( loop > N_LOOP / 2 ) {
                flgRand = ( mr.getNumber() % 100 ) < 1;
            } else {
                flgRand = ( mr.getNumber() % 100 ) < 10;
            }
            if( flgRand ) {
#endif
                // 選択肢のインデックスをランダムに選ぶ
                //// cout << "　　選択肢のインデックスをランダムに選ぶ...";
                idxOptChoice = rand()/*mr.getNumber()*/ % nChoice;
                //// cout << endl;
            } else {
                // 最小のエネルギーを与える選択肢のインデックスをランダムに選ぶ
                //// cout << "　　最小のエネルギーを与える選択肢のインデックスをランダムに選ぶ...";
                vector<int> idxMin;
                for( int i = 0; i < (int)choiceToEnergy.size(); ++i ) {
                    if( choiceToEnergy[ i ] == min ) {
                        idxMin.push_back( i );
                    }
                }
                idxOptChoice = idxMin[ rand()/*mr.getNumber()*/ % idxMin.size() ];
                //// cout << endl;
            }

            // 現在の状態を最適な選択肢で置き換える
            //// cout << "　　現在の状態を最適な選択肢で置き換える...";
            ChoiceList::iterator itOptChoice = idxTimeToChoiceList[ idxTime ].begin();
            advance( itOptChoice, idxOptChoice );

            map<int,int>::iterator itGrpToPointIdx = itOptChoice->begin();
            for( ; itGrpToPointIdx != itOptChoice->end(); ++itGrpToPointIdx ) {
                int idxGrp = itGrpToPointIdx->first;
                itOptVar->grpToStatus[ idxGrp ][ t ] = itGrpToPointIdx->second;
            }
            //// cout << "完了" << endl;

            // エネルギーを更新する
            //// cout << "　　エネルギーを更新する...";
            int offset_idx = 0;
            for( itGrpToPointIdx = itOptChoice->begin(); itGrpToPointIdx != itOptChoice->end(); ++itGrpToPointIdx, ++offset_idx ) {
                int idx = idxStart * (int)modifyGroup.size() + idxTimeToPointChoiceVector[ idxTime ].idxGrpIdxPointToOffset[ itGrpToPointIdx->first ][ itGrpToPointIdx->second ];
                int idxEnergy, offset_t;
                if( param_calc_energy.idxNext1[ idx ].second >= -1 ) {
                    offset_t = (int)distance( time.begin(), find( time.begin(), time.end(), param_calc_energy.idxNext1[ idx ].first ) );
                    idxEnergy = offset_idx * (int)time.size() + offset_t;
                    itOptVar->energy[ idxEnergy ] = evec1[ idx ];
                }
                if( param_calc_energy.idxCur[ idx ].second >= -1 ) {
                    offset_t = idxTime;
                    idxEnergy = offset_idx * (int)time.size() + offset_t;
                    itOptVar->energy[ idxEnergy ] = evec2[ idx ];
                }
                if( param_calc_energy.idxPrev1[ idx ].second >= -1 ) {
                    offset_t = (int)distance( time.begin(), find( time.begin(), time.end(), param_calc_energy.idxPrev1[ idx ].first ) );
                    idxEnergy = offset_idx * (int)time.size() + offset_t;
                    itOptVar->energy[ idxEnergy ] = evec3[ idx ];
                }
            }
            //// cout << "完了" << endl;
        }
//        // cout << "完了" << endl;
    }

//    cerr << "完了" << endl;

//    cerr << "　　エネルギー最小状態探索...";

    // エネルギー最小な状態を求める
    map<int,TimeToPointIndex> optGrpToStatus; // 最適な各グループの点番号の選択状況
    vector<Ipp32f> e( optvar.size() );
    for( int i = 0; i < (int)optvar.size(); ++i ) {
        ippsSum_32f( (Ipp32f*)&(optvar[ i ].energy[ 0 ])
                   , (int)optvar[ i ].energy.size()
                   , (Ipp32f*)&e[ i ]
                   , ippAlgHintAccurate );
    }

    int idxMin;
    Ipp32f min_energy;
    ippsMinIndx_32f( (Ipp32f*)&e[ 0 ]
                   , (int)e.size()
                   , &min_energy
                   , &idxMin );
    optGrpToStatus = optvar[ idxMin ].grpToStatus;

//    cerr << "完了" << endl;

    //
    // 最適な選択結果から軌跡を生成する
//    cerr << "　　軌跡生成...";
    pDst->clear();
    int nEnabledPoints = 0;
    map<int,TimeToPointIndex>::iterator itStatus = optGrpToStatus.begin();
    for( ; itStatus != optGrpToStatus.end(); ++itStatus ) {
        // cout << "　　　グループ" << itStatus->first;
        int idxGrp = itStatus->first;
        TrajectoryElement trj;
        bool flgMaking = false;
        int idxPrevPoint = -1;
        int id = -1;
        TimeToPointIndex::iterator itIdxPoint = itStatus->second.begin();
        for( ; itIdxPoint != itStatus->second.end(); ++itIdxPoint ) {
            int idxPoint = itIdxPoint->second;
            map<int,int>::iterator itID;
            if( ( itID = idxPointToID.find( idxPoint ) ) != idxPointToID.end() ) {
                id = itID->second;
            }
            if( idxPoint != -1 ) {
                PosXYTID pos = infoTrj.points[ idxPoint ];
                if( idxPrevPoint != -1 ) {
                    if( find( infoTrj.connectable[ idxPrevPoint ].begin()
                            , infoTrj.connectable[ idxPrevPoint ].end()
                            , idxPoint ) != infoTrj.connectable[ idxPrevPoint ].end() ) {
                        //trj.insert( PosXYTID( pos.x, pos.y, pos.t, (*pPointIdxToTrjNo)[ idxPoint ] ) );
                        trj.insert( pos );
                        ++nEnabledPoints;
                    } else {
                        if( trj.size() >= 3 ) {
                            pDst->push_back( trj );
                            pDstID->push_back( id );
                        }
                        id = -1;
                        trj.clear();
                        //trj.insert( PosXYTID( pos.x, pos.y, pos.t, (*pPointIdxToTrjNo)[ idxPoint ] ) );
                        trj.insert( pos );
                        ++nEnabledPoints;
                    }
                } else {
                        //trj.insert( PosXYTID( pos.x, pos.y, pos.t, (*pPointIdxToTrjNo)[ idxPoint ] ) );
                        trj.insert( pos );
                        ++nEnabledPoints;
                }
                flgMaking = true;
            } else {
                if( flgMaking ) {
                    if( trj.size() >= 3 ) {
                        pDst->push_back( trj );
                        pDstID->push_back( id );
                    }
                    id = -1;
                    trj.clear();
                    flgMaking = false;
                }
            }
            idxPrevPoint = idxPoint;
        }
        if( trj.size() >= 3 ) {
            pDst->push_back( trj );
            pDstID->push_back( id );
        }
        id = -1;
        // cout << ", 現時点での軌跡数: " << pDst->size() << endl;
    }
//    cerr << "完了" << endl;

    // cout << "　　エネルギー: " << min_energy + energyExPoints
    //     << ", min_energy=" << min_energy
    //     << ", energyExPoints=" << energyExPoints << endl;

    //return min_energy + energyExPoints;
    int nAllPoints = 0;
    for( vector<int>::iterator itIdxTrj = section.idxTrajectory.begin(); itIdxTrj != section.idxTrajectory.end(); ++itIdxTrj ) {
        nAllPoints += infoTrj.trjElement[ *itIdxTrj ].size();
    }
    return min_energy + 30.0 * (double)( nAllPoints - nEnabledPoints );
}

bool next_status( PointIndex* pIdxNextPoint, TimeToPointIndex::iterator* pItNext, TimeToPointIndex::iterator itIdxPoint, TimeToPointIndex& status, CStatusTable& tblConnectable )
{
    TimeToPointIndex::iterator itIdxNextPoint = itIdxPoint;
    //int idxPoint; // 時刻timeでの点番号

    if( itIdxPoint != status.end() ) {
    //if( ( idxPoint = itIdxPoint->second ) != -1 ) {
        advance( itIdxNextPoint, 1 );
        if( itIdxNextPoint != status.end() ) {
            //if( tblConnectable[ idxPoint ].get( itIdxNextPoint->second ) ) {
                if( pIdxNextPoint ) {
                    *pIdxNextPoint = PointIndex( itIdxNextPoint->first, itIdxNextPoint->second );
                }

                if( pItNext ) {
                    *pItNext = itIdxNextPoint;
                }

                return true;
            //}
        }
    //}
    }

    pIdxNextPoint->first = 0;
    pIdxNextPoint->second = -2;
    return false;
}

bool prev_status( PointIndex* pIdxPrevPoint, TimeToPointIndex::iterator* pItPrev, TimeToPointIndex::iterator itIdxPoint, TimeToPointIndex& status, CStatusTable& tblConnectable )
{
    TimeToPointIndex::iterator itIdxPrevPoint = itIdxPoint;
    //int idxPoint; // 時刻timeでの点番号

    if( itIdxPoint != status.end() && itIdxPoint != status.begin() ) {
        //if( ( idxPoint = itIdxPoint->second ) != -1 ) {
            advance( itIdxPrevPoint, -1 );
            //if( tblConnectable[ itIdxPrevPoint->second ].get( idxPoint ) ) {
                if( pIdxPrevPoint ) {
                    *pIdxPrevPoint = PointIndex( itIdxPrevPoint->first, itIdxPrevPoint->second );
                }

                if( pItPrev ) {
                    *pItPrev = itIdxPrevPoint;
                }

                return true;
            //}
        //}
    }

    pIdxPrevPoint->first = 0;
    pIdxPrevPoint->second = -2;
    return false;
}

void energy2_init( PARAM_CALC_ENERGY* pDst, vector<PointIndex>& data, vector<int>& modifyGroup, int nChoice, vector<int>& listIdxStart, vector<OptVar>& optvar, CStatusTable& tblConnectable, vector<TIME_MICRO_SEC>& listTime )
{
    vector<PointIndex>& next2 = pDst->idxNext2;
    vector<PointIndex>& next1 = pDst->idxNext1;
    vector<PointIndex>& cur = pDst->idxCur;
    vector<PointIndex>& prev1 = pDst->idxPrev1;
    vector<PointIndex>& prev2 = pDst->idxPrev2;
    int nModifyGrp = (int)modifyGroup.size();

    //
    // curの中身を作る

    // dataの内容をcurにnStart回コピー
    cur.resize( data.size() * listIdxStart.size() );
    for( int idx = 0; idx < (int)listIdxStart.size(); ++idx ) {
       ippsCopy_8u( (Ipp8u*)&data[ 0 ]
                  , (Ipp8u*)&cur[ idx * (int)data.size() ]
                  , (int)( data.size() * sizeof( PointIndex ) ) );
    }


    //
    // next2, next1, prev1, prev2の中身を作る

    next2.resize( cur.size() );
    next1.resize( cur.size() );
    prev1.resize( cur.size() );
    prev2.resize( cur.size() );

    //for( int idxStart = 0; idxStart < nStart; ++idxStart ) {
        //const size_t idx = idxStart * modifyGroup.size() * nChoice;
    int distanceIdxStart = 0;
    for( vector<int>::iterator itIdxStart  = listIdxStart.begin(); itIdxStart != listIdxStart.end(); ++itIdxStart, ++distanceIdxStart ) {
        const size_t idx = distanceIdxStart * modifyGroup.size() * nChoice;
        int i = 0;
        for( vector<int>::iterator itIdxGrp = modifyGroup.begin(); itIdxGrp != modifyGroup.end(); ++itIdxGrp, ++i ) {
            TimeToPointIndex& status = optvar[ *itIdxStart ].grpToStatus[ *itIdxGrp ];
            TimeToPointIndex::iterator itIdxPoint, itIdxPrevPoint, itIdxNextPoint;

            //if( ( itIdxPoint = status.find( time ) ) != status.end() ) {
                itIdxPoint = status.find( cur[ idx + i ].first );
                // 次の時刻，次の次の時刻の状態をgrpToStatusから取得
                if( next_status( &next1[ idx + i ], &itIdxNextPoint, itIdxPoint, status, tblConnectable ) ) {
                    next_status( &next2[ idx + i ], NULL, itIdxNextPoint, status, tblConnectable );
                } else {
                    next2[ idx + i ].first = 0;
                    next2[ idx + i ].second = -2;
                }

                // 前の時刻，前の前の時刻の状態をgrpToStatusから取得
                if( prev_status( &prev1[ idx + i ], &itIdxPrevPoint, itIdxPoint, status, tblConnectable ) ) {
                    prev_status( &prev2[ idx + i ], NULL, itIdxPrevPoint, status, tblConnectable );
                } else {
                    prev2[ idx + i ].first = 0;
                    prev2[ idx + i ].second = -2;
                }
            //}
        }

        // 同一内容をnChoice回コピー
        for( int idxChoice = 1; idxChoice < nChoice; ++idxChoice ) {
            ippsCopy_8u( (Ipp8u*)&next1[ idx ]
                       , (Ipp8u*)&next1[ idx + idxChoice * nModifyGrp ]
                       , nModifyGrp * sizeof( PointIndex ) );
            ippsCopy_8u( (Ipp8u*)&next2[ idx ]
                       , (Ipp8u*)&next2[ idx + idxChoice * nModifyGrp ]
                       , nModifyGrp * sizeof( PointIndex ) );
            ippsCopy_8u( (Ipp8u*)&prev1[ idx ]
                       , (Ipp8u*)&prev1[ idx + idxChoice * nModifyGrp ]
                       , nModifyGrp * sizeof( PointIndex ) );
            ippsCopy_8u( (Ipp8u*)&prev2[ idx ]
                       , (Ipp8u*)&prev2[ idx + idxChoice * nModifyGrp ]
                       , nModifyGrp * sizeof( PointIndex ) );
        }
    }
}

void energy2( Ipp32f* e, vector<PointIndex>& next, vector<PointIndex>& cur, vector<PointIndex>& prev, int len, vector<int>& modifyGroup, TrajectoriesInfo& infoTrj, PARAM_RENOVATE_TRAJECTORY param, BUFFER_CALC_ENERGY* pBuffer )
{
    CStatusList& tmp_a = pBuffer->tmp_a;
    CStatusList& tmp_b = pBuffer->tmp_b;
    CStatusList& tmp_c = pBuffer->tmp_c;
    CStatusList& tmp_next = pBuffer->tmp_next;
    CStatusList& tmp_cur = pBuffer->tmp_cur;
    CStatusList& tmp_prev = pBuffer->tmp_prev;
    CStatusList& tmp_mask = pBuffer->tmp_mask;
    vector<Ipp32f>& tmp_ea = pBuffer->tmp_ea;
    vector<Ipp32f>& tmp_eb = pBuffer->tmp_eb;
    vector<Ipp32f>& tmp_ec = pBuffer->tmp_ec;
    vector<Ipp32f>& pos_x_next = pBuffer->pos_x_next;
    vector<Ipp32f>& pos_y_next = pBuffer->pos_y_next;
    vector<Ipp32f>& pos_x_cur = pBuffer->pos_x_cur;
    vector<Ipp32f>& pos_y_cur = pBuffer->pos_y_cur;
    vector<Ipp32f>& pos_x_prev = pBuffer->pos_x_prev;
    vector<Ipp32f>& pos_y_prev = pBuffer->pos_y_prev;
    vector<Ipp32f>& dt_next_cur = pBuffer->dt_next_cur;
    vector<Ipp32f>& dt_cur_prev = pBuffer->dt_cur_prev;

    int nModifyGrp = (int)modifyGroup.size();
    IppiSize size;
    size.width = len;
    size.height = 1;

    tmp_a.SetSize( len );
    tmp_b.SetSize( len );
    tmp_c.SetSize( len );
    tmp_next.SetSize( len );
    tmp_cur.SetSize( len );
    tmp_prev.SetSize( len );
    tmp_mask.SetSize( len );

    tmp_ea.resize( len );
    tmp_eb.resize( len );
    tmp_ec.resize( len );

    // curの要素で-1以上のものにtrueが対応し，それ以外にはfalseが対応するマスクを作る。
    for( int i = 0; i < len; ++i ) {
        tmp_mask.set( i, cur[ i ].second >= -1 );
    }

    // next, cur, prevの要素で-1より大きなものにはtrueが対応する配列を作成。
    for( int i = 0; i < len; ++i ) {
        tmp_next.set( i, next[ i ].second > -1 );
        tmp_cur.set( i, cur[ i ].second > -1 );
        tmp_prev.set( i, prev[ i ].second > -1 );
    }

    // 孤立点に対応するビットがtrueとなるtmp_cを作成。
    ippsOr_8u( (Ipp8u*)&tmp_next[ 0 ]
             , (Ipp8u*)&tmp_prev[ 0 ]
             , (Ipp8u*)&tmp_c[ 0 ]
             , (int)tmp_c.size() );
    ippsNot_8u_I( (Ipp8u*)&tmp_c[ 0 ]
               , (int)tmp_c.size() );

    // 未使用点に対応するtmp_cのビットをtrueにする。
    ippsNot_8u( (Ipp8u*)&tmp_cur[ 0 ]
              , (Ipp8u*)&tmp_a[ 0 ]
              , (int)tmp_a.size() );
    ippsOr_8u_I( (Ipp8u*)&tmp_a[ 0 ]
               , (Ipp8u*)&tmp_c[ 0 ]
               , (int)tmp_c.size() );

    // 始点・終点に対応するビットがtrueとなるtmp_bを作成。
    ippsAnd_8u( (Ipp8u*)&tmp_next[ 0 ]
              , (Ipp8u*)&tmp_prev[ 0 ]
              , (Ipp8u*)&tmp_b[ 0 ]
              , (int)tmp_b.size() );
    ippsNot_8u_I( (Ipp8u*)&tmp_b[ 0 ]
                , (int)tmp_b.size() );
    ippsNot_8u( (Ipp8u*)&tmp_c[ 0 ]
              , (Ipp8u*)&tmp_a[ 0 ]
              , (int)tmp_a.size() );
    ippsAnd_8u_I( (Ipp8u*)&tmp_a[ 0 ]
                , (Ipp8u*)&tmp_b[ 0 ]
                , (int)tmp_b.size() );

    // 中継点に対応するビットがtrueとなるtmp_aを作成
    ippsOr_8u( (Ipp8u*)&tmp_b[ 0 ]
             , (Ipp8u*)&tmp_c[ 0 ]
             , (Ipp8u*)&tmp_a[ 0 ]
             , (int)tmp_a.size() );
    ippsNot_8u_I( (Ipp8u*)&tmp_a[ 0 ]
                , (int)tmp_a.size() );

    // curの要素で-1より小さい部分がエネルギーに反映されないよう，マスクする。
    ippsAnd_8u_I( (Ipp8u*)&tmp_mask[ 0 ]
                , (Ipp8u*)&tmp_a[ 0 ]
                , (int)tmp_a.size() );
    ippsAnd_8u_I( (Ipp8u*)&tmp_mask[ 0 ]
                , (Ipp8u*)&tmp_b[ 0 ]
                , (int)tmp_b.size() );
    ippsAnd_8u_I( (Ipp8u*)&tmp_mask[ 0 ]
                , (Ipp8u*)&tmp_c[ 0 ]
                , (int)tmp_c.size() );


    //
    // 未使用点・孤立点のエネルギー計算
    for( int i = 0; i < len; ++i ) {
        tmp_ec[ i ] = tmp_c.get( i ) ? (Ipp32f)param.lambda3 : 0.0f;
    }


    //
    // 始点・終点のエネルギー計算
    for( int i = 0; i < len; ++i ) {
        tmp_eb[ i ] = tmp_b.get( i ) ? (Ipp32f)param.lambda2 / 2.0f : 0.0f;
    }

    //
    // 中継点のエネルギー計算
    int nRelay = 0;
    pos_x_next.clear();
    pos_y_next.clear();
    pos_x_cur.clear();
    pos_y_cur.clear();
    pos_x_prev.clear();
    pos_y_prev.clear();

    for( int i = 0; i < len; ++i ) {
        if( tmp_a.get( i ) ) {
            pos_x_next.push_back( (Ipp32f)infoTrj.points[ next[ i ].second ].x );
            pos_y_next.push_back( (Ipp32f)infoTrj.points[ next[ i ].second ].y );
            pos_x_cur.push_back( (Ipp32f)infoTrj.points[ cur[ i ].second ].x );
            pos_y_cur.push_back( (Ipp32f)infoTrj.points[ cur[ i ].second ].y );
            pos_x_prev.push_back( (Ipp32f)infoTrj.points[ prev[ i ].second ].x );
            pos_y_prev.push_back( (Ipp32f)infoTrj.points[ prev[ i ].second ].y );

            dt_next_cur.push_back( (Ipp32f)( infoTrj.points[ next[ i ].second ].t - infoTrj.points[ cur[ i ].second ].t ) * 1.0e-6f );
            dt_cur_prev.push_back( (Ipp32f)( infoTrj.points[ cur[ i ].second ].t - infoTrj.points[ prev[ i ].second ].t ) * 1.0e-6f );
            ++nRelay;
        }
    }

    if( nRelay > 0 ) {
        // pos_x_next = ( pos_x_cur - pos_x_next )^2
        ippsSub_32f_I( (Ipp32f*)&pos_x_cur[ 0 ]
                     , (Ipp32f*)&pos_x_next[ 0 ]
                     , nRelay );
        ippsSqr_32f_I( (Ipp32f*)&pos_x_next[ 0 ]
                     , nRelay );

        // pos_y_next = ( pos_y_cur - pos_y_next )^2
        ippsSub_32f_I( (Ipp32f*)&pos_y_cur[ 0 ]
                     , (Ipp32f*)&pos_y_next[ 0 ]
                     , nRelay );
        ippsSqr_32f_I( (Ipp32f*)&pos_y_next[ 0 ]
                     , nRelay );

        // pos_y_next = sqrt( pos_x_next + pos_y_next )
        ippsAdd_32f_I( (Ipp32f*)&pos_x_next[ 0 ]
                     , (Ipp32f*)&pos_y_next[ 0 ]
                     , nRelay );
        ippsSqrt_32f_I( (Ipp32f*)&pos_y_next[ 0 ]
                      , nRelay );

        // pos_x_next = pos_y_next / dt_next_cur;
        ippsDiv_32f( (Ipp32f*)&dt_next_cur[ 0 ]
                   , (Ipp32f*)&pos_y_next[ 0 ]
                   , (Ipp32f*)&pos_x_next[ 0 ]
                   , nRelay );


        // pos_x_prev = ( pos_x_cur - pos_x_prev )^2
        ippsSub_32f_I( (Ipp32f*)&pos_x_cur[ 0 ]
                     , (Ipp32f*)&pos_x_prev[ 0 ]
                     , nRelay );
        ippsSqr_32f_I( (Ipp32f*)&pos_x_prev[ 0 ]
                     , nRelay );

        // pos_y_prev = ( pos_y_cur - pos_y_prev )^2
        ippsSub_32f_I( (Ipp32f*)&pos_y_cur[ 0 ]
                     , (Ipp32f*)&pos_y_prev[ 0 ]
                     , nRelay );
        ippsSqr_32f_I( (Ipp32f*)&pos_y_prev[ 0 ]
                     , nRelay );

        // pos_y_prev = sqrt( pos_x_prev + pos_y_prev )
        ippsAdd_32f_I( (Ipp32f*)&pos_x_prev[ 0 ]
                     , (Ipp32f*)&pos_y_prev[ 0 ]
                     , nRelay );
        ippsSqrt_32f_I( (Ipp32f*)&pos_y_prev[ 0 ]
                      , nRelay );

        // pos_x_prev = pos_y_prev / dt_prev_cur;
        ippsDiv_32f( (Ipp32f*)&dt_cur_prev[ 0 ]
                   , (Ipp32f*)&pos_y_prev[ 0 ]
                   , (Ipp32f*)&pos_x_prev[ 0 ]
                   , nRelay );

        // pos_x_cur = abs( pos_x_next - pos_x_prev )
        ippsSub_32f( (Ipp32f*)&pos_x_prev[ 0 ]
                   , (Ipp32f*)&pos_x_next[ 0 ]
                   , (Ipp32f*)&pos_y_cur[ 0 ]
                   , nRelay );
        ippsAbs_32f( (Ipp32f*)&pos_y_cur[ 0 ]
                   , (Ipp32f*)&pos_x_cur[ 0 ]
                   , nRelay );

        // pos_x_cur = param.lambda1 * pos_x_cur;
        ippsMulC_32f_I( (Ipp32f)param.lambda1
                      , &pos_x_cur[ 0 ]
                      , nRelay );

    }
    int idxRelay = 0;
    for( int i = 0; i < len; ++i ) {
        tmp_ea[ i ] = tmp_a.get( i ) ? pos_x_cur[ idxRelay++ ] : 0.0f;
    }

    // 結果を保存
    ippsAdd_32f( (Ipp32f*)&tmp_ea[ 0 ]
               , (Ipp32f*)&tmp_eb[ 0 ]
               , e
               , len );
    ippsAdd_32f_I( (Ipp32f*)&tmp_ec[ 0 ]
                 , e
                 , len );
}

double energy( TIME_MICRO_SEC time, map<int,TimeToPointIndex>& grpToStatus, map<int,TimeToPointChoice>& choiceOfGroup, CStatusTable& tblConnectable, TrajectoriesInfo& infoTrj, PARAM_RENOVATE_TRAJECTORY param, double* p_t1, double* p_t2 )
{
    double e = 0.0;
    //PerformanceCounter cpc;

    map<int,TimeToPointIndex>::iterator itStatus = grpToStatus.begin();
    for( ; itStatus != grpToStatus.end(); ++itStatus ) {
        int idxGrp = itStatus->first;
        int idxPoint; // 時刻timeでの点番号
        int idxPrevPoint = -1; // 時刻timeの直前の点番号
        int idxNextPoint = -1; // 時刻timeの直後の点番号
        TimeToPointIndex::iterator itIdxPoint, itIdxPrevPoint, itIdxNextPoint;

        if( ( itIdxPoint = itStatus->second.find( time ) ) != itStatus->second.end() ) {
            if( ( idxPoint = itIdxPoint->second ) != -1 ) {
                itIdxNextPoint = itIdxPoint;
            //if( p_t1 ) {
            //    cpc.setStartTime();
            //}
                advance( itIdxNextPoint, 1 );
            //if( p_t1 ) {
            //    *p_t1 += cpc.getUsedTime();
            //}
                if( itIdxNextPoint != itStatus->second.end() ) {
                    //if( find( infoTrj.connectable[ idxPoint ].begin()
                    //        , infoTrj.connectable[ idxPoint ].end()
                    //        , itIdxNextPoint->second ) != infoTrj.connectable[ idxPoint ].end() ) {
                    //    idxNextPoint = itIdxNextPoint->second;
                    //}
                    //if( tblConnectable[ idxPoint ].get( itIdxNextPoint->second ) ) {
                        idxNextPoint = itIdxNextPoint->second;
                    //}
                }

                if( itIdxPoint != itStatus->second.begin() ) {
                    itIdxPrevPoint = itIdxPoint;
            //if( p_t1 ) {
            //    cpc.setStartTime();
            //}
                    advance( itIdxPrevPoint, -1 );
            //if( p_t1 ) {
            //    *p_t1 += cpc.getUsedTime();
            //}
                    if( itIdxPrevPoint->second != -1 ) {
                        //if( find( infoTrj.connectable[ itIdxPrevPoint->second ].begin()
                        //        , infoTrj.connectable[ itIdxPrevPoint->second ].end()
                        //        , idxPoint ) != infoTrj.connectable[ itIdxPrevPoint->second ].end() ) {
                        //    idxPrevPoint = itIdxPrevPoint->second;
                        //}
                        //if( tblConnectable[ itIdxPrevPoint->second ].get( idxPoint ) ) {
                            idxPrevPoint = itIdxPrevPoint->second;
                        //}
                    }
                }
            }

            //if( p_t2 ) {
            //    cpc.setStartTime();
            //}

            // 時刻timeで，idxGrp番目のグループの取りうる点の数
            int nChoice = (int)choiceOfGroup[ idxGrp ].count( time );

            if( idxPoint == -1 || ( idxNextPoint == -1 && idxPrevPoint == -1 ) ) {
                // どの点も利用されないか，idxPointが孤立点である場合のエネルギー
                e += (double)nChoice * param.lambda3;
            } else if( ( idxPoint != -1 && idxNextPoint == -1 ) || ( idxPoint != -1 && idxPrevPoint == -1 ) ) {
                // idxPointが始点または終点
                e += param.lambda2 / 2.0;
                e += (double)( nChoice - 1 ) * param.lambda3;

            } else {
                // idxPointが中継点
                PosXYTID pos, posPrev, posNext;
                pos = infoTrj.points[ idxPoint ];
                posPrev = infoTrj.points[ idxPrevPoint ];
                posNext = infoTrj.points[ idxNextPoint ];

                double v1, v2;
                v1 = sqrt( ( posNext.x - pos.x ) * ( posNext.x - pos.x )
                       + ( posNext.y - pos.y ) * ( posNext.y - pos.y ) )
                   / ( (double)( posNext.t - pos.t ) * 1.0e-6 );
                v2 = sqrt( ( posPrev.x - pos.x ) * ( posPrev.x - pos.x )
                       + ( posPrev.y - pos.y ) * ( posPrev.y - pos.y ) )
                   / ( (double)( pos.t - posPrev.t ) * 1.0e-6 );
                e += param.lambda1 * abs( v1 - v2 );
                e += (double)( nChoice - 1 ) * param.lambda3;
            }

            //if( p_t2 ) {
            //    *p_t2 += cpc.getUsedTime();
            //}
        }
    }

    return e;
}

void makechoicelist( std::map<int,TimeToPointChoice>::iterator itFirst
                                        , std::map<int,TimeToPointChoice>::iterator itLast
                                        , TIME_MICRO_SEC time
                                        , ChoiceList* pListChoice
                                        , std::map<int,int> grpToPointIdx
                                        , HashTable tblHash1
                                        , HashTable tblHash2
                                        , HashTable tblHash3
                                        , HashTable tblHash4
                                        , TrajectoriesInfo& infoTrj
                                        , map<int,TimeToPointIndex>& grpToIdxRsvPoint
                                        , double territory )
{
    if( itFirst == itLast ) {
        pListChoice->push_back( grpToPointIdx );
//        if( grpToPointIdx.empty() ) {
//            cerr << "　　選択肢リスト異常！！　"
//                 << "時刻：" << time
//                 << "グループ構成：";
//            for( std::map<int,TimeToPointChoice>::iterator it = itFirst; itFirst != itLast; ++it ) {
//                cerr << it->first << " ";
//            }
//            cerr << endl;
//        } else {
//            cerr << "選択肢リスト正常(" << time << ")...";
//        }
        return;
    }

    int idxGroup = itFirst->first;
    TimeToPointChoice::iterator it = itFirst->second.lower_bound( time );
    TimeToPointChoice::iterator itEnd = itFirst->second.upper_bound( time );
    if( it == itEnd ) {
        map<int,TimeToPointChoice>::iterator itInc = itFirst;
        ++itInc;
        makechoicelist( itInc
                      , itLast
                      , time
                      , pListChoice
                      , grpToPointIdx
                      , tblHash1
                      , tblHash2
                      , tblHash3
                      , tblHash4
                      , infoTrj
                      , grpToIdxRsvPoint
                      , territory );
    } else {
        // 予約軌跡を構成する点があるかチェック
        TimeToPointIndex::iterator itIdxRsvPoint;
        int idxRsvPoint = -1;
        if( ( itIdxRsvPoint = grpToIdxRsvPoint[ idxGroup ].find( time ) ) != grpToIdxRsvPoint[ idxGroup ].end() ) {
            idxRsvPoint = itIdxRsvPoint->second;
        }

        if( idxRsvPoint == -1 ) {
            map<int,int> tmp = grpToPointIdx;
            tmp[ idxGroup ] = -1;
            map<int,TimeToPointChoice>::iterator itInc = itFirst;
            ++itInc;
            makechoicelist( itInc
                          , itLast
                          , time
                          , pListChoice
                          , tmp
                          , tblHash1
                          , tblHash2
                          , tblHash3
                          , tblHash4
                          , infoTrj
                          , grpToIdxRsvPoint
                          , territory );
        }

        for( ; it != itEnd; ++it ) {
            if( idxRsvPoint == -1 ||  idxRsvPoint == it->second ) {
                map<int,int> tmp = grpToPointIdx;
                tmp[ idxGroup ] = it->second;
                map<int,TimeToPointChoice>::iterator itInc = itFirst;
                ++itInc;

                // ハッシュテーブルを用いて，点itの近傍に他のグループの点が無いかチェックする。
                // 有った場合は点itを使わない。なかった場合はハッシュテーブルに登録して選択肢作成を続ける。
                // ただし，点itが予約軌跡を構成する点の場合は強制的に点itを使用する。
                HashTable th1, th2, th3, th4;
                th1 = tblHash1;
                th2 = tblHash2;
                th3 = tblHash3;
                th4 = tblHash4;
                long hash;
                hash = hash1( infoTrj.points[ it->second ].x, infoTrj.points[ it->second ].y, territory );
                if( !th1[ hash ].empty() && idxRsvPoint == -1 ) {
                    continue;
                }
                th1[ hash ].push_back( it->second );

                hash = hash2( infoTrj.points[ it->second ].x, infoTrj.points[ it->second ].y, territory );
                if( !th2[ hash ].empty() && idxRsvPoint == -1 ) {
                    continue;
                }
                th2[ hash ].push_back( it->second );

                hash = hash3( infoTrj.points[ it->second ].x, infoTrj.points[ it->second ].y, territory );
                if( !th3[ hash ].empty() && idxRsvPoint == -1 ) {
                    continue;
                }
                th3[ hash ].push_back( it->second );

                hash = hash4( infoTrj.points[ it->second ].x, infoTrj.points[ it->second ].y, territory );
                if( !th4[ hash ].empty() && idxRsvPoint == -1 ) {
                    continue;
                }
                th4[ hash ].push_back( it->second );

                makechoicelist( itInc
                              , itLast
                              , time
                              , pListChoice
                              , tmp
                              , th1
                              , th2
                              , th3
                              , th4
                              , infoTrj
                              , grpToIdxRsvPoint
                              , territory );
            }
        }
    }
}

void clustering( int index, vector<int>* pListClass, int label, CStatusTable& adjmat )
{
    if( pListClass->at( index ) == label )
        return;

    pListClass->at( index ) = label;
    for( int i = 0; i < (int)adjmat.size(); ++i ) {
        if( adjmat[ index ].get( i ) ) {
            clustering( i, pListClass, label, adjmat );
        }
    }
}

/*
void grouping_prepare( int root, int index, TrajectoryGroup grp, std::vector<TrajectoryGroup>* pStorageGrp, std::vector<CStatusList>* pStorageGrpBits, vector<int>& idxTrajectory, CStatusTable& tblGrp, CStatusTable& tblNGrp, map<int,int>* pReserve, bool flgContainReserveTrj )
{
    grp.push_back( index );
    pStorageGrp->push_back( grp );

    // grpに追加した軌跡indexが予約軌跡の場合はflgContainReserveTrjをtrueにする。
    if( pReserve->find( index ) != pReserve->end() ) {
        flgContainReserveTrj = true;
    }

    CStatusList grpbit;
    grpbit.SetSize( (int)tblGrp.size() );
    for( TrajectoryGroup::iterator it = grp.begin(); it != grp.end(); ++it ) {
        grpbit.set( *it, true );
    }
    pStorageGrpBits->push_back( grpbit );

    bool flgTerminal = true;

    vector<int>::iterator itIdxTrj = idxTrajectory.begin();

    // index番目の軌跡とidxTrajectoryに含まれる軌跡のグループ化を試みる
    for( ; itIdxTrj != idxTrajectory.end(); ++itIdxTrj ) {
        bool flgNotGroupable = false;

        // grp中の軌跡に，itIdxTrj番目の軌跡とグループ化不可能な軌跡があるか，
        // pReserve中の軌跡が既にあり(flgContainReserveTrj)，itIdxTrj番目の軌跡も予約軌跡の場合は
        // itIdxTrjはグループ化不可能とみなす
        for( TrajectoryGroup::iterator it = grp.begin(); it != grp.end(); ++it ) {
            if( tblNGrp[ *it ].get( *itIdxTrj ) ) {
                flgNotGroupable = true;
                break;
            }
            if( flgContainReserveTrj ) {
                if( pReserve->find( *itIdxTrj ) != pReserve->end() ) {
                    flgNotGroupable = true;
                    break;
                }
            }
        }
        if( *itIdxTrj != index && tblGrp[ index ].get( *itIdxTrj ) && !flgNotGroupable
            && find( grp.begin(), grp.end(), *itIdxTrj ) == grp.end() ) {
            // 下記の条件を満たす場合，itIdxTrj番目の軌跡を同一グループに入れる
            // ・index != itIdxTrj（同一の軌跡は重複してグループに存在しない）
            // ・grpにitIdxTrjが存在しない（同上）
            // ・index番目の軌跡とitIdxTrj番目の軌跡がグループ化可能
            // ・grp内の軌跡とitIdxTrj番目の軌跡がグループ化不可能でない

            grouping_prepare( root, *itIdxTrj, grp, pStorageGrp, pStorageGrpBits, idxTrajectory, tblGrp, tblNGrp, pReserve, flgContainReserveTrj );
            flgTerminal = false;
        }
    }

//    if( flgTerminal && pStorageGrp ) {
//        pStorageGrp->push_back( grp );
//    }
}

void grouping( int index, CStatusList grpbit, vector<TrajectoryGroup>* pStorageGrp, vector<CStatusList>& vecGrpBits, CStatusTable& tblGrp, CStatusTable& tblNGrp )
{
    //int nBit;

    if( index >= (int)vecGrpBits.size() ) {
        if( grpbit.count() > 0 ) {
        //if( ( nBit = accumulate( enable.begin(), enable.end(), 0 ) ) > 0 ) {
            IppiSize size;
            size.width = (int)grpbit.size();
            size.height = 1;

            //
            // grpbitでビットの立っている軌跡番号からグループ化可・不可能な軌跡
            // を求める
            CStatusList groupable, notgroupable;
            groupable.SetSize( (int)tblGrp.size() );
            notgroupable.SetSize( (int)tblGrp.size() );
            for( int i = 0; i < (int)tblGrp.size(); ++i ) {
                if( grpbit.get( i ) ) {
                    ippiOr_8u_C1IR( (Ipp8u*)&(tblGrp[ i ][ 0 ])
                                  , (int)tblGrp[ i ].size()
                                  , (Ipp8u*)&(groupable[ 0 ])
                                  , (int)groupable.size()
                                  , size );
                    ippiOr_8u_C1IR( (Ipp8u*)&(tblNGrp[ i ][ 0 ])
                                  , (int)tblNGrp[ i ].size()
                                  , (Ipp8u*)&(notgroupable[ 0 ])
                                  , (int)notgroupable.size()
                                  , size );
                }
            }

#if 1
            //
            // grpbitが極大なグループかどうかを判定する
            ippiOr_8u_C1IR( (Ipp8u*)&(grpbit[ 0 ])
                          , (int)grpbit.size()
                          , (Ipp8u*)&(groupable[ 0 ])
                          , (int)groupable.size()
                          , size );
            ippiXor_8u_C1IR( (Ipp8u*)&(groupable[ 0 ])
                           , (int)groupable.size()
                           , (Ipp8u*)&(notgroupable[ 0 ])
                           , (int)notgroupable.size()
                           , size );
            ippiAnd_8u_C1IR( (Ipp8u*)&(notgroupable[ 0 ])
                           , (int)notgroupable.size()
                           , (Ipp8u*)&(groupable[ 0 ])
                           , (int)groupable.size()
                           , size );
            if( grpbit == groupable ) {
                // 極大なグループな時はグループに追加
                TrajectoryGroup grp;
                for( int i = 0; i < (int)tblGrp.size(); ++i ) {
                    if( grpbit.get( i ) ) {
                        grp.push_back( i );
                    }
                }
                pStorageGrp->push_back( grp );
            }
#else
            //
            // grpbitがNotGroupableな軌跡同士を含んでいないか調べる
            ippiAnd_8u_C1IR( (Ipp8u*)&(grpbit[ 0 ])
                           , (int)grpbit.size()
                           , (Ipp8u*)&(notgroupable[ 0 ])
                           , (int)notgroupable.size()
                           , size );
            bool flgProper = true;
            for( int i = 0; i < (int)notgroupable.size(); ++i ) {
                if( notgroupable[ i ] != 0 ) {
                    flgProper = false;
                    break;
                }
            }
            if( flgProper ) {
                // 適切なグループの場合は追加
                TrajectoryGroup grp;
                for( int i = 0; i < (int)tblGrp.size(); ++i ) {
                    if( grpbit.get( i ) ) {
                        grp.push_back( i );
                    }
                }
                pStorageGrp->push_back( grp );
            }
#endif
        }
    } else {
        //
        // index番目のvecGrpBitsを使わないときのパターン
        grouping( index + 1, grpbit, pStorageGrp, vecGrpBits, tblGrp, tblNGrp );

        //
        // index番目のvecGrpBitsを使うときのパターン
        bool flgZero = true;
        for( int i = 0; i < (int)grpbit.size() && flgZero; ++i ) {
            flgZero = ( grpbit[ i ] == 0 );
        }

        // grpbitにindex番目のvecGrpBitsをORする
        IppiSize size;
        size.width = (int)grpbit.size();
        size.height = 1;
        ippiOr_8u_C1IR( (Ipp8u*)&(vecGrpBits[ index ][ 0 ])
                      , (int)vecGrpBits[ index ].size()
                      , (Ipp8u*)&(grpbit[ 0 ])
                      , (int)grpbit.size()
                      , size );

        // grpbitが複数のvecGrpBitsを利用してできたパターンで，
        // かつ単一のvecGrpBitsでgrpbitが表現できる場合はそれ以降の探索を行わない
        if( !flgZero && find( vecGrpBits.begin(), vecGrpBits.end(), grpbit ) != vecGrpBits.end() ) {
            return;
        }
        grouping( index + 1, grpbit, pStorageGrp, vecGrpBits, tblGrp, tblNGrp );
    }
}


void makeset( int index, TrajectorySet setTrj, vector<TrajectorySet>* pStorageSet, vector<int> idxTrj, vector<TrajectoryGroup>& grp, vector<TrajectoryWithPointIndex>& trjPointIdx, int& nMinGrp )
{
    if( nMinGrp > 0 && (int)setTrj.size() >= nMinGrp ) {
        return;
    }
    setTrj.push_back( index );
    idxTrj.insert( idxTrj.end(), grp[ index ].begin(), grp[ index ].end() );

    //if( index == grp.size() - 1 ) {
    //    pStorageSet->push_back( setTrj );
    //    return;
    //}

    bool flgTerminal = true;

    for( int i = 0; i < (int)grp.size(); ++i ) {
        if( i != index && find( setTrj.begin(), setTrj.end(), i ) == setTrj.end() ) {
            vector<int> tmp( idxTrj.begin(), idxTrj.end() );
            tmp.insert( tmp.end(), grp[ i ].begin(), grp[ i ].end() );
            sort( tmp.begin(), tmp.end() );
            if( unique( tmp.begin(), tmp.end() ) == tmp.end() ) {
                makeset( i, setTrj, pStorageSet, idxTrj, grp, trjPointIdx, nMinGrp );
                flgTerminal = false;
            }
        }
    }

    if( flgTerminal ) {
        sort( setTrj.begin(), setTrj.end() );
        pStorageSet->push_back( setTrj );
        nMinGrp = (int)setTrj.size();
    }
}
*/


//void subgrouping( int root
//                   , int index
//                   , TrajectoryGroup grp
//                   , vector<TrajectoryGroup>* pStorageGrp
//                   , vector<CStatusList>* pStorageGrpBits
//                   , vector<int>& idxTrajectory
//                   , CStatusTable& tblGrp
//                   , CStatusTable& tblNGrp
//                   , map<int,int>* pReserve
//                   , bool flgContainReserveTrj ) {
//    grp.push_back( index );
//    pStorageGrp->push_back( grp );
//
//    // grpに追加した軌跡indexが予約軌跡の場合はflgContainReserveTrjをtrueにする。
//    if( pReserve->find( index ) != pReserve->end() ) {
//        flgContainReserveTrj = true;
//    }
//
//    CStatusList grpbit;
//    grpbit.SetSize( (int)tblGrp.size() );
//    for( TrajectoryGroup::iterator it = grp.begin(); it != grp.end(); ++it ) {
//        grpbit.set( *it, true );
//    }
//    pStorageGrpBits->push_back( grpbit );
//
//    bool flgTerminal = true;
//
//    vector<int>::iterator itIdxTrj = idxTrajectory.begin();
//
//    // index番目の軌跡とidxTrajectoryに含まれる軌跡のグループ化を試みる
//    for( ; itIdxTrj != idxTrajectory.end(); ++itIdxTrj ) {
//        bool flgNotGroupable = false;
//
//        // grp中の軌跡に，itIdxTrj番目の軌跡とグループ化不可能な軌跡があるか，
//        // pReserve中の軌跡が既にあり(flgContainReserveTrj)，itIdxTrj番目の軌跡も予約軌跡の場合は
//        // itIdxTrjはグループ化不可能とみなす
//        for( TrajectoryGroup::iterator it = grp.begin(); it != grp.end(); ++it ) {
//            if( tblNGrp[ *it ].get( *itIdxTrj ) ) {
//                flgNotGroupable = true;
//                break;
//            }
//            if( flgContainReserveTrj ) {
//                if( pReserve->find( *itIdxTrj ) != pReserve->end() ) {
//                    flgNotGroupable = true;
//                    break;
//                }
//            }
//        }
//        if( *itIdxTrj != index && tblGrp[ index ].get( *itIdxTrj ) && !flgNotGroupable
//            && find( grp.begin(), grp.end(), *itIdxTrj ) == grp.end() ) {
//            // 下記の条件を満たす場合，itIdxTrj番目の軌跡を同一グループに入れる
//            // ・index != itIdxTrj（同一の軌跡は重複してグループに存在しない）
//            // ・grpにitIdxTrjが存在しない（同上）
//            // ・index番目の軌跡とitIdxTrj番目の軌跡がグループ化可能
//            // ・grp内の軌跡とitIdxTrj番目の軌跡がグループ化不可能でない
//
//            subgrouping( root, *itIdxTrj, grp, pStorageGrp, pStorageGrpBits, idxTrajectory, tblGrp, tblNGrp, pReserve, flgContainReserveTrj );
//            flgTerminal = false;
//        }
//    }
//}
void subgrouping( int root
                   , int index
                   , TrajectoryGroup grp
                   , vector<TrajectoryGroup>* pStorageGrp
                   , vector<CStatusList>* pStorageGrpBits
                   , vector<int>& idxTrajectory
                   , TrajectoriesInfo& infoTrj
                   , std::map<int,int>* pReserve
                   , bool flgContainReserveTrj ) {
    grp.push_back( index );
    //pStorageGrp->push_back( grp );

    // grpに追加した軌跡indexが予約軌跡の場合はflgContainReserveTrjをtrueにする。
    if( pReserve->find( index ) != pReserve->end() ) {
        flgContainReserveTrj = true;
    }

    CStatusList grpbit;
    grpbit.SetSize( (int)infoTrj.tableGroupable.size() );
    for( TrajectoryGroup::iterator it = grp.begin(); it != grp.end(); ++it ) {
        grpbit.set( *it, true );
    }

    bool flgTerminal = true;

    vector<int>::iterator itIdxTrj = idxTrajectory.begin();

    // index番目の軌跡とidxTrajectoryに含まれる軌跡のグループ化を試みる
    for( ; itIdxTrj != idxTrajectory.end(); ++itIdxTrj ) {
        bool flgNotGroupable = false;

        // grp中の軌跡に，itIdxTrj番目の軌跡とグループ化不可能な軌跡があるか，
        // pReserve中の軌跡が既にあり(flgContainReserveTrj)，itIdxTrj番目の軌跡も予約軌跡の場合は
        // itIdxTrjはグループ化不可能とみなす
        for( TrajectoryGroup::iterator it = grp.begin(); it != grp.end(); ++it ) {
            bool flgShareTime = ( max( infoTrj.trjElement[ *it ].begin()->t, infoTrj.trjElement[ *itIdxTrj ].begin()->t )
                                    < min ( infoTrj.trjElement[ *it ].rbegin()->t, infoTrj.trjElement[ *itIdxTrj ].rbegin()->t ) );
            if( infoTrj.tableNotGroupable[ *it ].get( *itIdxTrj ) || !flgShareTime ) {
                flgNotGroupable = true;
                break;
            }
            if( flgContainReserveTrj ) {
                if( pReserve->find( *itIdxTrj ) != pReserve->end() ) {
                    flgNotGroupable = true;
                    break;
                }
            }
        }
        if( *itIdxTrj != index && infoTrj.tableGroupable[ index ].get( *itIdxTrj ) && !flgNotGroupable
            && find( grp.begin(), grp.end(), *itIdxTrj ) == grp.end() ) {
            // 下記の条件を満たす場合，itIdxTrj番目の軌跡を同一グループに入れる
            // ・index != itIdxTrj（同一の軌跡は重複してグループに存在しない）
            // ・pGrpにitIdxTrjが存在しない（同上）
            // ・index番目の軌跡とitIdxTrj番目の軌跡がグループ化可能
            // ・grp内の軌跡とitIdxTrj番目の軌跡がグループ化不可能でない

            subgrouping( root, *itIdxTrj, grp, pStorageGrp, pStorageGrpBits, idxTrajectory, infoTrj, pReserve, flgContainReserveTrj );
            flgTerminal = false;
        }
    }

    if( flgTerminal && pStorageGrp ) {
        pStorageGrp->push_back( grp );
    }
    if( flgTerminal && pStorageGrpBits ) {
        pStorageGrpBits->push_back( grpbit );
    }
}

void grouping( int root
               , int index
               , vector<int> subgrps
               , vector<TrajectoryGroup>* pStorageGrp
               , vector<CStatusList>* pStorageGrpBits
               , vector<TrajectoryGroup>& srcSubGrp
               , CStatusTable& tblGrp
               , CStatusTable& tblNGrp
               , map<int,int>* pReserve
               , bool flgContainReserveTrj ) {
    subgrps.push_back( index );
    TrajectoryGroup grp;
    for( vector<int>::iterator it = subgrps.begin(); it != subgrps.end(); ++it ) {
        grp.insert( grp.end(), srcSubGrp[ *it ].begin(), srcSubGrp[ *it ].end() );
    }
    sort( grp.begin(), grp.end() );
    grp.erase( unique( grp.begin(), grp.end() ), grp.end() );

    // grpに追加したサブグループindexがに予約軌跡がある場合はflgContainReserveTrjをtrueにする。
    for( vector<int>::iterator itIdxTrj = srcSubGrp[ index ].begin(); itIdxTrj != srcSubGrp[ index ].end(); ++itIdxTrj ) {
        if( pReserve->find( *itIdxTrj ) != pReserve->end() ) {
            flgContainReserveTrj = true;
            break;
        }
    }

//#define MAXIMAL_GROUP
#ifndef MAXIMAL_GROUP
    pStorageGrp->push_back( grp );
#endif
    CStatusList grpbit;
    grpbit.SetSize( (int)tblGrp.size() );
    for( vector<int>::iterator it = grp.begin(); it != grp.end(); ++it ) {
        grpbit.set( *it, true );
    }
#ifndef MAXIMAL_GROUP
    pStorageGrpBits->push_back( grpbit );
#endif

    bool flgTerminal = true;

    int idxSubGrp = 0;
    vector<TrajectoryGroup>::iterator itSubGrp = srcSubGrp.begin();
    // index番目のサブグループとsrcSubGrpに含まれるサブグループのグループ化を試みる
    for( ; itSubGrp != srcSubGrp.end(); ++itSubGrp, ++idxSubGrp ) {
        bool flgNotGroupable = false;
        bool flgGroupable = false;

        // grp中の軌跡に，itSubGrp内の軌跡とグループ化不可能な軌跡があれば
        // itSubGrpはグループ化不可能とみなす。ただし同一番号の軌跡同士はグループ化不可能としない。
        // またpReserve中の軌跡が既にあり(flgContainReserveTrj)，itSubGrp内にも約軌跡があれば
        // itIdxTrjはグループ化不可能とみなす。
        for( TrajectoryGroup::iterator it1 = grp.begin(); it1 != grp.end() && !flgNotGroupable; ++it1 ) {
            for( TrajectoryGroup::iterator it2 = itSubGrp->begin(); it2 != itSubGrp->end(); ++it2 ) {
                if( ( *it1 != *it2 ) && tblNGrp[ *it1 ].get( *it2 ) ) {
                    flgNotGroupable = true;
                    break;
                }

                if( flgContainReserveTrj ) {
                    if( pReserve->find( *it2 ) != pReserve->end() ) {
                        flgNotGroupable = true;
                        break;
                    }
                }
            }
        }

        // index番目のサブグループ中の軌跡に，itSubGrp内の軌跡とグループ化可能な軌跡があれば
        // itSubGrpはグループ化可能とみなす。ただし同一番号の軌跡同士はグループ化可能としない。
        for( TrajectoryGroup::iterator it1 = srcSubGrp[ index ].begin(); it1 != srcSubGrp[ index ].end() && !flgGroupable; ++it1 ) {
            for( TrajectoryGroup::iterator it2 = itSubGrp->begin(); it2 != itSubGrp->end(); ++it2 ) {
                if( ( *it1 != *it2 ) && tblGrp[ *it1 ].get( *it2 ) ) {
                    flgGroupable = true;
                    break;
                }
            }
        }

        if( index != idxSubGrp && flgGroupable && !flgNotGroupable
            && find( subgrps.begin(), subgrps.end(), idxSubGrp ) == subgrps.end() ) {
            // 下記の条件を満たす場合，itSubGroupを同一グループに入れる
            // ・index != idxSubGrp（同一のサブグループ同士を同一グループに入れない）
            // ・subgrpsにidxSubGrpが存在しない
            // ・flgGroupable == true（subgrps内の軌跡と，itSubGrp内の軌跡にグループ化可能なものがある）
            // ・flgNotGroupable == false（subgrps内の軌跡と，itSubGrp内の軌跡が全てグループ化不可能ではない）
            grouping( root, idxSubGrp, subgrps, pStorageGrp, pStorageGrpBits, srcSubGrp, tblGrp, tblNGrp, pReserve );
            flgTerminal = false;
        }
    }
#ifdef MAXIMAL_GROUP
    if( flgTerminal ) {
        pStorageGrp->push_back( grp );
        pStorageGrpBits->push_back( grpbit );
    }
#endif
}

//void assign_priority( map< int,vector<int> >* pDst, vector<TrajectoryGroup>* pStorageGrp )
//{
//    int max;
//    vector<int> nTrj;
//    for( vector<TrajectoryGroup>::iterator itGrp = pStorageGrp->begin(); itGrp != pStorageGrp->end(); ++itGrp ) {
//        nTrj.push_back( (int)itGrp->size() );
//    }
//    max = *max_element( nTrj.begin(), nTrj.end() );
//
//    for( int idx = 0; idx < (int)pStorageGrp->size(); ++idx ) {
//        (*pDst)[ max - (int)(*pStorageGrp)[ idx ].size() ].push_back( idx );
//    }
//}

void makeset( TrajectorySet setTrj, vector<TrajectorySet>* pStorageSet, vector<int> idxTrj, vector<int>& grpIdx, vector<TrajectoryGroup>& grp )
{
    bool flgTerminal = true;

    for( vector<int>::iterator itIdxGrp = grpIdx.begin(); itIdxGrp != grpIdx.end(); ++itIdxGrp ) {
        if( find( setTrj.begin(), setTrj.end(), *itIdxGrp ) == setTrj.end() ) {
            vector<int> tmp( idxTrj.begin(), idxTrj.end() );
            tmp.insert( tmp.end(), grp[ *itIdxGrp ].begin(), grp[ *itIdxGrp ].end() );
            sort( tmp.begin(), tmp.end() );
            if( unique( tmp.begin(), tmp.end() ) == tmp.end() ) {
                TrajectorySet tmp_setTrj( setTrj.begin(), setTrj.end() );
                vector<int> tmp_idxTrj( idxTrj.begin(), idxTrj.end() );
                tmp_setTrj.push_back( *itIdxGrp );
                tmp_idxTrj.insert( tmp_idxTrj.end(), grp[ *itIdxGrp ].begin(), grp[ *itIdxGrp ].end() );
                makeset( tmp_setTrj, pStorageSet, tmp_idxTrj, grpIdx, grp );
                flgTerminal = false;
            }
        }
    }

    if( flgTerminal ) {
        sort( setTrj.begin(), setTrj.end() );
        pStorageSet->push_back( setTrj );
    }
}

void makeset( vector<TrajectorySet>* pDst
              , int idxGrpStart
              , TrajectorySet trjSet
              , CStatusList trjSetBit
              , set<CStatusList>& setSearched
              , vector< vector<GroupCombiniation> >& idxGrpToCmb
              , vector< vector<CStatusList> >& idxGrpToCmbBit
              , vector<TrajectoryGroup>& grp )
{
    bool flgTerminal = true;

    if( setSearched.find( trjSetBit ) != setSearched.end() ) {
        return;
    }

    setSearched.insert( trjSetBit );


    for( int idxGrp = idxGrpStart; idxGrp < (int)idxGrpToCmb.size(); ++idxGrp ) {
        // 現在のセットtrjSetの一部をidxGrp番目のグループで置き換えられるか調べる
        vector<GroupCombiniation>::iterator itGrpCmb = idxGrpToCmb[ idxGrp ].begin();
        for( ; itGrpCmb != idxGrpToCmb[ idxGrp ].end(); ++itGrpCmb/*, ++itGrpCmbBit*/ ) {
            if( includes( trjSet.begin(), trjSet.end(), itGrpCmb->begin(), itGrpCmb->end() ) ) {
                // 置換可能な場合は，trjSetからitGrpCmbに含まれるグループを消去し，idxGrpを追加する。
                TrajectorySet tmpTrjSet;
                set_difference( trjSet.begin()
                              , trjSet.end()
                              , itGrpCmb->begin()
                              , itGrpCmb->end()
                              , inserter( tmpTrjSet, tmpTrjSet.begin() ) );
                tmpTrjSet.push_back( idxGrp );
                sort( tmpTrjSet.begin(), tmpTrjSet.end() );
                CStatusList tmpTrjSetBit;
                tmpTrjSetBit.SetSize( (int)grp.size() );
                for( int i = 0; i < (int)tmpTrjSet.size(); ++i ) {
                    tmpTrjSetBit.set( tmpTrjSet[ i ], true );
                }

                // さらに置換を試みる
                flgTerminal = false;
                makeset( pDst, idxGrp + 1, tmpTrjSet, tmpTrjSetBit, setSearched, idxGrpToCmb, idxGrpToCmbBit, grp );

                break;
            }
        }
    }

    if( flgTerminal ) {
        // これ以上置換が不可能な場合はセットとして登録
        pDst->push_back( trjSet );
    }
}

void inclusive_relation( vector<GroupCombiniation>* pDst
                                            , int idxGrpSrc
                                            , GroupCombiniation grpCmb
                                            , CStatusList bit
                                            , vector<TrajectoryGroup>& grp
                                            , vector<CStatusList>& grpBits )
{
    for( int idxGrp = 0; idxGrp < (int)grp.size(); ++idxGrp ) {
        if( ( idxGrp != idxGrpSrc ) && ( grp[ idxGrp ].size() < grp[ idxGrpSrc ].size() ) && ( find( grpCmb.begin(), grpCmb.end(), idxGrp ) == grpCmb.end() ) ) {
            // idxGrpSrcのグループがidxGrpの表すグループを含むかどうか調べる
            CStatusList tmpBit = bit;
            ippsOr_8u( (Ipp8u*)&(grpBits[ idxGrpSrc ][ 0 ])
                     , (Ipp8u*)&(grpBits[ idxGrp ][ 0 ])
                     , (Ipp8u*)&(tmpBit[ 0 ])
                     , (int)tmpBit.size() );
            if( grpBits[ idxGrpSrc ] == tmpBit ) {
                // 現在のグループビットが示す軌跡と，idxGrpの含む軌跡が干渉しないか調べる
                CStatusList collisionBit;
                collisionBit.resize( bit.size() );
                ippsAnd_8u( (Ipp8u*)&(grpBits[ idxGrp ][ 0 ])
                          , (Ipp8u*)&(bit[ 0 ])
                          , (Ipp8u*)&(collisionBit[ 0 ])
                          , (int)collisionBit.size() );
                if( collisionBit.count() == 0 ) {
                    // idxGrp番目のグループを追加
                    GroupCombiniation tmpGrpCmb( grpCmb.begin(), grpCmb.end() );
                    tmpGrpCmb.push_back( idxGrp );
                    tmpBit = bit;
                    ippsOr_8u_I( (Ipp8u*)&(grpBits[ idxGrp ][ 0 ])
                               , (Ipp8u*)&(tmpBit[ 0 ])
                               , (int)tmpBit.size() );
                    if( grpBits[ idxGrpSrc ] == tmpBit ) {
                        // 現在のグループの組み合わせでidxGrpSrc番目のグループを完全に表現できた場合は帰る
                        sort( tmpGrpCmb.begin(), tmpGrpCmb.end() );
                        pDst->push_back( tmpGrpCmb );
                        return;
                    } else {
                        // 他のグループの追加を試みる
                        inclusive_relation( pDst, idxGrpSrc, tmpGrpCmb, tmpBit, grp, grpBits );
                    }
                }
            }

        }
    }
}

void _rectifyset( vector<TrajectorySet>* pDst
                  , TrajectorySet trjSet
                  , CStatusList trjSetBit
                  , TrajectorySet& srcSet
                  , CStatusTable& tableGroupNear
                  , CStatusList& grpReserved
                  , vector<TrajectoryGroup>& grp
                  , CStatusTable& tblNGrp
                  , CStatusTable& tblNear
                  , CStatusTable& tblVeryNear
                  , map<int,int>& reserve
                  , vector< vector<GroupCombiniation> >& idxGrpToCmb
                  , vector< vector<CStatusList> >& idxGrpToCmbBit )
{
    bool flgTerminal = true;

    for( int offset = 0; offset < (int)srcSet.size(); ++offset ) {
            // 現在のセットtrjSetへのグループsrcSet[ offset ]の追加を試みる
        if( !trjSetBit.get( offset ) ) {
            bool flgAdd = true;
            // グループsrcSet[ offset ]がtrjSet中のグループと近接しないか調べる。
            CStatusList tmpTrjSetBit = trjSetBit;
            tmpTrjSetBit.set( offset, true );
            CStatusList grpNear;
            grpNear.SetSize( (int)srcSet.size() );
            for( int i = 0; i < (int)srcSet.size(); ++i ) {
                if( tmpTrjSetBit.get( i ) ) {
                    ippsAnd_8u( (Ipp8u*)&(tmpTrjSetBit[ 0 ]), (Ipp8u*)&(tableGroupNear[ i ][ 0 ]), (Ipp8u*)&(grpNear[ 0 ]), (int)grpNear.size() );
                    if( grpNear.count() >= 1 ) {
                        flgAdd = false;
                        break;
                    }
                }
            }
            if( flgAdd ) {
                // 近接しなければ追加
                TrajectorySet tmpTrjSet = trjSet;
                tmpTrjSet.push_back( srcSet[ offset ] );
                sort( tmpTrjSet.begin(), tmpTrjSet.end() );
                _rectifyset( pDst, tmpTrjSet, tmpTrjSetBit, srcSet, tableGroupNear, grpReserved, grp, tblNGrp, tblNear, tblVeryNear, reserve, idxGrpToCmb, idxGrpToCmbBit );
                flgTerminal = false;
            } else {
                // 近接していれば,包含関係を元にグループsrcSet[ offset ]を分割して追加を試みる。
                TrajectorySet tmpSrcSet = srcSet;
                tmpSrcSet.erase( find( tmpSrcSet.begin(), tmpSrcSet.end(), tmpSrcSet[ offset ] ) );
                vector<TrajectorySet>::iterator itCmb = idxGrpToCmb[ srcSet[ offset ] ].begin();
                for( ; itCmb != idxGrpToCmb[ srcSet[ offset ] ].end(); ++itCmb ) {
                    TrajectorySet tmp = tmpSrcSet;
                    tmp.insert( tmp.end(), itCmb->begin(), itCmb->end() );
                    sort( tmp.begin(), tmp.end() );
                    rectifyset( pDst, trjSet, tmp, grp, tblNGrp, tblNear, tblVeryNear, reserve, idxGrpToCmb, idxGrpToCmbBit );
                }
            }
        }
    }

    if( flgTerminal ) {
        pDst->push_back( trjSet );
    }
}

void rectifyset( vector<TrajectorySet>* pDst
                 , TrajectorySet& initSet
                 , TrajectorySet& srcSet
                 , vector<TrajectoryGroup>& grp
                 , CStatusTable& tblNGrp
                 , CStatusTable& tblNear
                 , CStatusTable& tblVeryNear
                 , map<int,int>& reserve
                 , vector< vector<GroupCombiniation> >& idxGrpToCmb
                 , vector< vector<CStatusList> >& idxGrpToCmbBit
                 , bool strictCheck )
{
    // 予約軌跡を含むグループを列挙する
    vector<int> idxRsvTrj;
    for( map<int,int>::iterator itRsvTrj = reserve.begin(); itRsvTrj != reserve.end(); ++itRsvTrj ) {
        idxRsvTrj.push_back( itRsvTrj->first );
    }

    TrajectorySet trjSet;
    CStatusList trjSetBit;
    trjSetBit.SetSize( (int)srcSet.size() );

    CStatusList grpReserved;
    grpReserved.SetSize( (int)srcSet.size() );
    for( int offset = 0; offset < (int)srcSet.size(); ++offset ) {
        vector<int> idxRsvTrjInThisGrp;
        int idxGrp = srcSet[ offset ];
        set_intersection( grp[ idxGrp ].begin()
                        , grp[ idxGrp ].end()
                        , idxRsvTrj.begin()
                        , idxRsvTrj.end()
                        , inserter( idxRsvTrjInThisGrp, idxRsvTrjInThisGrp.begin() ) );
        if( !idxRsvTrjInThisGrp.empty() ) {
            grpReserved.set( offset, true );
            trjSet.push_back( srcSet[ offset ] );
            trjSetBit.set( offset, true );
        }
    }
    sort( trjSet.begin(), trjSet.end() );

    // 任意の２つのグループについて，近接しているかどうかを表すテーブルを作成する。
    CStatusTable tableGroupNear;
    tableGroupNear.SetSize( (int)srcSet.size() );
    for( int offset1 = 0; offset1 < (int)srcSet.size(); ++offset1 ) {
        for( int offset2 = offset1 + 1; offset2 < (int)srcSet.size(); ++offset2 ) {
            int idxGrp1 = srcSet[ offset1 ];
            int idxGrp2 = srcSet[ offset2 ];
            bool flgEscape = false;
            for( TrajectoryGroup::iterator itIdxTrj1 = grp[ idxGrp1 ].begin(); itIdxTrj1 != grp[ idxGrp1 ].end() && !flgEscape; ++itIdxTrj1 ) {
                for( TrajectoryGroup::iterator itIdxTrj2 = grp[ idxGrp2 ].begin(); itIdxTrj2 != grp[ idxGrp2 ].end() && !flgEscape; ++itIdxTrj2 ) {
                    if( tblNear[ *itIdxTrj1 ].get( *itIdxTrj2 ) ) {
                        tableGroupNear[ offset1 ].set( offset2, true );
                        tableGroupNear[ offset2 ].set( offset1, true );
                        flgEscape = true;

                    }
                    if( strictCheck ) {
                        // 予約グループ同士が極めて近接している場合はセットを破棄
                        if( tblVeryNear[ *itIdxTrj1 ].get( *itIdxTrj2 ) ) {
                            if( grpReserved.get( offset1 ) && grpReserved.get( offset2 ) ) {
                                //cerr << "　　　セットを破棄" << endl;
                                return;
                            }
                        }
                    }
                }
            }
        }
    }

    // 初期セットinitSetに基づきtrjSet,trjSetBitの情報を更新
    for( TrajectorySet::iterator it = initSet.begin(); it != initSet.end(); ++it ) {
        int offset = (int)distance( srcSet.begin(), find( srcSet.begin(), srcSet.end(), *it ) );
        if( !trjSetBit.get( offset ) ) {
            trjSet.push_back( *it );
            trjSetBit.set( offset, true );
        }
    }

    _rectifyset( pDst, trjSet, trjSetBit, srcSet, tableGroupNear, grpReserved, grp, tblNGrp, tblNear, tblVeryNear, reserve, idxGrpToCmb, idxGrpToCmbBit );

}
