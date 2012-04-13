/**
 * @file Trajectory.cpp
 * @brief 軌跡クラス
 *
 * @author 福司 謙一郎
 * @date 2009
 */

#include <iostream>
#include <algorithm>
#include "Trajectory.h"

using namespace std;


void CTrajectory::Clip( TIME_MICRO_SEC timeBegin, TIME_MICRO_SEC timeEnd )
{
    CTrajectory::iterator itTrjElement;
    TrajectoryElement::iterator itErase;

    for( itTrjElement = begin(); itTrjElement != end(); ++itTrjElement ) {
        itErase = itTrjElement->lower_bound( PosXYT( 0.0, 0.0, timeBegin ) );
        itTrjElement->erase( itTrjElement->begin(), itErase );
        itErase = itTrjElement->upper_bound( PosXYT( 0.0, 0.0, timeEnd ) );
        itTrjElement->erase( itErase, itTrjElement->end() );
    }
   
    // 長さが２未満の軌跡の要素を消去
    for( itTrjElement = begin(); itTrjElement != end(); ) {
        if( itTrjElement->size() < 2 ) {
            itTrjElement = erase( itTrjElement );
        } else {
            ++itTrjElement;
        }
    }
}

void CTrajectory::Integrate( CTrajectory* pDst )
{
    multiset<PosXYTID,PosXYT_T_Less> elements;

    // 軌跡の構成要素すべての位置情報をelementsに挿入する
    for( CTrajectory::iterator itTrjElement = begin(); itTrjElement != end(); ++itTrjElement ) {
        elements.insert( itTrjElement->begin(), itTrjElement->end() );
    }

    // IDを決める
    int id = 0;
    for( multiset<PosXYTID,PosXYT_T_Less>::iterator itPos = elements.begin(); itPos != elements.end() && id == 0; ++itPos ) {
        id = itPos->ID;
    }


    // pDstの初期化
    pDst->resize( 1 );
    pDst->front().clear();

    // elements内で同一の時刻を持つ位置情報について平均位置を計算し，pDstに格納する。
    multiset<PosXYTID,PosXYT_T_Less>::iterator itBegin, itEnd;
    itBegin = elements.begin();
    while( itBegin != elements.end() ) {
        itEnd = elements.upper_bound( PosXYT( 0.0, 0.0, itBegin->t ) );
        PosXYTID pos( 0.0, 0.0, itBegin->t, id );
        unsigned int n = 0;
        for( ; itBegin != itEnd; ++itBegin, ++n ) {
            pos.x += itBegin->x;
            pos.y += itBegin->y;
        }
        pos.x /= n;
        pos.y /= n;
        pDst->front().insert( pos );
    }

}

ostream& operator<<( ostream& os, CTrajectory& obj )
{
    CTrajectory::iterator itTrjElement;
    int i = 0;
    for( itTrjElement = obj.begin(); itTrjElement != obj.end(); ++itTrjElement, ++i ) {
        TrajectoryElement::iterator itPos;
        for( itPos = itTrjElement->begin(); itPos != itTrjElement->end(); ++itPos ) {
            os << itPos->x << ", " << itPos->y << ", " << (double)itPos->t / 1.0e6 << endl;
        }
        os << endl;
    }

    return os;
}

bool CTrajectory_LatestTime_Less::operator() ( CTrajectory& obj1, TIME_MICRO_SEC t ) const
{
    TIME_MICRO_SEC latestTime_obj1;
    CTrajectory::iterator itTrjElement;

    // obj1の最新時刻を調べる
    latestTime_obj1 = 0;
    for( itTrjElement = obj1.begin(); itTrjElement != obj1.end(); ++itTrjElement ) {
        TIME_MICRO_SEC latestTime = itTrjElement->rbegin()->t;
        if( latestTime_obj1 < latestTime ) {
            latestTime_obj1 = latestTime;
        }
    }

    return latestTime_obj1 < t;
}

bool CTrajectory_LatestTime_Less::operator() ( TIME_MICRO_SEC t, CTrajectory& obj1 ) const
{
    TIME_MICRO_SEC latestTime_obj1;
    CTrajectory::iterator itTrjElement;

    // obj1の最新時刻を調べる
    latestTime_obj1 = 0;
    for( itTrjElement = obj1.begin(); itTrjElement != obj1.end(); ++itTrjElement ) {
        TIME_MICRO_SEC latestTime = itTrjElement->rbegin()->t;
        if( latestTime_obj1 < latestTime ) {
            latestTime_obj1 = latestTime;
        }
    }

    return t < latestTime_obj1;
}

bool CTrajectory_LatestTime_Less::operator ()(CTrajectory &obj1, CTrajectory &obj2) const
{
    TIME_MICRO_SEC latestTime_obj1, latestTime_obj2;
    CTrajectory::iterator itTrjElement;

    // obj1の最新時刻を調べる
    latestTime_obj1 = 0;
    for( itTrjElement = obj1.begin(); itTrjElement != obj1.end(); ++itTrjElement ) {
        TIME_MICRO_SEC latestTime = itTrjElement->rbegin()->t;
        if( latestTime_obj1 < latestTime ) {
            latestTime_obj1 = latestTime;
        }
    }

    //obj2の最新時刻を調べる
    latestTime_obj2 = 0;
    for( itTrjElement = obj2.begin(); itTrjElement != obj2.end(); ++itTrjElement ) {
        TIME_MICRO_SEC latestTime = itTrjElement->rbegin()->t;
        if( latestTime_obj2 < latestTime ) {
            latestTime_obj2 = latestTime;
        }
    }

    return latestTime_obj1 < latestTime_obj2;
}

