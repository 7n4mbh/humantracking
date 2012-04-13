/**
 * @file Trajectory.h
 * @brief 軌跡作成コンポーネントクラス
 *
 * @author 福司 謙一郎
 * @date 2009
 */

#ifndef _TRAJECTORY_H
#define _TRAJECTORY_H

#include <vector>
#include <list>
#include <set>
#include <map>
#include "PersonTrackingCommon.h"
#include <ipp.h>

/**
 * 軌跡の構成要素
 */
typedef std::set<PosXYTID,PosXYT_T_Less> TrajectoryElement;

/**
 * 軌跡クラス
 */
class CTrajectory : public std::list<TrajectoryElement> {
    friend std::ostream& operator<<( std::ostream& os, CTrajectory& obj );
public:
    /**
     * 開始時刻と終了時刻を指定して軌跡を切り出す<br>
     * 切り出し後の軌跡は開始時刻と終了時刻を含む。
     */
    void Clip( TIME_MICRO_SEC timeBegin, TIME_MICRO_SEC timeEnd );

    /**
     * 軌跡の構成要素を統合する
     * @param pDst 統合後の軌跡
     */
    void Integrate( CTrajectory* pDst );
};

/**
 * 追跡結果軌跡とIDのマップ
 */
typedef std::map<unsigned int,CTrajectory> TrackingResultTrajectories;

/**
 * CTrajectory型の比較のための関数オブジェクト
 */
class CTrajectory_LatestTime_Less {
public:
    bool operator() ( CTrajectory& obj1, TIME_MICRO_SEC t ) const;
    bool operator() ( TIME_MICRO_SEC t, CTrajectory& obj1 ) const;
    bool operator() ( CTrajectory& obj1, CTrajectory& obj2 ) const;
};

/**
 * ２つのTrajectoryElement間の距離を定義する関数オブジェクト
 */
class TrajectoryElement_Distance {
public:
    /**
     * コンストラクタ
     */
    TrajectoryElement_Distance( double distanceLimit, unsigned int nLimit, TIME_MICRO_SEC minCommonTimeRange, bool flgUseID = true ) {
        this->distanceLimit = distanceLimit;
        this->nLimit = nLimit;
        this->minCommonTimeRange = minCommonTimeRange;
        this->flgUseID = flgUseID;
    }

    /**
     * ２つのTrajectoryElement間の距離[m]を返す
     * @retval 0以上 ２つのTrajectoryElement間の距離[m]
     * @retval 負数 距離は無限大
     */
    double operator()( TrajectoryElement& obj1, TrajectoryElement& obj2 );

private:
    double distanceLimit; ///< ２つのTrajectoryElementでdistanceLimit[m]以上の値をとる部分がnLimit[個]以上ある場合は，距離を無限大とする。
    unsigned int nLimit; ///< distanceLimitを参照
    TIME_MICRO_SEC minCommonTimeRange; ///< ２つのTrajectoryElementがminCommonTimeRange[usec]未満の共通部分しかない場合は距離を無限大とする。
    bool flgUseID; ///< 距離を評価する場合に色情報に基づくIDを利用するかどうか。
};

/**
 * ２つの軌跡間の距離を定義する関数オブジェクト
 */
class CTrajectory_Distance {
public:
    /**
     * コンストラクタ
     */
    CTrajectory_Distance( double distanceLimit, unsigned int nLimit, TIME_MICRO_SEC minCommonTimeRange, bool flgUseID = true )
        : distanceTrajectoryElement( distanceLimit, nLimit, minCommonTimeRange, flgUseID )
    {
        this->distanceLimit = distanceLimit;
        this->nLimit = nLimit;
        this->minCommonTimeRange = minCommonTimeRange;
    }

    /**
     * ２つの軌跡間の距離[m]を返す
     * @retval 0以上 ２つの軌跡間の距離[m]
     * @retval 負数 距離は無限大
     */
    double operator()( CTrajectory& obj1, CTrajectory& obj2 );

private:
    double distanceLimit; ///< ２つの軌跡でdistanceLimit[m]以上の値をとる部分がnLimit[個]以上ある場合は，距離を無限大とする。
    unsigned int nLimit; ///< distanceLimitを参照
    TIME_MICRO_SEC minCommonTimeRange; ///< ２つの軌跡がminCommonTimeRange[usec]未満の共通部分しかない場合は距離を無限大とする。
    TrajectoryElement_Distance distanceTrajectoryElement;
};

inline double TrajectoryElement_Distance::operator()( TrajectoryElement& obj1, TrajectoryElement& obj2 )
{
    TIME_MICRO_SEC timeBegin; // 距離を評価する開始時間
    TIME_MICRO_SEC timeEnd; // 距離を評価する終了時間

    // 共通する時間（距離を評価する時間範囲）を調べる
    timeBegin = max( obj1.begin()->t, obj2.begin()->t );
    timeEnd = min( obj1.rbegin()->t, obj2.rbegin()->t );

    // 共通時間がない場合は無限大を返す
    if( timeBegin > timeEnd ) {
        return -0.3;
    }

    // 共通時間がminCommonTimeRange[usec]未満の場合は無限大を返す
    if( ( timeEnd - timeBegin ) < minCommonTimeRange ) {
        return -0.4;
    }

//    // テスト
//    // 共通時間が長い方の軌跡の8割未満の場合は無限大を返す
//    TIME_MICRO_SEC timeLenMax = max( obj1.rbegin()->t - obj1.begin()->t, obj2.rbegin()->t - obj2.begin()->t );
//    if( ( timeEnd - timeBegin ) < ( 50 * timeLenMax ) / 100 ) {
//        return -0.4;
//    }

    if( flgUseID ) {
        // obj1のIDを求める
        int id1 = 0;
        TrajectoryElement::iterator itPos;
        for( itPos = obj1.begin(); itPos != obj1.end() && id1 == 0; ++itPos ) {
            id1 = itPos->ID;
        }

        // ob1のIDを求める
        int id2 = 0;
        for( itPos = obj2.begin(); itPos != obj2.end() && id2 == 0; ++itPos ) {
            id2 = itPos->ID;
        }

        // obj1とobj2のIDがいずれも0以外で，かつ一致しない場合は無限大を返す
        if( ( id1 != 0 ) && ( id2 != 0 ) && ( id1 != id2 ) ) {
            return -1.0;
        }
    }

    // 共通する時刻でXY座標の差を計算する
    double ex = 0.0, ey = 0.0;
    unsigned int cnt = 0, size = 0;
    const double dlsq = distanceLimit * distanceLimit;
    double max = 0.0;
    TrajectoryElement::iterator it1, it2;
    it1 = obj1.find( PosXYT( 0.0, 0.0, timeBegin ) );
    it2 = obj2.find( PosXYT( 0.0, 0.0, timeBegin ) );
    for( ; it1 != obj1.end() && it1->t <= timeEnd && it2 != obj2.end() && it2->t <= timeEnd; ++it1, ++it2, ++size ) {
        double dx = it1->x - it2->x;
        double dy = it1->y - it2->y;
        double dsq = dx * dx + dy * dy;
        if( dsq > dlsq/*distanceLimit * distanceLimit*/ ) {
            cnt++;
            if( cnt > nLimit ) {
                return -1.0;
            }
        }
        ex += dx;
        ey += dy;

        double d = sqrt( dsq );
        if( d > max ) {
            max = d;
        }
    }
    ex /= (double)size;
    ey /= (double)size;

    return sqrt( ex * ex + ey * ey );
}

inline double CTrajectory_Distance::operator()( CTrajectory& obj1, CTrajectory& obj2 )
{
    CTrajectory::iterator itObj1, itObj2;
    double min = -2.0;
    double distance;

    for( itObj1 = obj1.begin(); itObj1 != obj1.end(); ++itObj1 ) {
        for( itObj2 = obj2.begin(); itObj2 != obj2.end(); itObj2++ ) {
            distance = distanceTrajectoryElement( *itObj1, *itObj2 );
            //if( distance >= 0.0 ) {
                if( min < -1.0  || ( min < 0.0 && distance < 0.0 && min < distance ) || ( distance >= 0.0 && min > distance ) ) {
                    min = distance;
                }
            //}
        }
    }

    return min;
}

#endif /* _TRAJECTORY_H */
