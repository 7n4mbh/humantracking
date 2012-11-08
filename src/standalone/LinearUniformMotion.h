/**
 * @file LinearUniformMotion.h
 * @brief 等速直線運動クラス
 *
 * @author 福司 謙一郎
 * @date 2009
 */

#include <math.h>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include "PosXYT.h"

#ifndef _LINEARUNIFORMMOTION_H
#define _LINEARUNIFORMMOTION_H

/**
 * ある時刻における等速直線運動の情報
 */
class LUMSlice {
public:
    PosXYTID pos; ///< 時刻と位置とID
    double gradX, gradY; ///< posにおける,X方向およびY方向の傾き
};

/**
 * ２つのInfoLUMSlice型の距離を定義する関数オブジェクト
 */
class LUMSlice_Dist {
public:
    double operator()( LUMSlice& obj1, LUMSlice& obj2 ) {
        PosXYT_Dist dist;
        return dist( obj1.pos, obj2.pos );
    }
};

/**
 * 等速直線運動クラス
 *
 * 等速直線運動はbeginPos(x1,y1,t1)およびendPos(x2,y2,t2)の２点を結ぶ<br>
 * 線分で表現する。ただしt1<=t2とする。
 */
class CLinearUniformMotion {
public:
    /**
     * コンストラクタ
     */
    CLinearUniformMotion() {
        beginPos = PosXYTID( 0.0, 0.0, 0, 0 );
        endPos = PosXYTID( 0.0, 0.0, 0, 0 );
    }

    /**
     * コンストラクタ
     */
    CLinearUniformMotion( PosXYTID pos1, PosXYTID pos2 ) {
        if( pos1.t < pos2.t ) {
            beginPos = pos1;
            endPos = pos2;
        } else {
            beginPos = pos2;
            endPos = pos1;
        }
    }

    /**
     * 時刻の差を返す
     * @return 時刻の差（endPos.t - beginPos.t）
     */
    TIME_MICRO_SEC GetTimeDifference() { return endPos.t - beginPos.t; }

    /**
     * 速さを返す
     * @return 等速直線運動の速さ[m/s]
     */
    double GetSpeed() {
        double dx, dy;
        dx = beginPos.x - endPos.x;
        dy = beginPos.y - endPos.y;
        return sqrt( dx * dx + dy * dy ) / ( (double)GetTimeDifference() / 1.0e6 );
    }

    /**
     * 指定した時刻が等速直線運動の範囲内にあるかどうかを返す
     * @retval 0以外 指定した時刻が等速直線運動の範囲内にある
     * @retval 0 指定した時刻が等速直線運動の範囲外にある
     */
    int IsInRange( TIME_MICRO_SEC time ) {
        return ( time >= beginPos.t && time <= endPos.t );
    }

    /**
     * 始点位置を返す
     */
    PosXYT GetBeginPos() const { return beginPos; }

    /**
     * 終点位置を返す
     */
    PosXYT GetEndPos() const { return endPos; }

    /**
     * X方向の傾きを返す
     * @return X方向の傾き[m/us]
     */
    double GetGradX() {
        return ( endPos.x - beginPos.x ) / (double)GetTimeDifference();
    }

    /**
     * Y方向の傾きを返す
     * @return Y方向の傾き[m/us]
     */
    double GetGradY() {
        return ( endPos.y - beginPos.y ) / (double)GetTimeDifference();
    }

    /**
     * 始点と終点のIDが同一，かつ0以外のときtrueを返す
     */
    bool IsColoredLUM() {
        return ( ( beginPos.ID != 0 ) && ( endPos.ID != 0 ) && ( beginPos.ID == endPos.ID ) );
    }

    /**
     * 任意の時刻での等速直線運動の情報を返す
     * @param time 時刻
     * @param pLUMSlice 情報の格納先
     * @retval 0以外 成功
     * @retval 0 指定した時刻が等速直線運動の範囲外にあるため失敗
     */
    int GetLUMSlice( TIME_MICRO_SEC time, LUMSlice* pLUMSlice ) {
        if( IsInRange( time ) ) {
            pLUMSlice->gradX = GetGradX();
            pLUMSlice->gradY = GetGradY();
            pLUMSlice->pos.x = pLUMSlice->gradX * (double)( time - beginPos.t ) + beginPos.x;
            pLUMSlice->pos.y = pLUMSlice->gradY * (double)( time - beginPos.t ) + beginPos.y;
            pLUMSlice->pos.t = time;
            pLUMSlice->pos.ID = ( beginPos.ID != 0 ) ? beginPos.ID : endPos.ID;
            return TRUE;
        } else {
            return FALSE;
        }

    }

private:
    PosXYTID beginPos; ///< 始点位置
    PosXYTID endPos; ///< 終点位置
};

/**
 * PosXYT型の比較のための関数オブジェクト
 */
class LUM_EndTime_Less : public std::binary_function< CLinearUniformMotion, CLinearUniformMotion, bool > {
public:
    bool operator() ( const CLinearUniformMotion& obj1, const CLinearUniformMotion& obj2 ) const {
        return obj1.GetEndPos().t < obj2.GetEndPos().t;
    }
};

/**
 * 時刻と等速直線運動の情報のテーブル
 */
typedef std::map< TIME_MICRO_SEC, std::vector< LUMSlice > > LUMSliceTable;


#endif /* _LINEARUNIFORMMOTION_H */
