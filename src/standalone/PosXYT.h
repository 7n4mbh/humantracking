/**
 * @file PosXYT.h
 * @brief 位置情報に関するクラス
 *
 * @author 福司 謙一郎
 * @date 2009
 */

#ifndef _POSXYT_H
#define _POSXYT_H

#ifdef WIN32
#include <windows.h>
#endif

#include <math.h>
#include <functional>
#include <utility>
#include <iostream>
#include <fstream>
#include "pttype.h"

/**
 * XY座標・時刻構情報
 */
class PosXYT {
public:
    PosXYT() {
        this->x = 0.0;
        this->y = 0.0;
        this->t = 0;
    }

    PosXYT( double x, double y, ULONGLONG t ) {
        this->x = x;
        this->y = y;
        this->t = t;
    }

public:
    double x, y; ///< X,Y座標[m]
    TIME_MICRO_SEC t; ///< 時刻[usec]
};

/**
 * PEPMap上の値（人物存在確率）を付加したXY座標・時刻情報
 */
class PosXYTV : public PosXYT {
public:
    PosXYTV()
        : PosXYT() {
        this->value = 0.0;
    }

    PosXYTV( double x, double y, TIME_MICRO_SEC t, double value )
        : PosXYT( x, y, t ) {
        this->value = value;
    }
public:
    double value; ///< PEPMap上の値（Plan-View Occupancy Mapにおける占有率）
};

/**
 * PEPMap上の値（人物存在確率）と人物IDを付加したXY座標・時刻情報
 */
class PosXYTVID : public PosXYTV {
public:
    PosXYTVID()
        : PosXYTV() {
        this->ID = 0;
    }

    PosXYTVID( double x, double y, TIME_MICRO_SEC t, double value, unsigned int ID )
        : PosXYTV( x, y, t, value ) {
        this->ID = ID;
    }
public:
    unsigned int ID; // 人物ID（1以上:有効な人物ID 0:無効な人物ID）
};

/**
 * 人物IDを付加したXY座標・時刻情報
 */
class PosXYTID : public PosXYT {
public:
    PosXYTID()
        : PosXYT() {
        this->ID = 0;
    }

    PosXYTID( double x, double y, TIME_MICRO_SEC t, unsigned int ID )
        : PosXYT( x, y, t ) {
        this->ID = ID;
    }

    PosXYTID( PosXYTVID pos ) : PosXYT( pos.x, pos.y, pos.t ) {
        this->ID = pos.ID;
    }

    PosXYTID( PosXYT pos ) : PosXYT( pos.x, pos.y, pos.t ) {
        this->ID = 0;
    }

    /**
     * 出力ストリーム
     */
    friend std::ostream& operator<<( std::ostream& os, PosXYTID& pos );

    /**
     * 入力ストリーム
     */
    friend std::istream& operator>>( std::istream& is, PosXYTID& pos );

    /**
     *  代入演算子
     */
    PosXYTID operator = ( PosXYTVID pos ) {
        this->x = pos.x;
        this->y = pos.y;
        this->t = pos.t;
        this->ID = pos.ID;

        return *this;
    }

    PosXYTID operator = ( PosXYT pos ) {
        this->x = pos.x;
        this->y = pos.y;
        this->t = pos.t;
        this->ID = 0;

        return *this;
    }

public:
    unsigned int ID; // 人物ID（1以上:有効な人物ID 0:無効な人物ID）
};

/**
 * PosXYT型の比較のための関数オブジェクト
 */
class PosXYT_T_Less : public std::binary_function< PosXYT, PosXYT, bool > {
public:
    bool operator() ( const PosXYT& obj1, const PosXYT& obj2 ) const {
        return obj1.t < obj2.t;
    }

    bool operator() ( const std::pair<PosXYT,unsigned long>& obj1, const std::pair<PosXYT,unsigned long>& obj2 ) const {
        return obj1.first.t < obj2.first.t;
    }
};

/**
 * PosXYT型の比較のための関数オブジェクト
 */
class PosXYT_T_Equal : public std::binary_function< PosXYT, PosXYT, bool > {
public:
    bool operator() ( const PosXYT& obj1, const PosXYT& obj2 ) const {
        return obj1.t == obj2.t;
    }

    bool operator() ( const std::pair<PosXYT,unsigned long>& obj1, const std::pair<PosXYT,unsigned long>& obj2 ) const {
        return obj1.first.t == obj2.first.t;
    }
};

/**
 * PosXYT型の比較のための関数オブジェクト
 */
class PosXYT_XY_Less : public std::binary_function< PosXYT, PosXYT, bool > {
public:
    bool operator() ( const PosXYT& obj1, const PosXYT& obj2 ) const {
        return ( obj1.x != obj2.x ) ? obj1.x < obj2.x : obj1.y < obj2.y;
    }
};

/**
 * PosXYT型の比較のための関数オブジェクト
 */
class PosXYT_XY_Equal : public std::binary_function< PosXYT, PosXYT, bool > {
public:
    bool operator() ( const PosXYT& obj1, const PosXYT& obj2 ) const {
        return ( obj1.x == obj2.x && obj1.y == obj2.y );
    }
};

/**
 * PosXYT型の比較のための関数オブジェクト
 */
class PosXYT_XYT_Less : public std::binary_function< PosXYT, PosXYT, bool > {
public:
    bool operator() ( const PosXYT& obj1, const PosXYT& obj2 ) const {
        return ( obj1.t != obj2.t ) ? obj1.t < obj2.t
                                     : ( ( obj1.x != obj2.x ) ? obj1.x < obj2.x : obj1.y < obj2.y );
    }
};

/**
 * PosXYT型の比較のための関数オブジェクト
 */
class PosXYT_XYT_Equal : public std::binary_function< PosXYT, PosXYT, bool > {
public:
    bool operator() ( const PosXYT& obj1, const PosXYT& obj2 ) const {
        return ( obj1.t == obj2.t && obj1.x == obj2.x && obj1.y == obj2.y );
    }
};

/**
 * PosXYTV型の和算のための関数オブジェクト
 */
class PosXYTV_Sum {
public:
    double operator() ( double c, const std::pair<PosXYTV,unsigned long>& obj ) {
        return c + obj.first.value;
    }
};

/**
 * ２つのPosXYT型の距離を定義する関数オブジェクト
 */
class PosXYT_Dist {
public:
    double operator()( PosXYT& obj1, PosXYT& obj2 ) {
        double dx, dy;
        dx = obj1.x - obj2.x;
        dy = obj1.y - obj2.y;
        return sqrt( dx * dx + dy * dy );
    }
};

/**
 * PosXYT型の比較のための関数オブジェクト
 */
class PosXYTID_TID_Less : public std::binary_function< PosXYTID, PosXYTID, bool > {
public:
    bool operator() ( const PosXYTID& obj1, const PosXYTID& obj2 ) const {
        return ( obj1.t != obj2.t ) ? obj1.t < obj2.t
                                     : obj1.ID < obj2.ID;
    }
};

#endif /* _POSXYT_H */
