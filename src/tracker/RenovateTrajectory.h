/**
 * @file RenovateTrajectory.h
 * @brief 軌跡修復に関する処理
 *
 * @author 福司 謙一郎
 * @date 2009
 */

#include <vector>
#include <list>
#include <set>
#include <map>
#include <functional>
#include "ipp.h"
#include "PosXYT.h"
#include "Trajectory.h"

#ifndef _RENOVATE_TRAJECTORY_H
#define _RENOVATE_TRAJECTORY_H

/**
 * 軌跡の構成要素
 */
//typedef std::set<PosXYT,PosXYT_T_Less> TrajectoryElement;

/**
 * 軌跡のグループ
 */
typedef std::vector<int> TrajectoryGroup;

/**
 * TrajectoryGroupの組み合わせ
 */
typedef std::vector<int> TrajectorySet;

/**
 * 時刻と点番号のペア
 */
typedef std::pair<TIME_MICRO_SEC,int> PointIndex;

/**
 *  PointIndex型の比較のための関数オブジェクト
 */
class PointIndex_Less : public std::binary_function< PointIndex, PointIndex, bool > {
public:
    bool operator() ( const PointIndex& obj1, const PointIndex& obj2 ) const {
        if( obj1.first != obj2.first ) {
            return obj1.first < obj2.first;
        } else {
            return obj1.second < obj2.second;
        }
    }
};

class PointIndex_Equal : public std::binary_function< PointIndex, PointIndex, bool > {
public:
    bool operator() ( const PointIndex& obj1, const PointIndex& obj2 ) const {
        return ( obj1.first == obj2.first ) && ( obj1.second == obj2.second );
    }
};

/**
 * PointIndexを用いた軌跡の表現
 */
typedef std::vector<PointIndex> TrajectoryWithPointIndex;

/**
 * 時刻から点番号のマップ
 */
typedef std::map<TIME_MICRO_SEC,int> TimeToPointIndex;

/**
 * ある時刻での点番号の候補へのマップ
 */
typedef std::multimap<TIME_MICRO_SEC,int> TimeToPointChoice;

/**
 * グループ番号から点番号へのマップのリスト
 */
typedef std::list< std::map<int,int> > ChoiceList;

/**
 * 選択肢ベクトル
 */
typedef struct {
    int nChoice; // 選択肢の個数
    vector<int> modifyGroup; // 選択肢ベクトルに含まれる選択肢が定義しているグループ
    vector<PointIndex> data; // 時刻と点番号から成るデータのベクトル
    map< int, map<int,int> > idxGrpIdxPointToOffset; // あるグループの選択肢（点番号）がdataでどの位置にあるかを示すマップ
} ChoiceVector;

inline long _hash( double x, double y, double w )
{
    long m, n, k;
    m = (long)floor( x / w / 2.0 );
    n = (long)floor( y / w / 2.0 );
    k = max( ( abs( 2 * m + 1 ) + 1 ) / 2
           , ( abs( 2 * n + 1 ) + 1 ) / 2 );

    long ret;
    if( n == -k ) {
        ret = 4 * k * k -     k + m + 1;
    } else if( m == -k ) {
        ret = 4 * k * k - 3 * k - n + 1;
    } else if( n == k - 1 ) {
        ret = 4 * k * k - 5 * k - m + 2;
    } else {
        ret = 4 * k * k - 7 * k + n + 4;
    }

    return ret;
}

#define hash1( x, y, w ) ( _hash( ( x )        , ( y )        , ( w ) ) )
#define hash2( x, y, w ) ( _hash( ( x ) - ( w ), ( y )        , ( w ) ) )
#define hash3( x, y, w ) ( _hash( ( x )        , ( y ) - ( w ), ( w ) ) )
#define hash4( x, y, w ) ( _hash( ( x ) - ( w ), ( y ) - ( w ), ( w ) ) )

/**
 * ハッシュテーブル（ハッシュ値と点番号の対応を保持）
 */
typedef std::map< long, std::vector<int> > HashTable;

/**
 * 時刻[usec]とハッシュテーブルの対応を保持
 */
typedef std::map< TIME_MICRO_SEC, HashTable > TimeToHashTable;

/**
 * bool型の配列。1要素を1ビットに割り当てている。
 */
class CStatusList : public std::vector<Ipp8u> {
public :
    void SetSize( int nElement ) {
        assign( ( nElement + 7 ) / 8, 0 );
    }

    bool get( int i ) {
        if( i < 0 ) {
            return false;
        }

        size_t idx = i / 8;
        int bit = i % 8;

        return ( ( (*this)[ idx ] & ( 0x1 << bit ) ) != 0 );
    }

    void set( int i, bool status ) {
        size_t idx = i / 8;
        int bit = i % 8;

        if( status == true ) {
            (*this)[ idx ] |= ( 0x1 << bit );
        } else {
            (*this)[ idx ] &= ~( 0x1 << bit );
        }
    }

    int count() {
        int ret = 0;
        CStatusList tmp = *this;
        for( int i = 0; i < (int)tmp.size(); ++i ) {
            while( tmp[ i ] ) {
                ++ret;
                tmp[ i ] &= ( tmp[ i ] - 1 );
            }
        }

        return ret;
    }
};

/**
 * ステータステーブル
 */
class CStatusTable : public std::vector<CStatusList> {
public :
    void SetSize( int nElement ) {
        resize( nElement );
        for( CStatusTable::iterator it = begin(); it != end(); ++it ) {
            it->SetSize( nElement );
        }
    }
};

/**
 * セクション構造体
 */
typedef struct {
    std::vector<int> idxTrajectory; ///< セクションに含まれる軌跡番号
    std::vector<TrajectoryGroup> trjGroup; ///< 軌跡のグループ一覧
    std::vector<TrajectoryWithPointIndex> groupToPoints; ///< 各グループに含まれる点
    std::vector<TrajectorySet> trjSet; ///< 軌跡の組み合わせ方の一覧
} Section;

/**
 * 軌跡情報構造体
 */
typedef struct {
    std::vector<Section> section; ///< セクション
    std::vector<TrajectoryElement> trjElement; ///< 軌跡
    std::vector<PosXYTID> points; ///< 全ての点。点のIDは軌跡番号（trjElementの添字）を表す。
    std::vector<TrajectoryWithPointIndex> trjPointIdx; ///< PointIndexを用いた軌跡の表現
    TimeToHashTable tableHash1, tableHash2, tableHash3, tableHash4; ///< ハッシュテーブル
    TimeToHashTable tableHashVeryNear1, tableHashVeryNear2, tableHashVeryNear3, tableHashVeryNear4; ///< ハッシュテーブル
    std::vector< list<int> > connectable; // 各点が接続可能な点の番号
    CStatusTable tableNear; // 近接テーブル
    CStatusTable tableVeryNear;
    CStatusTable tableGroupable, tableNotGroupable; // グループ化可能テーブル・グループ化不可能テーブル
public:
    void clear() {
        section.clear();
        trjElement.clear();
        points.clear();
        trjPointIdx.clear();
        tableHash1.clear();
        tableHash2.clear();
        tableHash3.clear();
        tableHash4.clear();
        tableHashVeryNear1.clear();
        tableHashVeryNear2.clear();
        tableHashVeryNear3.clear();
        tableHashVeryNear4.clear();
        connectable.clear();
        tableNear.clear();
        tableVeryNear.clear();
        tableGroupable.clear();
        tableNotGroupable.clear();
    }
} TrajectoriesInfo;

/**
 * 軌跡修復パラメータ構造体
 */
typedef struct {
    double lambda1, lambda2, lambda3, territory, territoryVeryNear;
    TIME_MICRO_SEC termConnect;
} PARAM_RENOVATE_TRAJECTORY;

/**
 * 最適化処理に利用する変数
 */
typedef struct {
    map<int,TimeToPointIndex> grpToStatus; // 各グループの点番号の選択状況
    vector<Ipp32f> energy; // グループごと時刻順にエネルギーを並べた配列
} OptVar;

/**
 * エネルギー計算用変数
 */
typedef struct {
    vector<PointIndex> idxNext2;
    vector<PointIndex> idxNext1;
    vector<PointIndex> idxCur;
    vector<PointIndex> idxPrev1;
    vector<PointIndex> idxPrev2;
} PARAM_CALC_ENERGY;

/**
 * エネルギー計算用領域
 */
typedef struct {
    CStatusList tmp_a;
    CStatusList tmp_b;
    CStatusList tmp_c;
    CStatusList tmp_next;
    CStatusList tmp_cur;
    CStatusList tmp_prev;
    CStatusList tmp_mask;
    vector<Ipp32f> tmp_ea;
    vector<Ipp32f> tmp_eb;
    vector<Ipp32f> tmp_ec;
    vector<Ipp32f> pos_x_next, pos_y_next;
    vector<Ipp32f> pos_x_cur, pos_y_cur;
    vector<Ipp32f> pos_x_prev, pos_y_prev;
    vector<Ipp32f> dt_next_cur, dt_cur_prev;
} BUFFER_CALC_ENERGY;

/**
 * グループの組み合わせ
 */
typedef std::vector<int> GroupCombiniation;

/**
 * セクション分割のためのクラスタリング処理
 */
void clustering( int index, std::vector<int>* pListClass, int label, CStatusTable& adjmat );

///**
// * グルーピング準備処理
// */
//void grouping_prepare( int root
//                            , int index
//                            , TrajectoryGroup grp
//                            , std::vector<TrajectoryGroup>* pStorageGrp
//                            , std::vector<CStatusList>* pStorageGrpBits
//                            , std::vector<int>& idxTrajectory
//                            , CStatusTable& tblGrp
//                            , CStatusTable& tblNGrp
//                            , map<int,int>* pReserve
//                            , bool flgContainReserveTrj = false );
//
//void grouping( int index
//                    //, vector<int> enable
//                    , CStatusList grpbit
//                    , std::vector<TrajectoryGroup>* pStorageGrp
//                    , std::vector<CStatusList>& vecGrpBits
//                    , CStatusTable& tblGrp
//                    , CStatusTable& tblNGrp );
//
//
///**
// * 組み合わせ作成処理
// */
//void makeset( int index, TrajectorySet setTrj
//              , std::vector<TrajectorySet>* pStorageSet
//              , std::vector<int> idxTrj
//              , std::vector<TrajectoryGroup>& grp
//              , std::vector<TrajectoryWithPointIndex>& trjPointIdx
//              , int& nMinGrp );

/**
 * グルーピング準備処理
 */
static void subgrouping( int root
                       , int index
                       , TrajectoryGroup grp
                       , std::vector<TrajectoryGroup>* pStorageGrp
                       , std::vector<CStatusList>* pStorageGrpBits
                       , std::vector<int>& idxTrajectory
                       , TrajectoriesInfo& infoTrj
                       , std::map<int,int>* pReserve
                       , bool flgContainReserveTrj = false );

/**
 * グルーピング処理
 */
static void grouping( int root
                    , int index
                    , vector<int> subgrps
                    , std::vector<TrajectoryGroup>* pStorageGrp
                    , std::vector<CStatusList>* pStorageGrpBits
                    , std::vector<TrajectoryGroup>& srcSubGrp
                    , CStatusTable& tblGrp
                    , CStatusTable& tblNGrp
                    , map<int,int>* pReserve
                    , bool flgContainReserveTrj = false );

///**
// * グループへの優先度割り当て処理
// */
//void assign_priority( std::map< int,vector<int> >* pDst, std::vector<TrajectoryGroup>* pStorageGrp );

/**
 *  組み合わせ作成処理
 */
//void makeset( TrajectorySet setTrj
//              , std::vector<TrajectorySet>* pStorageSet
//              , std::vector<int> idxTrj
//              , std::vector<int>& grpIdx
//              , std::vector<TrajectoryGroup>& grp );
void makeset( std::vector<TrajectorySet>* pDst
              , int idxGrpStart
              , TrajectorySet trjSet
              , CStatusList trjSetBit
              , std::set<CStatusList>& setSearched
              , std::vector< vector<GroupCombiniation> >& idxGrpToCmb
              , std::vector< vector<CStatusList> >& idxGrpToCmbBit
              , std::vector<TrajectoryGroup>& grp );

/**
 * セットの修正
 */
//void _rectifyset( std::vector<TrajectorySet>* pDst, TrajectorySet trjSet, CStatusList trjSetBit, TrajectorySet& srcSet, CStatusTable& tableGroupNear, CStatusList& grpReserved );
//void rectifyset( std::vector<TrajectorySet>* pDst, TrajectorySet& srcSet, std::vector<TrajectoryGroup>& grp, CStatusTable& tblNGrp, CStatusTable& tblNear, CStatusTable& tblVeryNear, std::map<int,int>& reserve, bool strictCheck = true );
void _rectifyset( std::vector<TrajectorySet>* pDst
                   , TrajectorySet trjSet
                   , CStatusList trjSetBit
                   , TrajectorySet& srcSet
                   , CStatusTable& tableGroupNear
                   , CStatusList& grpReserved
                   , std::vector<TrajectoryGroup>& grp
                   , CStatusTable& tblNGrp
                   , CStatusTable& tblNear
                    , CStatusTable& tblVeryNear
                   , std::map<int,int>& reserve
                   , std::vector< vector<GroupCombiniation> >& idxGrpToCmb
                   , std::vector< vector<CStatusList> >& idxGrpToCmbBit );
void rectifyset( std::vector<TrajectorySet>* pDst
                  , TrajectorySet& initSet
                  , TrajectorySet& srcSet
                  , std::vector<TrajectoryGroup>& grp
                  , CStatusTable& tblNGrp
                  , CStatusTable& tblNear
                  , CStatusTable& tblVeryNear
                  , std::map<int,int>& reserve
                  , std::vector< vector<GroupCombiniation> >& idxGrpToCmb
                  , std::vector< vector<CStatusList> >& idxGrpToCmbBit
                  , bool strictCheck = true );

/**
 * グループの包含関係を調べる
 */
void inclusive_relation( std::vector<GroupCombiniation>* pDst
                           , int idxGrpSrc
                           , GroupCombiniation grpCmb
                           , CStatusList bit
                           , std::vector<TrajectoryGroup>& grp
                           , std::vector<CStatusList>& grpBits );

/**
 * 指定した時刻での選択肢リストの作成
 */
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
                          , std::map<int,TimeToPointIndex>& grpToIdxRsvPoint
                          , double territory );

/**
 * 指定した時刻でのエネルギー計算
 */
double energy( TIME_MICRO_SEC time, map<int,TimeToPointIndex>& grpToStatus, map<int,TimeToPointChoice>& choiceOfGroup, CStatusTable& tblConnectable, TrajectoriesInfo& infoTrj, PARAM_RENOVATE_TRAJECTORY param, double* p_t1 = NULL, double* p_t2 = NULL );

/**
 * エネルギー計算準備
 */
bool next_status( PointIndex* pIdxNextPoint, TimeToPointIndex::iterator* pItNext, TimeToPointIndex::iterator itIdxPoint, TimeToPointIndex& status, CStatusTable& tblConnectable );
bool prev_status( PointIndex* pIdxPrevPoint, TimeToPointIndex::iterator* pItPrev, TimeToPointIndex::iterator itIdxPoint, TimeToPointIndex& status, CStatusTable& tblConnectable );
void energy2_init( PARAM_CALC_ENERGY* pDst, std::vector<PointIndex>& data, std::vector<int>& modifyGroup, int nChoice, std::vector<int>& listIdxStart, std::vector<OptVar>& optvar, CStatusTable& tblConnectable, vector<TIME_MICRO_SEC>& listTime );
void energy2( Ipp32f* e, std::vector<PointIndex>& next, std::vector<PointIndex>& cur, std::vector<PointIndex>& prev, int len, std::vector<int>& modifyGroup, TrajectoriesInfo& infoTrj, PARAM_RENOVATE_TRAJECTORY param, BUFFER_CALC_ENERGY* pBuffer );

#endif
