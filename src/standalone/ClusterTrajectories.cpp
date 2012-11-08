/**
 * @file ClusterTrajectories.cpp
 * @brief 軌跡のクラスタリングに関する実装
 *
 * @author 福司 謙一郎
 * @date 2009
 */

//#include <mpi.h>
#include <vector>
#include <map>
#include <deque>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <ipp.h>
#include "track.h"
//#include "MersenneRandomizer.h"

using namespace std;
//using namespace nyx::util::random;

typedef map< long, vector<int> > HashTable; // ハッシュテーブル（ハッシュ値と軌跡番号の対応を保持）
typedef map< TIME_MICRO_SEC, HashTable > TimeToHashTable; // 時刻[usec]とハッシュテーブルの対応を保持
typedef vector<long> HashMap; // ハッシュマップ（軌跡番号とハッシュ値の対応を保持）
typedef map< TIME_MICRO_SEC, HashMap > TimeToHashMap; // 時刻[usec]とハッシュマップの対応を保持

//inline long _hash( double x, double y, double w )
//{
//    long m, n, k;
//    m = (long)floor( x / w / 2.0 );
//    n = (long)floor( y / w / 2.0 );
//    k = max( ( abs( 2 * m + 1 ) + 1 ) / 2
//           , ( abs( 2 * n + 1 ) + 1 ) / 2 );
//
//    long ret;
//    if( n == -k ) {
//        ret = 4 * k * k -     k + m + 1;
//    } else if( m == -k ) {
//        ret = 4 * k * k - 3 * k - n + 1;
//    } else if( n == k - 1 ) {
//        ret = 4 * k * k - 5 * k - m + 2;
//    } else {
//        ret = 4 * k * k - 7 * k + n + 4;
//    }
//
//    return ret;
//}
//
//#define hash1( x, y, w ) ( _hash( ( x )        , ( y )        , ( w ) ) )
//#define hash2( x, y, w ) ( _hash( ( x ) - ( w ), ( y )        , ( w ) ) )
//#define hash3( x, y, w ) ( _hash( ( x )        , ( y ) - ( w ), ( w ) ) )
//#define hash4( x, y, w ) ( _hash( ( x ) - ( w ), ( y ) - ( w ), ( w ) ) )
/*
void PrepareDistanceTable( double* distTable, size_t sizeTable, map<int,CTrajectory>::iterator itStart, map<int,CTrajectory>::iterator itEnd, map<int,CTrajectory>* pTrajectories, double distanceLimit )
{
    vector<int> indexTrj;
    int maxIndexTrj; // pTrajectoriesに含まれる軌跡の番号の最大値

    int myRank; // プロセス番号
    double start_d, end_d, start_d2, end_d2;


    MPI_Comm_rank( MPI_COMM_WORLD, &myRank );

    if( myRank == 0 ) {
        cout << "ハッシュテーブル作成開始...";
    }
    MPI_Barrier( MPI_COMM_WORLD );
    start_d = MPI_Wtime();

    //
    // ハッシュテーブルの作成
    TimeToHashTable tableHash1, tableHash2, tableHash3, tableHash4;
    TimeToHashMap mapHash1, mapHash2, mapHash3, mapHash4;
    indexTrj.reserve( pTrajectories->size() );
    map<int,CTrajectory>::iterator it1;
    for( it1 = pTrajectories->begin(); it1 != pTrajectories->end(); ++it1 ) {
        indexTrj.push_back( it1->first );
    }
    maxIndexTrj = *indexTrj.rbegin();

    for( it1 = pTrajectories->begin(); it1 != pTrajectories->end(); ++it1 ) {
        int iTrj = it1->first; // 軌跡番号
        TrajectoryElement::iterator itPos = it1->second.at( 0 ).begin();
        for( ; itPos != it1->second.at( 0 ).end(); ++itPos ) {
            long hash;

            if( mapHash1[ itPos->t ].size() < maxIndexTrj + 1 ) {
                mapHash1[ itPos->t ].resize( maxIndexTrj + 1 );
                mapHash2[ itPos->t ].resize( maxIndexTrj + 1 );
                mapHash3[ itPos->t ].resize( maxIndexTrj + 1 );
                mapHash4[ itPos->t ].resize( maxIndexTrj + 1 );
            }

            hash = hash1( itPos->x, itPos->y, distanceLimit );
            tableHash1[ itPos->t ][ hash ].push_back( iTrj );
            mapHash1[ itPos->t ][ iTrj ] = hash;

            hash = hash2( itPos->x, itPos->y, distanceLimit );
            tableHash2[ itPos->t ][ hash ].push_back( iTrj );
            mapHash2[ itPos->t ][ iTrj ] = hash;

            hash = hash3( itPos->x, itPos->y, distanceLimit );
            tableHash3[ itPos->t ][ hash ].push_back( iTrj );
            mapHash3[ itPos->t ][ iTrj ] = hash;

            hash = hash4( itPos->x, itPos->y, distanceLimit );
            tableHash4[ itPos->t ][ hash ].push_back( iTrj );
            mapHash4[ itPos->t ][ iTrj ] = hash;
        }
    }

    MPI_Barrier( MPI_COMM_WORLD );
    end_d = MPI_Wtime();
    if( myRank == 0 ) {
        cout << "終了:";
        cout << ( end_d - start_d ) * 1.0e3 << "[msec]" << endl;
    }

    if( myRank == 0 ) {
        cout << "距離行列準備開始（自プロセスの軌跡:" << distance( itStart, itEnd ) << "[個], "
             << "距離評価軌跡数:" << pTrajectories->size() << "[個]）...";
    }
    MPI_Barrier( MPI_COMM_WORLD );
    start_d = MPI_Wtime();

    int cnt = 0;

    //
    // 距離行列の準備
    for( it1 = itStart; it1 != itEnd; ++it1 ) {
        // 軌跡it1とpTrajectory中の任意の軌跡について，２軌跡が定義される共通の時刻でハッシュ値
        // が１度でも異なる場合，距離を-1.0（無限大）とする。
        int iTrj1 = it1->first; // it1の軌跡番号

//        start_d2 = MPI_Wtime();

        // hashTable1〜hashTable4について，軌跡it1と同じハッシュ値を持つ軌跡を取り出す
        vector<int> nNearPoint( *indexTrj.rbegin() + 1, 0 );
        vector<int> nNearPointSub;
        vector<int> indexNearTrajectories;
        indexNearTrajectories.reserve( pTrajectories->size() * 4 );
        TrajectoryElement::iterator itPos = it1->second.at( 0 ).begin();
        for( ; itPos != it1->second.at( 0 ).end(); ++itPos ) {
            indexNearTrajectories.clear();
            nNearPointSub.assign( nNearPoint.size(), 0 );

            long hash;
            //hash = hash1( itPos->x, itPos->y, distanceLimit );
            hash = mapHash1[ itPos->t ][ iTrj1 ];
            indexNearTrajectories.insert( indexNearTrajectories.end()
                                        , tableHash1[ itPos->t ][ hash ].begin()
                                        , tableHash1[ itPos->t ][ hash ].end() );
            //hash = hash2( itPos->x, itPos->y, distanceLimit );
            hash = mapHash2[ itPos->t ][ iTrj1 ];
            indexNearTrajectories.insert( indexNearTrajectories.end()
                                        , tableHash2[ itPos->t ][ hash ].begin()
                                        , tableHash2[ itPos->t ][ hash ].end() );
            //hash = hash3( itPos->x, itPos->y, distanceLimit );
            hash = mapHash3[ itPos->t ][ iTrj1 ];
            indexNearTrajectories.insert( indexNearTrajectories.end()
                                        , tableHash3[ itPos->t ][ hash ].begin()
                                        , tableHash3[ itPos->t ][ hash ].end() );
            //hash = hash4( itPos->x, itPos->y, distanceLimit );
            hash = mapHash4[ itPos->t ][ iTrj1 ];
            indexNearTrajectories.insert( indexNearTrajectories.end()
                                        , tableHash4[ itPos->t ][ hash ].begin()
                                        , tableHash4[ itPos->t ][ hash ].end() );

            for( vector<int>::iterator itIdx = indexNearTrajectories.begin(); itIdx != indexNearTrajectories.end(); ++itIdx ) {
                nNearPointSub[ *itIdx ] = 1;
            }

            for( size_t i = 0; i < nNearPoint.size(); ++i ) {
                if( nNearPointSub[ i ] )
                    nNearPoint[ i ]++;
            }
        }

//        end_d2 = MPI_Wtime();
//        if( myRank == 0 ) {
//            cout << "（追加期間:";
//            cout << ( end_d2 - start_d2 ) * 1.0e3 << "[msec]";
//        }
//        start_d2 = MPI_Wtime();



        //for( vector<int>::iterator itIdx = indexTrj.begin(); itIdx != indexTrj.end(); ++itIdx ) {
        map<int,CTrajectory>::iterator it2;
        for( it2 = pTrajectories->begin(); it2 != pTrajectories->end(); ++it2 ) {
            int iTrj2 = it2->first;//*itIdx;
            int index1 = iTrj1 * sizeTable + iTrj2;
            int index2 = iTrj2 * sizeTable + iTrj1;
            if( distTable[ index1 ] < -1.0 || distTable[ index2 ] < -1.0 ) {
                TIME_MICRO_SEC timeBegin = max( it1->second.at( 0 ).begin()->t, it2->second.at( 0 ).begin()->t );
                TIME_MICRO_SEC timeEnd = min( it1->second.at( 0 ).rbegin()->t, it2->second.at( 0 ).rbegin()->t );

                // 軌跡it1と軌跡it2が共通に定義される区間での点の数nnpを求める
                int nnp = distance( it1->second.at( 0 ).lower_bound( PosXYT( 0, 0, timeBegin ) )
                                   , it1->second.at( 0 ).upper_bound( PosXYT( 0, 0, timeEnd ) ) );
                nnp++;
//                if( myRank == 0 ){
//                    cout << "nNearPoint=" << nNearPoint[ iTrj2 ] << ", nnp=" << nnp << " ";
//                }

                // 共通に定義される区間内で異なるハッシュ値を１回でも持つものは距離を無限大にする
                if( nNearPoint[ iTrj2 ] < max( 20, nnp - 5 ) ) {
                    distTable[ index1 ] = distTable[ index2 ] = -1.0;
                    cnt++;
                }
            }
        }

//        end_d2 = MPI_Wtime();
//        if( myRank == 0 ) {
//            cout << ", 行列更新期間:";
//            cout << ( end_d2 - start_d2 ) * 1.0e3 << "[msec]）";
//        }
    }

    MPI_Barrier( MPI_COMM_WORLD );
    end_d = MPI_Wtime();
    if( myRank == 0 ) {
        cout << "終了:";
        cout << ( end_d - start_d ) * 1.0e3 << "[msec]";
        cout << ", " << cnt << "/" << distance( itStart, itEnd ) * pTrajectories->size() << "[個]の距離評価を除外可能" << endl;
    }
}
*/
void CalculateDistanceTable( double* distTable, vector<size_t>* pIdxTrjToCol, size_t stepDistTable, map<int,CTrajectory>::iterator itStart, map<int,CTrajectory>::iterator itEnd, map<int,CTrajectory>* pTrajectories, CTrajectory_Distance& funcDist )
{
//    int myRank; // プロセス番号
//
//    MPI_Comm_rank( MPI_COMM_WORLD, &myRank );
//
//    if( myRank == 0 ) {
//        cout << "距離行列作成開始...自プロセスの軌跡:" << distance( itStart, itEnd )
//             << "[個], 評価軌跡数:" << pTrajectories->size() << "[個]...";
//    }
//
//    if( distance( itStart, itEnd ) == pTrajectories->size() ) {
//        cout << endl << endl << "!!" << endl << endl;
//    }

    //int nMyTrj = distance( itStart, itEnd );

    map<int,CTrajectory>::iterator it1;
    map<int,CTrajectory>::iterator it2;
    int row, col;
    for( it1 = pTrajectories->begin(); it1 != pTrajectories->end(); ++it1 ) {
        row = 0;
        for( it2 = itStart; it2 != itEnd; ++it2, ++row ) {
            col = (*pIdxTrjToCol)[ it1->first ];
            int index1 = row * stepDistTable + col;
            if( distTable[ index1 ] < -1.0 ) {
                // ( it1->first != it2->first )とした方が遅くなってしまった
                //distTable[ index1 ] = ( it1->first != it2->first ) ? funcDist( it1->second, it2->second ) : 0.0;
                distTable[ index1 ] = funcDist( it1->second, it2->second );
//                if( it1->first >= itStart->first && it1->first < ( itStart->first + nMyTrj ) ) {
//                    int row2 = it1->first - itStart->first;
//                    int col2 = (*pIdxTrjToCol)[ it2->first ];
//                    int index2 = row2 * stepDistTable + col2;
////                    if( myRank == 0 ) {
////                        cout << endl << "index2=" << index2 << endl;
////                    }
//                    distTable[ index2 ] = distTable[ index1 ];
//                }
            }
        }

//        distTable[ it1->first * sizeTable + it1->first ] = 0.0;
//        map<int,CTrajectory>::iterator it2;
//        for( it2 = itStart; it2 != itEnd; ++it2 ) {
//            int index1 = it1->first * sizeTable + it2->first;
//            int index2 = it2->first * sizeTable + it1->first;
//            if( distTable[ index1 ] < -1.0 || distTable[ index2 ] < -1.0 ) {
//                distTable[ index1 ] = distTable[ index2 ] = funcDist( it1->second, it2->second );
//            }
//        }
    }

//    if( myRank == 0 ) {
//        cout << "終了" << endl;
//    }
}

void _clustering( size_t i, vector<int>* pIndexCluster, int label, double threshold, size_t nElements, double* distTable )
{
    if( pIndexCluster->at( i ) == label )
        return;

    pIndexCluster->at( i ) = label;
    for( size_t j = 0; j < nElements; ++j ) {
        if( i != j ) {
            size_t index = i * nElements + j;
            if( distTable[ index ] >= 0.0 && distTable[ index ] <= threshold ) {
                _clustering( j, pIndexCluster, label, threshold, nElements, distTable );
            }
        }
    }
}

int Clustering( vector<int>* pIndexCluster, double* distTable, size_t nElements, double threshold )
{
    int label = 0;
    for( size_t i = 0; i < nElements; ++i ) {
        if( pIndexCluster->at( i ) == -1 ) {
            _clustering( i, pIndexCluster, label, threshold, nElements, distTable );
            ++label;
        }
    }

    return label;
}

void _clustering2( int index, vector< vector<int> >* pDst, vector<int> cluster, double thConnect, double thDiameter, size_t nElements, std::vector<int>& classID, double* distTable )
{
    cluster.push_back( index );
    sort( cluster.begin(), cluster.end() );

    // 既にcluster内の軌跡を全て含むクラスタがある場合はこれ以上探索しない。
    for( vector< vector<int> >::iterator it = pDst->begin(); it != pDst->end(); ++it ) {
        if( includes( it->begin(), it->end(), cluster.begin(), cluster.end() ) && ( cluster != *it ) ) {
            return;
        }
    }

    bool flgTerminal = true;

    // 他の軌跡のclusterへの追加を試みる
    for( int idxTrj = 0; idxTrj < (int)nElements; ++idxTrj ) {
        //cerr << "classID[ index ] = " << classID[ index ] << ", classID[ idxTrj ] = " << classID[ idxTrj ] << endl;
        if( ( idxTrj != index ) && ( classID[ idxTrj ] == classID[ index ] ) ) {
            // cluster中にidxTrj番目の軌跡との距離がthreshold以下の軌跡が有ればclusterへ追加する。
            // ただし，cluster中にidxTrj番目の軌跡との距離がthreshold以上の軌跡が1つでも有ればclusterへの追加はできない。
            // また，既にcluster中にidxTrj番目の軌跡が有る場合も追加しない。
            //size_t idx = index * nElements + idxTrj;
            //bool flgCluster = ( distTable[ idx ] >= 0.0 && distTable[ idx ] <= threshold );
            bool flgCluster = false;
            for( vector<int>::iterator it = cluster.begin(); it != cluster.end(); ++it ) {
                if( *it == idxTrj ) {
                    flgCluster = false;
                    break;
                }

                size_t idx = *it * nElements + idxTrj;
                if( ( distTable[ idx ] >= 0.0 ) && ( distTable[ idx ] <= thConnect ) ) {
                    flgCluster = true;
                }
                if( ( distTable[ idx ] < -0.5 ) || ( distTable[ idx ] > thDiameter ) ) {
                    flgCluster = false;
                    break;
                }
            }

            if( flgCluster ) {
                _clustering2( idxTrj, pDst, cluster, thConnect, thDiameter, nElements, classID, distTable );
                flgTerminal = false;
            }
        }
    }

    if( flgTerminal ) {
//        for( vector< vector<int> >::iterator it = pDst->begin(); it != pDst->end(); ) {
//            if( includes( it->begin(), it->end(), cluster.begin(), cluster.end() ) ) {
//                return;
//            } else  if( includes( cluster.begin(), cluster.end(), it->begin(), it->end() ) ) {
//               it = pDst->erase( it );
//            } else {
//                ++it;
//            }
//        }
        pDst->push_back( cluster );
//        cout << "クラスタ" << pDst->size() << "（軌跡" << cluster.size() << "[個]）...";
    }
}

int Clustering2( vector< vector<int> >* pDst, vector<int>& classID, double* distTable, size_t nElements, double thConnect, double thDiameter )
{
    pDst->clear();

    for( int idxTrj = 0; idxTrj < (int)nElements; ++idxTrj ) {
        vector<int> cluster;
        _clustering2( idxTrj, pDst, cluster, thConnect, thDiameter, nElements, classID, distTable );
    }

    return (int)pDst->size();
}

bool VerifyClusteredTrajectories( CTrajectory& trj, double threshold )
{
    CTrajectory trjMean;
    trj.Integrate( &trjMean );

    deque<PosXYT> pos;

    // 軌跡trjを平均化した軌跡trjMeanの各時刻で，trjに含まれる軌跡との最小距離を求める。
    for( TrajectoryElement::iterator itMeanPos = trjMean.front().begin(); itMeanPos != trjMean.front().end(); ++itMeanPos ) {

//        pos.push_back( *itMeanPos );
//        if( pos.size() > 3 ) {
//            pos.erase( pos.begin() );
//        }
//
//        // 速度変化が急激な場合は異常とみなす
//        if( pos.size() == 3 ) {
//            double vx1, vy1, vx2, vy2, dvx, dvy;
//            vx1 = ( pos.at( 2 ).x - pos.at( 1 ).x ) / ( (double)( pos.at( 2 ).t - pos.at( 1 ).t ) * 1.0e-6 );
//            vy1 = ( pos.at( 2 ).y - pos.at( 1 ).y ) / ( (double)( pos.at( 2 ).t - pos.at( 1 ).t ) * 1.0e-6 );
//            vx2 = ( pos.at( 1 ).x - pos.at( 0 ).x ) / ( (double)( pos.at( 1 ).t - pos.at( 0 ).t ) * 1.0e-6 );
//            vy2 = ( pos.at( 1 ).y - pos.at( 0 ).y ) / ( (double)( pos.at( 1 ).t - pos.at( 0 ).t ) * 1.0e-6 );
//            dvx = vx1 - vx2;
//            dvy = vy1 - vy2;
//            if( sqrt( dvx * dvx + dvy * dvy ) > 5.0 ) {
//                cerr << "■違反クラスタ 速度変化" << sqrt( dvx * dvx + dvy * dvy ) << endl;
//                return false;
//            }
//        }

        vector<Ipp32f> distance;
        for( CTrajectory::iterator itTrjElm = trj.begin(); itTrjElm != trj.end(); ++itTrjElm ) {
            TrajectoryElement::iterator itPos;
            if( ( itPos = itTrjElm->find( PosXYT( 0.0, 0.0, itMeanPos->t ) ) ) != itTrjElm->end() ) {
                double dx, dy;
                dx = itPos->x - itMeanPos->x;
                dy = itPos->y - itMeanPos->y;
                distance.push_back( sqrt( dx * dx + dy * dy ) );
            }
        }
        Ipp32f minDist;
        ippsMin_32f( (Ipp32f*)&(distance[ 0 ]), (int)distance.size(), &minDist );
        // 最小距離がthresholdより大きいときは異常とみなす
        if( minDist > threshold ) {
            //cout << "■違反クラスタ 距離" << minDist << endl;
            return false;
        }
    }

    // 異常がなければ妥当と判断して帰る
    return true;
}

void CalcFrequency( double* pDst, double* distTable, int size, double sigma, double thDist ) {
    for( int idx1 = 0; idx1 < size; ++idx1 ) {
        int idx2 = idx1 * size;
        for( int i = 0; i < size; ++i, ++idx2 ) {
            double d = distTable[ idx2 ];
            if( d >= 0.0 && d <= thDist ) {
                //gnuplot> f(x)=exp(-(x**2)/s1/s1/2.0)/sqrt(2*pi)/s1
                //gnuplot> s1=0.2
                pDst[ idx1 ] += exp( -d * d / sigma / sigma / 2.0 ) / sqrt( 2.0 * M_PI ) / sigma;
            }
        }
    }
}

void ReduceTrajectory( vector<int>* pDst, double* frequency, int size, double fval )
{
    //MersenneRandomizer mr( 0, RAND_MAX );

    pDst->clear();
    pDst->reserve( size );
    for( size_t idx = 0; idx < size; ++idx ) {
        const double f = frequency[ idx ];
        //const double fth = 30.0;//45.0;//40.0;//30.0;
        if( f < fval ) {
            pDst->push_back( idx );
        } else {
            const double rand_0to100 = (double)( ( rand()/*mr.getNumber()*/ % 10000 ) ) / 100.0;
            const double scl = fval / f;
            if( rand_0to100 < 100.0 * scl) {
                pDst->push_back( idx );
            }
        }
    }
}

void _clustering3( int index, vector< vector<int> >* pDst, vector<int> cluster, double thConnect, double thDiameter, size_t nElements, double* distTable, std::vector<bool>& flgClustered )
{
    flgClustered[ index ] = true;
    cluster.push_back( index );
    sort( cluster.begin(), cluster.end() );

    // 既にcluster内の軌跡を全て含むクラスタがある場合はこれ以上探索しない。
    for( vector< vector<int> >::iterator it = pDst->begin(); it != pDst->end(); ++it ) {
        if( includes( it->begin(), it->end(), cluster.begin(), cluster.end() ) && ( cluster != *it ) ) {
            return;
        }
    }

    bool flgTerminal = true;

    // 他の軌跡のclusterへの追加を試みる
    for( int idxTrj = 0; idxTrj < (int)nElements; ++idxTrj ) { // idxTrj:追加を試みる軌跡の番号
        bool flgCluster = false;
        for( vector<int>::iterator it = cluster.begin(); it != cluster.end(); ++it ) {
            // idxTrj番目の軌跡が既にclusterに含まれている場合は追加しない
            if( *it == idxTrj ) {
                flgCluster = false;
                break;
            }

            // idxTrj番目の軌跡との距離が閾値thConnect以下の軌跡が
            // clusterに含まれている場合は追加できると仮判定する。
            // ただし，このforループによる処理で，のちにidxTrj番目の
            // 軌跡との距離が閾値thConnectより大きな軌跡がclusterに
            // 含まれていた場合はこの仮判定は棄却される。
            size_t idx = *it * nElements + idxTrj;
            if( ( distTable[ idx ] >= 0.0 ) && ( distTable[ idx ] <= thConnect ) ) {
                flgCluster = true;
            }
            if( ( distTable[ idx ] < -0.5 ) || ( distTable[ idx ] > thConnect ) ) {
                flgCluster = false;
                break;
            }
        }

        if( flgCluster ) {
            _clustering3( idxTrj, pDst, cluster, thConnect, thDiameter, nElements, distTable, flgClustered );
            flgTerminal = false;
        }
    }

    if( flgTerminal ) {
        pDst->push_back( cluster );
    }
}

int Clustering3( vector< vector<int> >* pDst, double* distTable, size_t nElements, double thConnect, double thDiameter )
{
    pDst->clear();

    vector<bool> flgClustered( nElements, false );

    for( int idxTrj = 0; idxTrj < (int)nElements; ++idxTrj ) {
        if( flgClustered[ idxTrj ] == false ) {
            vector<int> cluster;
            _clustering3( idxTrj, pDst, cluster, thConnect, thDiameter, nElements, distTable, flgClustered );
        }
    }

    return (int)pDst->size();
}