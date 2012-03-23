/**
 * @file CProbSampleReplace.h
 * @brief 復元抽出クラス
 *
 * @author 福司 謙一郎
 * @date 2008
 */

#ifndef _PROBSAMPLEREPLACE_H
#define _PROBSAMPLEREPLACE_H

#include <math.h>
#include <vector>
#include <algorithm>

using namespace std;

namespace Vier {

template <class DataType>
class CProbSampleReplace : public vector< pair< DataType, unsigned long > > {
public:
    /**
     * サンプリング
     * @param pDst サンプルの格納先
     * @param nSamples サンプルする特徴量の個数
     * @return 実際にサンプリングできた特徴量の個数
     */
    int Execute( DataType* pDst, int nSamples ) {
        int* ans = new int[ nSamples ];
        double* p = new double[ size() ];
        unsigned long Z = 0;
        size_t i;
        vector< pair< DataType, unsigned long > >::iterator it;

        if( nSamples < 0 )
            return 0;

        // 正規化係数（各データの重みの和）
        for( it = begin(); it != end(); it++ ) {
            Z += it->second;
        }

        // 各データの確率（重み／正規化係数）の計算
        for( it = begin(), i = 0; it != end(); it++, i++ ) {
            p[ i ] = (double)( it->second ) / (double)Z;
        }

        // サンプリングの実行
        ProbSampleReplace( (int)size(), p, NULL, nSamples, ans );

        // 結果の保存
        for( i = 0; i < (size_t)nSamples; i++ ) {
            pDst[ i ] = at( ans[ i ] - 1 ).first;
        }

        delete [] ans;
        delete [] p;

        return nSamples;
    }

    /**
     * Walker's alias method による復元抽出
     * 
     * ソースコードは下記のURLからコピー。ライセンスには注意。
     * http://tolstoy.newcastle.edu.au/R/devel/05/06/1403.html<br>
     * その他参考サイト<br>
     * http://d.hatena.ne.jp/higotakayuki2/20070826/p1
     * @param n サンプル元データの数
     * @param p サンプル元データの確率
     * @param perm 未使用(NULLにしておく)
     * @param nans 抽出するサンプル数
     * @param ans 抽出したサンプルのインデックスの格納先
     */
    void ProbSampleReplace(int n, double *p, int *perm, int nans, int *ans) {

        /* allocate memory for a, p and HL */     double * q = (double*)calloc(n, sizeof(double));

        int * a = (int*)calloc(n, sizeof(int));
        int * HL =(int*) calloc(n, sizeof(int)); 
        int * H = HL;       
        int * L = HL+n-1;      


        int i, j, k;
        double rU; /* U[0,1)*n */

        /* set up alias table */
        /* initialize q with n*p0,...n*p_n-1 */     for(i=0; i<n; ++i)

            q[i] = p[i]*n;

        /* initialize a with indices */
        for(i=0; i<n; ++i)

            a[i] = i;

        /* set up H and L */
        for(i=0; i<n; ++i) {

            if( q[i] >= 1.)
                *H++ = i;
            else
                *L-- = i;


        }

        while( H != HL && L != HL+n-1) {

            j = *(L+1);
            k = *(H-1);
            a[j] = k;
            q[k] += q[j] - 1;
            L++;                                  /* remove j from L */
            if( q[k] < 1. ) {
                *L-- = k;                         /* add k to L */
                --H;                              /* remove k */
            }


        }

        /* generate sample */
        for (i = 0; i < nans; ++i) {

            //rU = 1.0/*rand()*/ * n;
            rU = (double)n * (double)rand() / ( (double)RAND_MAX + 1.0 );

            k = (int)(rU);
            rU -= k;  /* rU becomes rU-[rU] */

            if( rU < q[k] )
                ans[i] = k+1;
            else
                ans[i] = a[k]+1;


        }

        free(HL);
        free(a);
        free(q); 
    } 
};

}

#endif