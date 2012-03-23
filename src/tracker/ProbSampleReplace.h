/**
 * @file CProbSampleReplace.h
 * @brief �������o�N���X
 *
 * @author ���i ����Y
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
     * �T���v�����O
     * @param pDst �T���v���̊i�[��
     * @param nSamples �T���v����������ʂ̌�
     * @return ���ۂɃT���v�����O�ł��������ʂ̌�
     */
    int Execute( DataType* pDst, int nSamples ) {
        int* ans = new int[ nSamples ];
        double* p = new double[ size() ];
        unsigned long Z = 0;
        size_t i;
        vector< pair< DataType, unsigned long > >::iterator it;

        if( nSamples < 0 )
            return 0;

        // ���K���W���i�e�f�[�^�̏d�݂̘a�j
        for( it = begin(); it != end(); it++ ) {
            Z += it->second;
        }

        // �e�f�[�^�̊m���i�d�݁^���K���W���j�̌v�Z
        for( it = begin(), i = 0; it != end(); it++, i++ ) {
            p[ i ] = (double)( it->second ) / (double)Z;
        }

        // �T���v�����O�̎��s
        ProbSampleReplace( (int)size(), p, NULL, nSamples, ans );

        // ���ʂ̕ۑ�
        for( i = 0; i < (size_t)nSamples; i++ ) {
            pDst[ i ] = at( ans[ i ] - 1 ).first;
        }

        delete [] ans;
        delete [] p;

        return nSamples;
    }

    /**
     * Walker's alias method �ɂ�镜�����o
     * 
     * �\�[�X�R�[�h�͉��L��URL����R�s�[�B���C�Z���X�ɂ͒��ӁB
     * http://tolstoy.newcastle.edu.au/R/devel/05/06/1403.html<br>
     * ���̑��Q�l�T�C�g<br>
     * http://d.hatena.ne.jp/higotakayuki2/20070826/p1
     * @param n �T���v�����f�[�^�̐�
     * @param p �T���v�����f�[�^�̊m��
     * @param perm ���g�p(NULL�ɂ��Ă���)
     * @param nans ���o����T���v����
     * @param ans ���o�����T���v���̃C���f�b�N�X�̊i�[��
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