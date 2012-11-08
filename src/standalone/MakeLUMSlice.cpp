/**
 * @file MakeLUMSlice.cpp
 * @brief LUMスライス作成に関する実装
 *
 * @author 福司 謙一郎
 * @date 2009
 */

#include <vector>
#include "track.h"

using namespace std;

int MakeLUMSlice( TIME_MICRO_SEC tk, LUMStorage* pStorageLUM, vector<LUMSlice>* pDst, const PARAM_EXTRACTLUM* pExtractlumParam )
{
    //
    // 時刻tkのLUMスライスを作成するために関係するpStorageLUM内のLUMは[tk, tk + pExtractlumParam->term]
    // なのでその範囲のLUMのみを用いる
    //
    TIME_MICRO_SEC timeBegin, timeEnd;
    LUMStorage::iterator it, itEnd;

    it = pStorageLUM->lower_bound( tk );
    itEnd = pStorageLUM->upper_bound( tk + pExtractlumParam->term );

    for( ; it != itEnd; ++it ) {
        vector<CLinearUniformMotion>::iterator itLUM = it->second.begin();
        for( ; itLUM != it->second.end(); ++itLUM ) {
            LUMSlice sliceLUM;
            if( itLUM->GetLUMSlice( tk, &sliceLUM ) ) {
                pDst->push_back( sliceLUM );
            }
        }
    }

    return TRUE;
}