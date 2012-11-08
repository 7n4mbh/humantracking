/**
 * @file pttype.h
 * @brief 基本的なデータ型の定義
 *
 * @author 福司 謙一郎
 * @date 2009
 */

#ifndef _PTTYPE_H_
#define _PTTYPE_H_

#ifndef WIN32

#include <stdint.h>
typedef uint64_t ULONGLONG;
typedef uint32_t DWORD;
typedef uint8_t BYTE;
typedef uint64_t TIME_MICRO_SEC;
#define TRUE 1
#define FALSE 0

#else

#include <windows.h>
typedef ULONGLONG TIME_MICRO_SEC;

#endif


#endif /* _PTTYPE_H_ */