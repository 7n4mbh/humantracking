#ifndef __HEADER_HUMANTRACKING__
#define __HEADER_HUMANTRACKING__

#if defined(WINDOWS) || defined(_WIN32)
#define WINDOWS_OS
#elif defined(linux) || defined(__linux__)
#define LINUX_OS
#endif

#include <string>

#ifdef WINDOWS_OS
#include <windows.h>
#endif
#ifdef LINUX_OS
#include <sys/time.h>
#endif

#define DELTA_EPOCH_IN_MICROSECS  11644473600000000ULL

#define SIZE_BUFFER ( 10000 )

typedef struct {
    unsigned int serialNumber;
    unsigned long long timeStamp;
    std::string data;
} PEPMapInfo;

inline unsigned long long getTimeStamp()
{
#ifdef WINDOWS_OS
    FILETIME ft;
    unsigned __int64 tmpres = 0;
    time_t _sec, _usec;
    GetSystemTimeAsFileTime( &ft );
    tmpres |= ft.dwHighDateTime;
    tmpres <<= 32;
    tmpres |= ft.dwLowDateTime;
    tmpres /= 10;
    tmpres -= DELTA_EPOCH_IN_MICROSECS; 

    return (unsigned long long)tmpres;
#endif
#ifdef LINUX_OS
    timeval tv;
    gettimeofday( &tv, NULL );
    return (unsigned long long)tv.tv_sec * 1000000ULL + (unsigned long long)tv.tv_usec;
#endif
}

#endif