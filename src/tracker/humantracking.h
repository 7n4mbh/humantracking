#ifndef __HEADER_HUMANTRACKING__
#define __HEADER_HUMANTRACKING__

#if defined(WINDOWS) || defined(_WIN32)
#define WINDOWS_OS
#elif defined(linux) || defined(__linux__)
#define LINUX_OS
#endif

#include <string>

#define SIZE_BUFFER ( 10000 )

typedef struct {
    unsigned int serialNumber;
    unsigned long long timeStamp;
    std::string data;
} PEPMapInfo;

#endif