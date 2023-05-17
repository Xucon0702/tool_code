#ifndef MV_LOG_H
#define MV_LOG_H

#ifndef GAC_LOG_DEBUG
#define GAC_LOG_DEBUG(...)  printf(__VA_ARGS__)
#endif

#ifndef GAC_LOG_INFO
#define GAC_LOG_INFO(...)  printf(__VA_ARGS__)
#endif

#endif