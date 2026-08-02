#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "board_config.h"

// ------------- FUNCTIONS -------------

#if SC2_DEBUG

void debugInit();    // Serial + optional csv header
void debugUpdate();  // print at the rate for the current SC2_DEBUG mode
void debugError(const char* msg);

#else

inline void debugInit() {}
inline void debugUpdate() {}
inline void debugError(const char*) {}

#endif

#endif  // __DEBUG_H__
