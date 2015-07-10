#define DEBUG_HELI_TIMINGS

#ifdef DEBUG_HELI_TIMINGS
#include <QtCore\qelapsedtimer.h>
#define TimerCreate() QElapsedTimer QET
#define TimerUpdate() QET.start()
#define TimerElapsed() qint64 te = QET.elapsed(); if(te>0) printf("Thread %s ran for %i ms\n", __FILE__, te)
#else
#define TimerCreate()
#define TimerUpdate()
#define TimerElapsed()
#endif