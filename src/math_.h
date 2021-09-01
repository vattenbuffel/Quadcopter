#if !defined(MATH_H_)
#define MATH_H_

#include <math.h>

#define HZ_TO_S(hz)(1.f/(hz))
#define S_TO_HZ(s)(1.f/(s))

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))


#define degToRad(angleInDegrees) ((angleInDegrees)*PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians)*180.0 / PI)

#define G 9.81
#define MS_TO_S(ms)((ms)/1000.f)
#define S_TO_MS(s)(1000.f * (s))

#define MS_TO_TICKS(ms) ((ms)/portTICK_PERIOD_MS)
#define S_TO_TICKS(s) MS_TO_TICKS(S_TO_MS((s)))
#define HZ_TO_TICKS(hz) (S_TO_TICKS(HZ_TO_S((hz))))

#endif // MATH_H_
