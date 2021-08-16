#if !defined(MATH_H_)
#define MATH_H_

#include <math.h>

#define HZ_TO_SEC(hz)(1.f/(hz))
#define SEC_TO_HZ(s)(1.f/(s))

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))


#define degToRad(angleInDegrees) ((angleInDegrees)*PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians)*180.0 / PI)

#define G 9.81
#define MILLIES_TO_S(ms)((ms)/1000.f)



#endif // MATH_H_
