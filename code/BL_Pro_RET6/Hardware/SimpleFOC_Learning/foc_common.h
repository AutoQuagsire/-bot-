#ifndef FOC_COMMON_H
#define FOC_COMMON_H

/* Control-loop timing */
#ifndef FOC_FREQUENCY
#define FOC_FREQUENCY 10000.0f
#endif

#ifndef FOC_PERIOD_S
#define FOC_PERIOD_S (1.0f / FOC_FREQUENCY)
#endif

#ifndef FOC_PERIOD_US
#define FOC_PERIOD_US 100U
#endif

/* Math / modulation constants */
#ifndef PI
#define PI 3.14159265359f
#endif

#ifndef V_SUPPLY
#define V_SUPPLY 19.0f
#endif

#ifndef Uq_max
#define Uq_max (V_SUPPLY * 0.577f)
#endif

#ifndef _SQRT3
#define _SQRT3 1.73205080757f
#endif

#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

#endif /* FOC_COMMON_H */
