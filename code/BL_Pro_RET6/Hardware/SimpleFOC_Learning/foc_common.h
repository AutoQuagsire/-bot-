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
#define V_SUPPLY 19.5f
#endif

#ifndef Uq_max
#define Uq_max (V_SUPPLY * 0.577f)
#endif

/* Bus-voltage measurement / compensation switches.
 *
 * APP_BUS_VOLTAGE_ENABLE:
 *   1 = enable ADC3 VBUS sampling, startup validity check, and debug values
 *   0 = bypass VBUS sampling and use fixed V_SUPPLY as the bus voltage
 *
 * APP_BUS_VOLTAGE_FOC_ENABLE:
 *   1 = use measured/filtered VBUS for FOC PWM modulation
 *   0 = keep FOC PWM modulation on fixed V_SUPPLY
 */
#ifndef APP_BUS_VOLTAGE_ENABLE
#define APP_BUS_VOLTAGE_ENABLE 1U
#endif

#ifndef APP_BUS_VOLTAGE_FOC_ENABLE
#define APP_BUS_VOLTAGE_FOC_ENABLE APP_BUS_VOLTAGE_ENABLE
#endif

#ifndef _SQRT3
#define _SQRT3 1.73205080757f
#endif

#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

#endif /* FOC_COMMON_H */
