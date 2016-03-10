#ifndef PARAMETERS_H_
#define PARAMETERS_H_

// TIMEOUT_THRESHOLD has to be larger than 1/TICK_FREQUENCY,
// otherwise the server will start and stop all the time since
// the tick frequency is not enough to keep it alive.

// EXECUTION_FREQUENCY determines the execution rate of the control
// algorithm. It relies on non-blocking functions to be placed inside
// the execute callback.

#define TICK_FREQUENCY       5.0  // Hz
#define TIMEOUT_THRESHOLD    2.0  // seconds
#define EXECUTION_FREQUENCY  10.0 // Hz

#endif
