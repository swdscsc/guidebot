/*
 * File: gb_supervise_entry.h
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-10-12 22:03:07
 */

#ifndef GB_SUPERVISE_ENTRY_H
#define GB_SUPERVISE_ENTRY_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void gb_supervise_entry(
    double plan_delta, double min_clear, double v, double prev_delta,
    double horizon, double delta_max, double wheelbase, double target_clear,
    double overhead, double drop, double front_clear, double ttc, double mu,
    double g, double latency_s, double margin, double slew_max_dps,
    double v_min, double body_clear, double ttc_brake_s, double safety_clear,
    double slow_threshold, double enable_brake_rule, double enable_gate,
    double enable_slew_limit, double dt, double *delta, double *brake,
    double *cue, double *reason, double *d_req);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for gb_supervise_entry.h
 *
 * [EOF]
 */
