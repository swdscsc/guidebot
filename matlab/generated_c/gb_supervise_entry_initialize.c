/*
 * File: gb_supervise_entry_initialize.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-10-12 22:03:07
 */

/* Include Files */
#include "gb_supervise_entry_initialize.h"
#include "gb_supervise_entry_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void gb_supervise_entry_initialize(void)
{
  rt_InitInfAndNaN();
  isInitialized_gb_supervise_entry = true;
}

/*
 * File trailer for gb_supervise_entry_initialize.c
 *
 * [EOF]
 */
