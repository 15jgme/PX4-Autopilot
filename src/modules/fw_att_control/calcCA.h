/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: calcCA.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 18-May-2022 11:47:21
 */

#ifndef CALCCA_H
#define CALCCA_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void calcCA(double v_meas, double rpm, double vinf, double *M1,
                   double *M2, double *M3, double *M4, double *M5, double *M6,
                   double *M7, double *M8, double *M9);

extern void calcCA_initialize(void);

extern void calcCA_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for calcCA.h
 *
 * [EOF]
 */
