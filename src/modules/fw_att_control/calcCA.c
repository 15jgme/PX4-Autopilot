/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: calcCA.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 18-May-2022 11:47:21
 */

/* Include Files */
#include "calcCA.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double v_meas
 *                double rpm
 *                double vinf
 *                double *M1
 *                double *M2
 *                double *M3
 *                double *M4
 *                double *M5
 *                double *M6
 *                double *M7
 *                double *M8
 *                double *M9
 * Return Type  : void
 */
void calcCA(double v_meas, double rpm, double vinf, double *M1, double *M2,
            double *M3, double *M4, double *M5, double *M6, double *M7,
            double *M8, double *M9)
{
  static const double V[110] = {
      0.156336454,  0.156336454,  0.156336454,  0.156336454,  0.156336454,
      0.156336454,  0.156336454,  0.156336454,  0.156336454,  0.156336454,
      0.142939755,  0.143274298,  0.144236696,  0.145715204,  0.147592844,
      0.149730239,  0.151960378,  0.154120349,  0.156099101,  0.157836404,
      0.12449467,   0.125294853,  0.127662714,  0.131491476,  0.136533063,
      0.142227959,  0.148100411,  0.153647038,  0.158176177,  0.161723393,
      0.103654093,  0.104978752,  0.10890636,   0.115283154,  0.123846679,
      0.133924798,  0.144287776,  0.153875413,  0.161071479,  0.163852071,
      0.080437634,  0.082320505,  0.087900516,  0.096990697,  0.109258684,
      0.124116893,  0.139693524,  0.153953862,  0.163349211,  0.165797264,
      0.054966543,  0.057426888,  0.064729471,  0.076657686,  0.092816061,
      0.112598271,  0.133992265,  0.153533548,  0.165846339,  0.168474477,
      0.027444178,  0.03049664,   0.039588216,  0.054473185,  0.074693054,
      0.099537509,  0.127172755,  0.152570626,  0.168638349,  0.171699702,
      -0.001874328, 0.001785855,  0.012726044,  0.030695378,  0.05514811,
      0.085260174,  0.119365045,  0.151139537,  0.171659606,  0.175259993,
      -0.032728784, -0.02842523,  -0.015539759, 0.005645211,  0.034482474,
      0.070037437,  0.11072865,   0.149362748,  0.174874022,  0.179027738,
      -0.064814458, -0.059769732, -0.044818343, -0.020318541, 0.013005062,
      0.054122603,  0.101419857,  0.147360112,  0.17827724,   0.182929631,
      -0.097502535, -0.091725529, -0.074619748, -0.046776089, -0.008957689,
      0.037754768,  0.091641937,  0.145232828,  0.181871525,  0.18696977};
  static const double dv[11] = {
      0.0, 0.1, 0.2, 0.30000000000000004, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
  double M[9];
  double x[9];
  double J;
  double Rm0;
  double Va;
  double n;
  double psiT;
  double psiT_tmp;
  double qx1;
  double qx2;
  double rx;
  double wOut;
  int b_low_i;
  int high_i;
  int low_i;
  int low_ip1;
  int mid_i;
  /*  Calc aileron area and a.c. */
  /* m */
  /* deg */
  /* m training edge full length */
  /* m training edge full length */
  /*  a.c. locations reletive to the tlhc/trhc of the aileron */
  /*  approximate */
  /*  cg_to_tlhc_ail = [0, 0, 0]'; % approximate */
  /*  Surface coeffecients  */
  /*  extract nominal area */
  /*  Other a.c. distances */
  /*  r_r = [-0.6, 0, -0.015]'; */
  /*  Airspeed */
  /* vmeas is assumed to be the velocity at the a.c. of the tail locations */
  /*   */
  /*      if rpm > 6710 %RPM */
  /*          rpm = 6710; */
  /*      elseif rpm < 0 */
  /*          rpm = 0; */
  /*      end */
  /* Axial pitot tube position */
  /* Radial pitot tube position */
  /*     %% Prop tables */
  /*  Negative J data from Selig's database (for APC 10x4.7) % we don't use it
   */
  /*  */
  /*  Thruster model */
  /* Rh = 0.006; % propeller hub radius */
  /* Rp = 0.127; % propeller radius */
  /* D = 2*Rp;   % propeller dia */
  /* m */
  /* hand measured */
  /* ------------------------------------------------------------- */
  /*  total velocity */
  /*  in-plane velocity */
  /*  azimuth angle of thruster (0 to +90 deg.) */
  psiT_tmp = fabs(vinf);
  psiT = fmax(0.0, fmin(1.5707963267948966, atan2(0.0, psiT_tmp)));
  /*  in-plane angle (0 to +/- 180) */
  if (rpm < 1.0) {
    /* 1716 % Too slow to run the motor */
    wOut = 0.0;
    qx2 = 0.0;
  } else {
    wOut = rpm;
    /*  advance ratio based on total velocity */
    J = fmax(0.0, fmin(1.0, sqrt(vinf * vinf) / (rpm / 60.0 * 0.254)));
    if (vinf >= 0.0) {
      /*  Forward flight */
      low_i = 1;
      low_ip1 = 2;
      high_i = 11;
      while (high_i > low_ip1) {
        mid_i = (low_i + high_i) >> 1;
        if (J >= dv[mid_i - 1]) {
          low_i = mid_i;
          low_ip1 = mid_i + 1;
        } else {
          high_i = mid_i;
        }
      }
      b_low_i = 1;
      low_ip1 = 2;
      high_i = 10;
      while (high_i > low_ip1) {
        mid_i = (b_low_i + high_i) >> 1;
        if (psiT >= 0.17453292519943295 * ((double)mid_i - 1.0)) {
          b_low_i = mid_i;
          low_ip1 = mid_i + 1;
        } else {
          high_i = mid_i;
        }
      }
      low_ip1 = b_low_i + 10 * (low_i - 1);
      Rm0 = V[low_ip1 - 1];
      high_i = b_low_i + 10 * low_i;
      qx1 = V[high_i - 1];
      Va = V[low_ip1];
      qx2 = V[high_i];
      n = dv[low_i - 1];
      if (J == n) {
        qx1 = Rm0;
        qx2 = Va;
      } else if (J != dv[low_i]) {
        rx = (J - n) / (dv[low_i] - n);
        if (Rm0 == qx1) {
          qx1 = Rm0;
        } else {
          qx1 = (1.0 - rx) * Rm0 + rx * qx1;
        }
        if (Va == qx2) {
          qx2 = Va;
        } else {
          qx2 = (1.0 - rx) * Va + rx * qx2;
        }
      }
      n = 0.17453292519943295 * ((double)b_low_i - 1.0);
      if ((psiT == n) || (qx1 == qx2)) {
        qx2 = qx1;
      } else {
        rx = 0.17453292519943295 * (double)b_low_i;
        if (psiT != rx) {
          rx = (psiT - n) / (rx - n);
          qx2 = (1.0 - rx) * qx1 + rx * qx2;
        }
      }
    } else {
      /*  Rearward flight */
      /*  A/c to Bart's paper Fig.8 & 9, for rearward flight, Cfx and Cmx have
       */
      /*  approx. static value */
      qx2 = 0.15633;
    }
  }
  /*  Transformation into UAV XYZ frame */
  /*  Induced Velocity at Propeller Plane */
  /*  Induced velocity in hover */
  n = wOut / 60.0;
  rx = 3.3121686421112381E-170;
  if (psiT_tmp > 3.3121686421112381E-170) {
    Va = 1.0;
    rx = psiT_tmp;
  } else {
    J = psiT_tmp / 3.3121686421112381E-170;
    Va = J * J;
  }
  Va = rx * sqrt(Va);
  if ((vinf >=
       -0.2 * (0.5 * (1.59 * (wOut / 60.0) * 0.254 * 0.39538588745679831))) &&
      (qx2 > 0.0)) {
    /*  if reverse velocity is greater than 20% of hover velocity */
    rx = (2.0 * sqrt(2.0 * qx2 * 0.064516 * (n * n) +
                     3.1415926535897931 * (Va * Va) / 4.0) -
          Va * 1.7724538509055159) /
         3.5449077018110318;
  } else {
    rx = 0.0;
  }
  /*  Slipstream model */
  /* m */
  /* m */
  /*  Equations used from propeller slipstream paper */
  /*  Eq. 11 */
  /*  Eq. 12 */
  if (qx2 > 0.0) {
    /* normal operation */
    Rm0 = 0.0579416;
  } else {
    /* Turbine state */
    Rm0 = 0.032265;
  }
  if (qx2 < 0.0) {
    /* Turbine state */
    /* Wang equation modified */
    J = sqrt(vinf * vinf -
             fabs(qx2) * (n * n) * pow(0.254, 4.0) / 0.050670747909749778);
    /* Lam Wang 2014 */
    /*          V0 = sqrt(va_1^2 - (1.39*n*D*sqrt(-CFX))^2); */
  } else {
    /* Normal operation */
    J = 2.0 * rx * 0.91823899371069173;
    /*  Efflux velocity from Vi0_avg original */
  }
  /*     %% Calculate slipstream at pitot tube */
  if (qx2 > 0.0) {
    /*  Eq. 13 */
    /*  Eq. 14 */
    rx = (0.04 - Rm0 * 0.8772423588001117) /
         (0.8839 * Rm0 + 0.011182375488522809);
    rx = vinf + J * 1.1674268968176857 * exp(-(rx * rx));
    /*  Eq. 15 */
  } else {
    /*  CFX < 0 assume turbine conditions  */
    /* Original */
    /*              A = -0.07 * ( (pitR - Rm0) / ( (Rm0 /2) - 0.204*(pitX - x0 -
     * Rp)) )^2; */
    /* Edited, replaced Rm0 with -Rm0 */
    rx = (0.04 - Rm0) / (Rm0 / 2.0 + 0.017203654597727398);
    rx = vinf - (vinf - fabs(J) * 0.8166201294701313) * exp(-0.07 * (rx * rx));
  }
  /*  Aileron coeffs, moment/delta  */
  rx = 2.2452631578947366E-5 * (rx * rx);
  J = 8.4197368421052638E-5 * (vinf * vinf);
  Rm0 = v_meas * v_meas;
  Va = rx * -0.04 + J * -0.076253709121543514;
  M[0] = Va + Va;
  M[3] = 0.0;
  M[6] = 0.0;
  Va = rx * -0.032367640184938207 + J * -0.025450106368783011;
  M[1] = Va + Va;
  M[4] = Rm0 * 0.00040034 * -0.54;
  M[7] = 0.0;
  M[2] = 0.0;
  M[5] = 0.0;
  M[8] = Rm0 * 0.00044899 * -0.6;
  memcpy(&x[0], &M[0], 9U * sizeof(double));
  low_ip1 = 0;
  high_i = 3;
  if (fabs(M[1]) > fabs(M[0])) {
    low_ip1 = 3;
    high_i = 0;
    x[0] = M[1];
    x[1] = M[0];
    x[3] = M[4];
    x[4] = 0.0;
    x[6] = 0.0;
    x[7] = 0.0;
  }
  x[1] /= x[0];
  x[2] /= x[0];
  x[4] -= x[1] * x[3];
  x[5] -= x[2] * x[3];
  x[7] -= x[1] * x[6];
  x[8] -= x[2] * x[6];
  x[5] /= x[4];
  x[8] -= x[5] * x[7];
  *M9 = (x[1] * x[5] - x[2]) / x[8];
  *M8 = -(x[1] + x[7] * *M9) / x[4];
  M[low_ip1] = ((1.0 - x[3] * *M8) - x[6] * *M9) / x[0];
  M[low_ip1 + 1] = *M8;
  M[low_ip1 + 2] = *M9;
  *M9 = -x[5] / x[8];
  *M8 = (1.0 - x[7] * *M9) / x[4];
  M[high_i] = -(x[3] * *M8 + x[6] * *M9) / x[0];
  M[high_i + 1] = *M8;
  M[high_i + 2] = *M9;
  *M9 = 1.0 / x[8];
  *M8 = -x[7] * *M9 / x[4];
  *M1 = M[0];
  *M2 = M[1];
  *M3 = M[2];
  *M4 = M[3];
  *M5 = M[4];
  *M6 = M[5];
  *M7 = -(x[3] * *M8 + x[6] * *M9) / x[0];
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void calcCA_initialize(void)
{
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void calcCA_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for calcCA.c
 *
 * [EOF]
 */
