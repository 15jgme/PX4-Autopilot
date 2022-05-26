/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: slipstreamSimp.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 26-May-2022 11:29:26
 */

/* Include Files */
#include "slipstreamSimp.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }
    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }
    y = atan2(b_u0, b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }
  return y;
}

/*
 * if rpm > 6710 %RPM
 *          rpm = 6710;
 *      elseif rpm < 0
 *          rpm = 0;
 *      end
 *
 * Arguments    : double va_in
 *                double rpm
 * Return Type  : double
 */
double slipstreamSimp(double va_in, double rpm)
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
  double J;
  double Vpit_plus_a;
  double d;
  double n;
  double psiT;
  double psiT_tmp;
  double qx1;
  double qx2;
  double rx;
  double scale;
  double wOut;
  int b_low_i;
  int high_i;
  int low_i;
  int low_ip1;
  int mid_i;
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
  psiT_tmp = fabs(va_in);
  psiT = fmax(0.0, fmin(1.5707963267948966, rt_atan2d_snf(0.0, psiT_tmp)));
  /*  in-plane angle (0 to +/- 180) */
  if (rpm < 1.0) {
    /* 1716 % Too slow to run the motor */
    wOut = 0.0;
    qx2 = 0.0;
  } else {
    wOut = rpm;
    /*  advance ratio based on total velocity */
    J = fmax(0.0, fmin(1.0, sqrt(va_in * va_in) / (rpm / 60.0 * 0.254)));
    if (va_in >= 0.0) {
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
      scale = V[low_ip1 - 1];
      high_i = b_low_i + 10 * low_i;
      qx1 = V[high_i - 1];
      n = V[low_ip1];
      qx2 = V[high_i];
      d = dv[low_i - 1];
      if (J == d) {
        qx1 = scale;
        qx2 = n;
      } else if (!(J == dv[low_i])) {
        rx = (J - d) / (dv[low_i] - d);
        if (scale == qx1) {
          qx1 = scale;
        } else {
          qx1 = (1.0 - rx) * scale + rx * qx1;
        }
        if (n == qx2) {
          qx2 = n;
        } else {
          qx2 = (1.0 - rx) * n + rx * qx2;
        }
      }
      d = 0.17453292519943295 * ((double)b_low_i - 1.0);
      if ((psiT == d) || (qx1 == qx2)) {
        qx2 = qx1;
      } else {
        rx = 0.17453292519943295 * (double)b_low_i;
        if (!(psiT == rx)) {
          rx = (psiT - d) / (rx - d);
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
  scale = 3.3121686421112381E-170;
  if (psiT_tmp > 3.3121686421112381E-170) {
    rx = 1.0;
    scale = psiT_tmp;
  } else {
    J = psiT_tmp / 3.3121686421112381E-170;
    rx = J * J;
  }
  rx = scale * sqrt(rx);
  if ((va_in >=
       -0.2 * (0.5 * (1.59 * (wOut / 60.0) * 0.254 * 0.39538588745679831))) &&
      (qx2 > 0.0)) {
    /*  if reverse velocity is greater than 20% of hover velocity */
    rx = (2.0 * sqrt(2.0 * qx2 * 0.064516 * (n * n) +
                     3.1415926535897931 * (rx * rx) / 4.0) -
          rx * 1.7724538509055159) /
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
    scale = 0.0579416;
  } else {
    /* Turbine state */
    scale = 0.032265;
  }
  if (qx2 < 0.0) {
    /* Turbine state */
    /* Wang equation modified */
    J = sqrt(va_in * va_in -
             fabs(qx2) * (n * n) * 0.004162314256 / 0.050670747909749778);
    /* Lam Wang 2014 */
    /*          V0 = sqrt(va_1^2 - (1.39*n*D*sqrt(-CFX))^2); */
  } else {
    /* Normal operation */
    J = 2.0 * rx * 0.91823899371069173;
    /*  Efflux velocity from Vi0_avg original */
  }
  /*     %% Calculate slipstream at pitot tube */
  if (qx2 > 0.0) {
    /*  Eq. 16 */
    /*  Eq. 17 */
    rx = (0.12 - scale * 0.60678724409448825) / (0.5176 * scale + 0.076185738);
    Vpit_plus_a = va_in + J * 1.0235069291338583 * exp(-(rx * rx));
    /*  Eq. 18 */
  } else {
    /*  CFX < 0 assume turbine conditions  */
    /* Original */
    /*              A = -0.07 * ( (pitR - Rm0) / ( (Rm0 /2) - 0.204*(pitX - x0 -
     * Rp)) )^2; */
    /* Edited, replaced Rm0 with -Rm0 */
    rx = (0.12 - scale) / (scale / 2.0 + 0.06772065599999999);
    Vpit_plus_a =
        va_in - (va_in - fabs(J) * 0.866244094488189) * exp(-0.07 * (rx * rx));
  }
  return Vpit_plus_a;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void slipstreamSimp_initialize(void)
{
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void slipstreamSimp_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for slipstreamSimp.c
 *
 * [EOF]
 */
