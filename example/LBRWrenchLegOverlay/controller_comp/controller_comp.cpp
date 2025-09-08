//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: controller_comp.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Sep-2025 19:44:19
//

// Include Files
#include "controller_comp.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Type Definitions
struct struct_T {
  double d1[3];
  double d2[3];
  double d3[3];
  double d4[3];
  double g;
  double mass1;
  double mass2;
};

// Variable Definitions
static struct_T param;

static boolean_T isInitialized_controller_comp{false};

// Function Declarations
namespace coder {
namespace internal {
namespace blas {
static double xnrm2(int n, const double x[24], int ix0);

}
} // namespace internal
static void mldivide(const double A[24], const double B[4], double Y[6]);

} // namespace coder
static void controller_comp_init();

// Function Definitions
//
// Arguments    : int n
//                const double x[24]
//                int ix0
// Return Type  : double
//
namespace coder {
namespace internal {
namespace blas {
static double xnrm2(int n, const double x[24], int ix0)
{
  double y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      double scale;
      int kend;
      scale = 3.3121686421112381E-170;
      kend = ix0 + n;
      for (int k{ix0}; k < kend; k++) {
        double absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          double t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          double t;
          t = absxk / scale;
          y += t * t;
        }
      }
      y = scale * std::sqrt(y);
    }
  }
  return y;
}

//
// Arguments    : const double A[24]
//                const double B[4]
//                double Y[6]
// Return Type  : void
//
} // namespace blas
} // namespace internal
static void mldivide(const double A[24], const double B[4], double Y[6])
{
  __m128d r;
  double b_A[24];
  double vn1[6];
  double vn2[6];
  double work[6];
  double b_B[4];
  double tau[4];
  double d;
  double smax;
  int b_i;
  int idxmax;
  int ix;
  int k;
  int pvt;
  int rankA;
  signed char jpvt[6];
  std::copy(&A[0], &A[24], &b_A[0]);
  tau[0] = 0.0;
  tau[1] = 0.0;
  tau[2] = 0.0;
  tau[3] = 0.0;
  for (k = 0; k < 6; k++) {
    jpvt[k] = static_cast<signed char>(k + 1);
    work[k] = 0.0;
    d = internal::blas::xnrm2(4, A, (k << 2) + 1);
    vn1[k] = d;
    vn2[k] = d;
  }
  for (int i{0}; i < 4; i++) {
    double atmp;
    double s;
    int ii;
    int ip1;
    int lastc;
    int lastv;
    ip1 = i + 2;
    rankA = i << 2;
    ii = rankA + i;
    ix = 7 - i;
    idxmax = 0;
    smax = std::abs(vn1[i]);
    for (k = 2; k < ix; k++) {
      s = std::abs(vn1[(i + k) - 1]);
      if (s > smax) {
        idxmax = k - 1;
        smax = s;
      }
    }
    pvt = i + idxmax;
    if (pvt != i) {
      ix = pvt << 2;
      smax = b_A[ix];
      b_A[ix] = b_A[rankA];
      b_A[rankA] = smax;
      smax = b_A[ix + 1];
      b_A[ix + 1] = b_A[rankA + 1];
      b_A[rankA + 1] = smax;
      smax = b_A[ix + 2];
      b_A[ix + 2] = b_A[rankA + 2];
      b_A[rankA + 2] = smax;
      smax = b_A[ix + 3];
      b_A[ix + 3] = b_A[rankA + 3];
      b_A[rankA + 3] = smax;
      ix = jpvt[pvt];
      jpvt[pvt] = jpvt[i];
      jpvt[i] = static_cast<signed char>(ix);
      vn1[pvt] = vn1[i];
      vn2[pvt] = vn2[i];
    }
    if (i + 1 < 4) {
      atmp = b_A[ii];
      rankA = ii + 2;
      tau[i] = 0.0;
      smax = internal::blas::xnrm2(3 - i, b_A, ii + 2);
      if (smax != 0.0) {
        s = std::abs(b_A[ii]);
        smax = std::abs(smax);
        if (s < smax) {
          s /= smax;
          smax *= std::sqrt(s * s + 1.0);
        } else if (s > smax) {
          smax /= s;
          smax = s * std::sqrt(smax * smax + 1.0);
        } else {
          smax = s * 1.4142135623730951;
        }
        if (b_A[ii] >= 0.0) {
          smax = -smax;
        }
        if (std::abs(smax) < 1.0020841800044864E-292) {
          ix = 0;
          b_i = (ii - i) + 4;
          do {
            ix++;
            idxmax = (((((b_i - ii) - 1) / 2) << 1) + ii) + 2;
            pvt = idxmax - 2;
            for (k = rankA; k <= pvt; k += 2) {
              r = _mm_loadu_pd(&b_A[k - 1]);
              _mm_storeu_pd(&b_A[k - 1],
                            _mm_mul_pd(_mm_set1_pd(9.9792015476736E+291), r));
            }
            for (k = idxmax; k <= b_i; k++) {
              b_A[k - 1] *= 9.9792015476736E+291;
            }
            smax *= 9.9792015476736E+291;
            atmp *= 9.9792015476736E+291;
          } while ((std::abs(smax) < 1.0020841800044864E-292) && (ix < 20));
          s = std::abs(atmp);
          smax = std::abs(internal::blas::xnrm2(3 - i, b_A, ii + 2));
          if (s < smax) {
            s /= smax;
            smax *= std::sqrt(s * s + 1.0);
          } else if (s > smax) {
            smax /= s;
            smax = s * std::sqrt(smax * smax + 1.0);
          } else {
            smax = s * 1.4142135623730951;
          }
          if (atmp >= 0.0) {
            smax = -smax;
          }
          tau[i] = (smax - atmp) / smax;
          s = 1.0 / (atmp - smax);
          for (k = rankA; k <= pvt; k += 2) {
            r = _mm_loadu_pd(&b_A[k - 1]);
            _mm_storeu_pd(&b_A[k - 1], _mm_mul_pd(_mm_set1_pd(s), r));
          }
          for (k = idxmax; k <= b_i; k++) {
            b_A[k - 1] *= s;
          }
          for (k = 0; k < ix; k++) {
            smax *= 1.0020841800044864E-292;
          }
          atmp = smax;
        } else {
          tau[i] = (smax - b_A[ii]) / smax;
          s = 1.0 / (b_A[ii] - smax);
          b_i = (ii - i) + 4;
          ix = (((((b_i - ii) - 1) / 2) << 1) + ii) + 2;
          idxmax = ix - 2;
          for (k = rankA; k <= idxmax; k += 2) {
            r = _mm_loadu_pd(&b_A[k - 1]);
            _mm_storeu_pd(&b_A[k - 1], _mm_mul_pd(_mm_set1_pd(s), r));
          }
          for (k = ix; k <= b_i; k++) {
            b_A[k - 1] *= s;
          }
          atmp = smax;
        }
      }
      b_A[ii] = atmp;
    } else {
      tau[3] = 0.0;
    }
    atmp = b_A[ii];
    b_A[ii] = 1.0;
    rankA = ii + 5;
    if (tau[i] != 0.0) {
      boolean_T exitg2;
      lastv = 4 - i;
      ix = (ii - i) + 3;
      while ((lastv > 0) && (b_A[ix] == 0.0)) {
        lastv--;
        ix--;
      }
      lastc = 4 - i;
      exitg2 = false;
      while ((!exitg2) && (lastc + 1 > 0)) {
        int exitg1;
        ix = (ii + (lastc << 2)) + 4;
        idxmax = ix;
        do {
          exitg1 = 0;
          if (idxmax + 1 <= ix + lastv) {
            if (b_A[idxmax] != 0.0) {
              exitg1 = 1;
            } else {
              idxmax++;
            }
          } else {
            lastc--;
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    } else {
      lastv = 0;
      lastc = -1;
    }
    if (lastv > 0) {
      if (lastc + 1 != 0) {
        std::memset(&work[0], 0,
                    static_cast<unsigned int>(lastc + 1) * sizeof(double));
        b_i = (ii + (lastc << 2)) + 5;
        for (pvt = rankA; pvt <= b_i; pvt += 4) {
          smax = 0.0;
          k = pvt + lastv;
          for (idxmax = pvt; idxmax < k; idxmax++) {
            smax += b_A[idxmax - 1] * b_A[(ii + idxmax) - pvt];
          }
          ix = ((pvt - ii) - 5) >> 2;
          work[ix] += smax;
        }
      }
      if (-tau[i] != 0.0) {
        ix = ii;
        for (int j{0}; j <= lastc; j++) {
          d = work[j];
          if (d != 0.0) {
            smax = d * -tau[i];
            b_i = ix + 5;
            k = lastv + ix;
            for (idxmax = b_i; idxmax <= k + 4; idxmax++) {
              b_A[idxmax - 1] += b_A[((ii + idxmax) - ix) - 5] * smax;
            }
          }
          ix += 4;
        }
      }
    }
    b_A[ii] = atmp;
    for (int j{ip1}; j < 7; j++) {
      ix = i + ((j - 1) << 2);
      d = vn1[j - 1];
      if (d != 0.0) {
        smax = std::abs(b_A[ix]) / d;
        smax = 1.0 - smax * smax;
        if (smax < 0.0) {
          smax = 0.0;
        }
        s = d / vn2[j - 1];
        s = smax * (s * s);
        if (s <= 1.4901161193847656E-8) {
          if (i + 1 < 4) {
            d = internal::blas::xnrm2(3 - i, b_A, ix + 2);
            vn1[j - 1] = d;
            vn2[j - 1] = d;
          } else {
            vn1[j - 1] = 0.0;
            vn2[j - 1] = 0.0;
          }
        } else {
          vn1[j - 1] = d * std::sqrt(smax);
        }
      }
    }
  }
  rankA = 0;
  smax = 1.3322676295501878E-14 * std::abs(b_A[0]);
  while ((rankA < 4) && (std::abs(b_A[rankA + (rankA << 2)]) > smax)) {
    rankA++;
  }
  b_B[0] = B[0];
  b_B[1] = B[1];
  b_B[2] = B[2];
  b_B[3] = B[3];
  for (int i{0}; i < 6; i++) {
    Y[i] = 0.0;
  }
  for (int j{0}; j < 4; j++) {
    if (tau[j] != 0.0) {
      smax = b_B[j];
      b_i = j + 2;
      for (int i{b_i}; i < 5; i++) {
        smax += b_A[(i + (j << 2)) - 1] * b_B[i - 1];
      }
      smax *= tau[j];
      if (smax != 0.0) {
        b_B[j] -= smax;
        ix = ((((3 - j) / 2) << 1) + j) + 2;
        idxmax = ix - 2;
        for (int i{b_i}; i <= idxmax; i += 2) {
          __m128d r1;
          r = _mm_loadu_pd(&b_A[(i + (j << 2)) - 1]);
          r1 = _mm_loadu_pd(&b_B[i - 1]);
          _mm_storeu_pd(&b_B[i - 1],
                        _mm_sub_pd(r1, _mm_mul_pd(r, _mm_set1_pd(smax))));
        }
        for (int i{ix}; i < 5; i++) {
          b_B[i - 1] -= b_A[(i + (j << 2)) - 1] * smax;
        }
      }
    }
  }
  for (int i{0}; i < rankA; i++) {
    Y[jpvt[i] - 1] = b_B[i];
  }
  for (int j{rankA}; j >= 1; j--) {
    ix = jpvt[j - 1] - 1;
    idxmax = (j - 1) << 2;
    Y[ix] /= b_A[(j + idxmax) - 1];
    for (int i{0}; i <= j - 2; i++) {
      pvt = jpvt[i] - 1;
      Y[pvt] -= Y[ix] * b_A[i + idxmax];
    }
  }
}

//
// Arguments    : void
// Return Type  : void
//
} // namespace coder
static void controller_comp_init()
{
  static const struct_T r{
      {0.19, 0.0, 0.08}, // d1
      {0.38, 0.0, 0.08}, // d2
      {0.2, 0.0, 0.0},   // d3
      {0.3, 0.0, 0.0},   // d4
      9.81,              // g
      7.64,              // mass1
      3.14               // mass2
  };
  param = r;
}

//
// x = [euler1; euler2(3); w1; w2(3)];
//
// Arguments    : const double x[8]
//                double u_comp[6]
// Return Type  : void
//
void controller_comp(const double x[8], double u_comp[6])
{
  double dv6[24];
  double dv[12];
  double dv1[12];
  double dv2[12];
  double dv3[12];
  double dv4[4];
  double dv5[4];
  double b_t11_tmp;
  double b_t5_tmp;
  double b_t9_tmp;
  double d;
  double d1;
  double d10;
  double d11;
  double d12;
  double d13;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  double d7;
  double d8;
  double d9;
  double t10_tmp;
  double t11_tmp;
  double t13_tmp;
  double t16_tmp;
  double t17_tmp;
  double t18_tmp;
  double t19_tmp;
  double t20_tmp;
  double t21_tmp;
  double t2_tmp;
  double t32_tmp;
  double t33_tmp;
  double t36_tmp;
  double t37_tmp;
  double t3_tmp;
  double t4_tmp;
  double t5_tmp;
  double t6_tmp;
  double t7_tmp;
  double t8_tmp;
  double t9_tmp;
  if (!isInitialized_controller_comp) {
    controller_comp_initialize();
  }
  //  eom matrices of 3d leg system
  // %% Unpacking
  // %% Jacobians
  // jacobian_JT1
  //     JT1 = jacobian_JT1(IN1,IN2)
  //     This function was generated by the Symbolic Math Toolbox version 24.2.
  //     26-Aug-2025 16:23:07
  t2_tmp = std::cos(x[0]);
  t3_tmp = std::cos(x[1]);
  t4_tmp = std::cos(x[2]);
  t5_tmp = std::sin(x[0]);
  t6_tmp = std::sin(x[1]);
  t7_tmp = std::sin(x[2]);
  t8_tmp = t5_tmp * t7_tmp;
  t9_tmp = t2_tmp * t4_tmp;
  t10_tmp = t2_tmp * t7_tmp;
  t11_tmp = t4_tmp * t5_tmp;
  t18_tmp = t8_tmp + t6_tmp * t9_tmp;
  t19_tmp = t9_tmp + t6_tmp * t8_tmp;
  t20_tmp = t10_tmp - t6_tmp * t11_tmp;
  t21_tmp = t11_tmp - t6_tmp * t10_tmp;
  // jacobian_JT2
  //     JT2 = jacobian_JT2(IN1,IN2,IN3)
  //     This function was generated by the Symbolic Math Toolbox version 24.2.
  //     26-Aug-2025 16:23:07
  b_t5_tmp = std::cos(x[3]);
  b_t9_tmp = std::sin(x[3]);
  b_t11_tmp = t5_tmp * b_t9_tmp;
  t13_tmp = t2_tmp * b_t5_tmp;
  t16_tmp = t2_tmp * b_t9_tmp;
  t17_tmp = b_t5_tmp * t5_tmp;
  t32_tmp = b_t11_tmp + t6_tmp * t13_tmp;
  t33_tmp = t13_tmp + t6_tmp * b_t11_tmp;
  t36_tmp = t16_tmp - t6_tmp * t17_tmp;
  t37_tmp = t17_tmp - t6_tmp * t16_tmp;
  // jacobian_JT3
  //     JT3 = jacobian_JT3(IN1,IN2,IN3)
  //     This function was generated by the Symbolic Math Toolbox version 24.2.
  //     26-Aug-2025 16:23:08
  // jacobian_JR1
  //     JR1 = jacobian_JR1(IN1)
  //     This function was generated by the Symbolic Math Toolbox version 24.2.
  //     26-Aug-2025 16:23:07
  // jacobian_JR2
  //     JR2 = jacobian_JR2(IN1)
  //     This function was generated by the Symbolic Math Toolbox version 24.2.
  //     26-Aug-2025 16:23:07
  dv[0] = 0.0;
  dv[1] = 1.0;
  dv[2] = 0.0;
  dv[3] = t2_tmp;
  dv[4] = 0.0;
  dv[5] = -t5_tmp;
  dv[6] = 0.0;
  dv[7] = 0.0;
  dv[8] = 0.0;
  dv[9] = t3_tmp * t5_tmp;
  dv[10] = -t6_tmp;
  dv[11] = t2_tmp * t3_tmp;
  // %% Jacobian derivatives
  // %% Forces
  // %% Equations of Motion
  // %% Control law
  // %% gravity and inertia compensation
  // %% static force overlay, so y_ddot = 0
  // %% EoM: M*y_ddot + k = qe + qu;
  d = param.d4[2] * t2_tmp;
  d1 = param.d2[1] * t18_tmp - param.d2[0] * t21_tmp;
  d2 = param.d2[2] * t2_tmp * t3_tmp;
  dv1[0] = (((d1 + param.d4[1] * t32_tmp) - param.d4[0] * t37_tmp) + d2) +
           d * t3_tmp;
  dv1[1] = 0.0;
  d3 = param.d4[2] * t3_tmp;
  d4 = -param.d2[0] * t19_tmp + param.d2[1] * t20_tmp;
  d5 = param.d2[2] * t3_tmp * t5_tmp;
  dv1[2] = (((d4 - param.d4[0] * t33_tmp) + param.d4[1] * t36_tmp) - d5) -
           d3 * t5_tmp;
  d6 = param.d2[1] * t3_tmp;
  d7 = param.d2[0] * t3_tmp;
  d8 = param.d4[1] * t3_tmp;
  d9 = param.d4[0] * t3_tmp;
  d10 = (d7 * t8_tmp - param.d2[2] * t5_tmp * t6_tmp) + d6 * t11_tmp;
  dv1[3] =
      ((d10 + d9 * b_t11_tmp) - param.d4[2] * t5_tmp * t6_tmp) + d8 * t17_tmp;
  d11 = -param.d2[2] * t3_tmp;
  d12 = param.d2[1] * t4_tmp * t6_tmp;
  d13 = param.d2[0] * t6_tmp * t7_tmp;
  dv1[4] = ((((d11 - d3) - d12) - d13) - param.d4[1] * b_t5_tmp * t6_tmp) -
           param.d4[0] * t6_tmp * b_t9_tmp;
  d3 = (-param.d2[2] * t2_tmp * t6_tmp + d6 * t9_tmp) + d7 * t10_tmp;
  dv1[5] = ((d3 - d * t6_tmp) + d8 * t13_tmp) + d9 * t16_tmp;
  d = -param.d2[1] * t19_tmp - param.d2[0] * t20_tmp;
  dv1[6] = d;
  d6 = d7 * t4_tmp - d6 * t7_tmp;
  dv1[7] = d6;
  d7 = param.d2[0] * t18_tmp + param.d2[1] * t21_tmp;
  dv1[8] = d7;
  dv1[9] = -param.d4[1] * t33_tmp - param.d4[0] * t36_tmp;
  dv1[10] = d9 * b_t5_tmp - d8 * b_t9_tmp;
  dv1[11] = param.d4[0] * t32_tmp + param.d4[1] * t37_tmp;
  dv2[0] = (param.d1[1] * t18_tmp - param.d1[0] * t21_tmp) +
           param.d1[2] * t2_tmp * t3_tmp;
  dv2[4] = 0.0;
  dv2[8] = (-param.d1[0] * t19_tmp + param.d1[1] * t20_tmp) -
           param.d1[2] * t3_tmp * t5_tmp;
  d8 = param.d1[0] * t3_tmp;
  d9 = param.d1[1] * t3_tmp;
  dv2[1] = (d8 * t8_tmp - param.d1[2] * t5_tmp * t6_tmp) + d9 * t11_tmp;
  dv2[5] = (-param.d1[2] * t3_tmp - param.d1[1] * t4_tmp * t6_tmp) -
           param.d1[0] * t6_tmp * t7_tmp;
  dv2[9] = (-param.d1[2] * t2_tmp * t6_tmp + d8 * t10_tmp) + d9 * t9_tmp;
  dv2[2] = -param.d1[0] * t20_tmp - param.d1[1] * t19_tmp;
  dv2[6] = d8 * t4_tmp - d9 * t7_tmp;
  dv2[10] = param.d1[0] * t18_tmp + param.d1[1] * t21_tmp;
  dv2[3] = 0.0;
  dv2[7] = 0.0;
  dv2[11] = 0.0;
  t7_tmp = -param.mass1 * param.g;
  d8 = param.d3[2] * t2_tmp;
  dv3[0] = (((d1 + param.d3[1] * t32_tmp) - param.d3[0] * t37_tmp) + d2) +
           d8 * t3_tmp;
  dv3[1] = 0.0;
  d1 = param.d3[2] * t3_tmp;
  dv3[2] = (((d4 - param.d3[0] * t33_tmp) + param.d3[1] * t36_tmp) - d5) -
           d1 * t5_tmp;
  d2 = param.d3[1] * t3_tmp;
  d4 = param.d3[0] * t3_tmp;
  dv3[3] =
      ((d10 + d4 * b_t11_tmp) - param.d3[2] * t5_tmp * t6_tmp) + d2 * t17_tmp;
  dv3[4] = ((((d11 - d1) - d12) - d13) - param.d3[1] * b_t5_tmp * t6_tmp) -
           param.d3[0] * t6_tmp * b_t9_tmp;
  dv3[5] = ((d3 - d8 * t6_tmp) + d2 * t13_tmp) + d4 * t16_tmp;
  dv3[6] = d;
  dv3[7] = d6;
  dv3[8] = d7;
  dv3[9] = -param.d3[1] * t33_tmp - param.d3[0] * t36_tmp;
  dv3[10] = d4 * b_t5_tmp - d2 * b_t9_tmp;
  dv3[11] = param.d3[0] * t32_tmp + param.d3[1] * t37_tmp;
  t4_tmp = -param.mass2 * param.g;
  for (int i{0}; i < 4; i++) {
    dv5[i] = dv2[i + 4] * t7_tmp + dv3[3 * i + 1] * t4_tmp;
    dv4[i] = 0.0;
  }
  for (int i{0}; i < 3; i++) {
    int i1;
    int i2;
    i1 = i << 2;
    dv6[i1] = dv1[i];
    i2 = (i + 3) << 2;
    dv6[i2] = dv[i];
    dv6[i1 + 1] = dv1[i + 3];
    dv6[i2 + 1] = dv[i + 3];
    dv6[i1 + 2] = dv1[i + 6];
    dv6[i2 + 2] = dv[i + 6];
    dv6[i1 + 3] = dv1[i + 9];
    dv6[i2 + 3] = dv[i + 9];
  }
  __m128d r;
  __m128d r1;
  __m128d r2;
  r = _mm_loadu_pd(&dv5[0]);
  r1 = _mm_loadu_pd(&dv4[0]);
  r2 = _mm_set1_pd(-1.0);
  _mm_storeu_pd(&dv5[0], _mm_mul_pd(_mm_add_pd(r, r1), r2));
  r = _mm_loadu_pd(&dv5[2]);
  r1 = _mm_loadu_pd(&dv4[2]);
  _mm_storeu_pd(&dv5[2], _mm_mul_pd(_mm_add_pd(r, r1), r2));
  coder::mldivide(dv6, dv5, u_comp);
  r = _mm_loadu_pd(&u_comp[0]);
  r1 = _mm_set1_pd(150.0);
  r2 = _mm_set1_pd(-150.0);
  _mm_storeu_pd(&u_comp[0], _mm_max_pd(_mm_min_pd(r, r1), r2));
  r = _mm_loadu_pd(&u_comp[2]);
  _mm_storeu_pd(&u_comp[2], _mm_max_pd(_mm_min_pd(r, r1), r2));
  r = _mm_loadu_pd(&u_comp[4]);
  _mm_storeu_pd(&u_comp[4], _mm_max_pd(_mm_min_pd(r, r1), r2));
}

//
// Arguments    : void
// Return Type  : void
//
void controller_comp_initialize()
{
  controller_comp_init();
  isInitialized_controller_comp = true;
}

//
// Arguments    : void
// Return Type  : void
//
void controller_comp_terminate()
{
  isInitialized_controller_comp = false;
}

//
// File trailer for controller_comp.cpp
//
// [EOF]
//
