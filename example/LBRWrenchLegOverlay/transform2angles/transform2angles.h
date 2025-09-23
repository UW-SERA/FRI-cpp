//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: transform2angles.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 23-Sep-2025 16:31:11
//

#ifndef TRANSFORM2ANGLES_H
#define TRANSFORM2ANGLES_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void transform2angles(const double T_femur[4][4],
                             const double T_tibia[4][4], double angles[4]);

extern void transform2angles_initialize();

extern void transform2angles_terminate();

#endif
//
// File trailer for transform2angles.h
//
// [EOF]
//
