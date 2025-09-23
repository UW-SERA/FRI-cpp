//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: transform2angles.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 23-Sep-2025 16:31:11
//

// Include Files
#include "transform2angles.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const double T_femur[4][4]
//                const double T_tibia[4][4]
//                double angles[4]
// Return Type  : void
//
void transform2angles(const double T_femur[4][4], const double T_tibia[4][4],
                      double angles[4])
{
  angles[0] = std::atan2(T_femur[0][2], T_femur[2][2]);
  angles[1] = -std::asin(T_femur[1][2]);
  angles[2] = std::atan2(T_femur[1][0], T_femur[1][1]);
  angles[3] = std::atan2(T_tibia[1][0], T_tibia[1][1]);
}

//
// Arguments    : void
// Return Type  : void
//
void transform2angles_initialize()
{
}

//
// Arguments    : void
// Return Type  : void
//
void transform2angles_terminate()
{
}

//
// File trailer for transform2angles.cpp
//
// [EOF]
//
