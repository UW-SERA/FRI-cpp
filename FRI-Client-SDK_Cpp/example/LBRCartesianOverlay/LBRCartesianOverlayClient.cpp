/**

The following license terms and conditions apply, unless a redistribution
agreement or other license is obtained by KUKA Deutschland GmbH, Augsburg, Germany.

SCOPE

The software “KUKA Sunrise.FRI Client SDK” is targeted to work in
conjunction with the “KUKA Sunrise.FRI” toolkit.
In the following, the term “software” refers to all material directly
belonging to the provided SDK “Software development kit”, particularly source
code, libraries, binaries, manuals and technical documentation.

COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2024 
KUKA Deutschland GmbH
Augsburg, Germany

LICENSE 

Redistribution and use of the software in source and binary forms, with or
without modification, are permitted provided that the following conditions are
met:
a) The software is used in conjunction with KUKA products only. 
b) Redistributions of source code must retain the above copyright notice, this
list of conditions and the disclaimer.
c) Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the disclaimer in the documentation and/or other
materials provided with the distribution. Altered source code of the
redistribution must be made available upon request with the distribution.
d) Modification and contributions to the original software provided by KUKA
must be clearly marked and the authorship must be stated.
e) Neither the name of KUKA nor the trademarks owned by KUKA may be used to
endorse or promote products derived from this software without specific prior
written permission.

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," without warranty of
any kind, including without limitation the warranties of merchantability,
fitness for a particular purpose and non-infringement. 
KUKA makes no warranty that the Software is free of defects or is suitable for
any particular purpose. In no event shall KUKA be responsible for loss or
damages arising from the installation or use of the Software, including but
not limited to any indirect, punitive, special, incidental or consequential
damages of any character including, without limitation, damages for loss of
goodwill, work stoppage, computer failure or malfunction, or any and all other
commercial damages or losses. 
The entire risk to the quality and performance of the Software is not borne by
KUKA. Should the Software prove defective, KUKA is not liable for the entire
cost of any service and repair.



\file
\version {2.7.0}
*/
#include <cstring>
#include <cstdio>
#include "LBRCartesianOverlayClient.h"
#include "friLBRState.h"
// Visual studio needs extra define to use math constants
#define _USE_MATH_DEFINES
#include <math.h>

using namespace KUKA::FRI;
//******************************************************************************
LBRCartesianOverlayClient::LBRCartesianOverlayClient(const double freqHz, const double amplRad, 
		const double filterCoeff) 
	   : _freqHz(freqHz)
	   , _amplRad(amplRad)
	   , _filterCoeff(filterCoeff)
	   , _offsetSin(0.0)
      , _offsetCos(0.0)
	   , _phi(0.0)
	   , _stepWidth(0.0)
      , _redundancyValue(0.0)
      , _increase(true)
{
   printf("LBRCartesianOverlayClient initialized:\n"
		   "\tfrequency (Hz): %f\n"
		   "\tamplitude (rad): %f\n"
		   "\tfilterCoeff: %f\n",
		   freqHz, amplRad, filterCoeff);
}

//******************************************************************************
LBRCartesianOverlayClient::~LBRCartesianOverlayClient()
{
}
      
//******************************************************************************
void LBRCartesianOverlayClient::onStateChange(const ESessionState oldState, const ESessionState newState)
{
   LBRClient::onStateChange(oldState, newState);

   switch (newState)
   {
      case MONITORING_READY:
      {
          _offsetSin = 0.0;
          _offsetCos = 0.0;
          _phi = 0.0;
          _stepWidth = 2 * M_PI * _freqHz * robotState().getSampleTime();
    	  break;
      }
      case IDLE:
      case MONITORING_WAIT:
      case COMMANDING_WAIT:
      case COMMANDING_ACTIVE:
      default:
      {
         break;
      }
   }
}

//******************************************************************************
void LBRCartesianOverlayClient::monitor()
{
	LBRClient::monitor();
	
   _redundancyValue = robotState().getMeasuredRedundancyValue();
	
	printf("Base --> TCP pose: %.3f / %.3f / %.3f \n", robotState().getMeasuredCartesianPose()[0],
			robotState().getMeasuredCartesianPose()[1], robotState().getMeasuredCartesianPose()[2]);
}
   
//******************************************************************************
void LBRCartesianOverlayClient::command()
{
   // calculate new offsets
   const double newOffsetSin = _amplRad * sin(_phi);
   _offsetSin = (_offsetSin * _filterCoeff) + (newOffsetSin * (1.0 - _filterCoeff));
   const double newOffsetCos = _amplRad * cos(_phi);
   _offsetCos = (_offsetCos * _filterCoeff) + (newOffsetCos * (1.0 - _filterCoeff));

   _phi += _stepWidth;
   if (_phi >= (2 * M_PI))
   {
      _phi -= (2 * M_PI);
   }

   // add offset to ipo cart pose for x and y directions
   double ipoCartPos[7];
   memcpy(ipoCartPos, robotState().getIpoCartesianPose(), 7 * sizeof(double));

   ipoCartPos[0] += _offsetSin;
   ipoCartPos[1] += _offsetCos;
   
   // changing redundancy value between -45° <= redundancy <= 45°
   if (_increase)
   {
       if (_redundancyValue < degToRad(45.0))
       {
           _redundancyValue += degToRad(0.1);
       }
       else
       {
           _increase = false;
       }
   }
   else
   {
       if (_redundancyValue > degToRad(-45.0))
       {
           _redundancyValue -= degToRad(0.1);
       }
       else
       {
           _increase = true;
       }
   }
   robotCommand().setCartesianPose(ipoCartPos, &_redundancyValue);   
   
}
//******************************************************************************
double LBRCartesianOverlayClient::degToRad(const double angleInDegrees) const
{
   return (angleInDegrees * M_PI) / 180.0;
}

//******************************************************************************

// clean up additional defines
#ifdef _USE_MATH_DEFINES
#undef _USE_MATH_DEFINES
#endif
