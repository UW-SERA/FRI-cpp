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
#ifndef _KUKA_FRI_LBR_CARTESIAN_OVERLAY_CLIENT_H
#define _KUKA_FRI_LBR_CARTESIAN_OVERLAY_CLIENT_H

#include "friLBRClient.h"



/**
 * \brief Example client that can overlay interpolator Cartesian poses. During command(), 
 * a circular motion in the XY plane is going to be performed, while the redundancy is slowly moved 
 * between -45 and 45 deg.
 */
class LBRCartesianOverlayClient : public KUKA::FRI::LBRClient
{
   
public:
      
   /**
    * \brief Constructor. 
    * 
    * @param freqHz Frequency in hertz
    * @param amplRad Amplitude in radians
    * @param filterCoeff Filter coefficient between 0 (filter off) and 1 (max filter)
    */
   LBRCartesianOverlayClient(double freqHz, double amplRad, double filterCoeff);
   
   /** 
    * \brief Destructor.
    */
   ~LBRCartesianOverlayClient();
   
   /**
    * \brief Callback for FRI state changes.
    * 
    * @param oldState
    * @param newState
    */
   virtual void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState);
   
   /**
    * \brief Callback for the FRI state 'Monitoring Ready'.
    */
   virtual void monitor();
   
   /**
    * \brief Callback for the FRI state 'Commanding Active'.
    */
   virtual void command();
   
   /**
    * \brief Converts degrees to radians for commanding with FRI
    * 
    * @param angleInDegrees angle to be converted
    * @return double containing the converted angle
    */
   double degToRad(double angleInDegrees) const;
      
private:
   double _freqHz;          //!< frequency (Hertz)
   double _amplRad;         //!< amplitude (radians)
   double _filterCoeff;     //!< filter coefficient
   double _offsetSin;       //!< offset for current interpolation step (x-direction)
   double _offsetCos;       //!< offset for current interpolation step (y-direction)
   double _phi;             //!< phase of sine wave
   double _stepWidth;       //!< stepwidth for sine 
   double _redundancyValue; //!< current redundancy value for Cartesian poses (radians)
   bool _increase;          //!< whether the redundancy value should be increased or decreased in the next FRI command  
      
};

#endif // _KUKA_FRI_LBR_CARTESIAN_OVERLAY_CLIENT_H
