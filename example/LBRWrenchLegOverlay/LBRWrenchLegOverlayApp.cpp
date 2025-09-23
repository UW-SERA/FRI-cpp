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
#include <cstdlib>
#include <cstdio>
#include <cstring> // strstr
#include "LBRWrenchLegOverlayClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include "TrackingDataClient.h"
#include "transform2angles/transform2angles.h"
#include "controller_comp/controller_comp.h"


using namespace KUKA::FRI;


const int DEFAULT_PORTID = 30200;



int main (const int argc, const char* const * const argv)
{
   // parse command line arguments
   if ((1 < argc) && (NULL != strstr (argv[1], "help")))
   {
	   printf(
	         "\nKUKA LBR wrench leg overlay application\n\n"
	         "\tCommand line arguments:\n"
	         "\t1) remote hostname (optional)\n"
	         "\t2) port ID (optional)\n"
	   );
	   return 1;
   }
   
   const char* const hostname = ((argc >= 2) && (argv[1][0] != '\0')) ? argv[1] : NULL;
   const int port = ((argc >= 3) && (argv[2][0] != '\0')) ? atoi(argv[2]) : DEFAULT_PORTID;
   
   printf("Enter LBRWrenchLegOverlay Client Application\n");

   /***************************************************************************/
   /*                                                                         */
   /*   Place user Client Code here                                           */
   /*                                                                         */
   /**************************************************************************/
   
   // create new tracking data transfer object
   Tracking_data_client tracker;

   static igtl::Matrix4x4 femur;
   static igtl::Matrix4x4 tibia;
   igtl::IdentityMatrix(femur);
   igtl::IdentityMatrix(tibia);


   // create new wrench overlay client
   LBRWrenchLegOverlayClient trafoClient;
   double wrench[LBRWrenchLegOverlayClient::CART_VECTOR_DIM] = {0};

   /***************************************************************************/
   /*                                                                         */
   /*   Standard application structure                                        */
   /*   Configuration                                                         */
   /*                                                                         */
   /***************************************************************************/


   // initialize tracker over tcp
   tracker.init();


   // create new udp connection to Robot
   UdpConnection connection;


   // pass connection and client to a new FRI client application
   ClientApplication app(connection, trafoClient);
   
   // connect client application to KUKA Sunrise controller
   bool success = 1;
   //success = app.connect(port, hostname);
   if (!success)
   {
      printf("\nConnection to KUKA Sunrise controller failed.");
   }


   /***************************************************************************/
   /*                                                                         */
   /*   Standard application structure                                        */
   /*   Execution mainloop                                                    */
   /*                                                                         */
   /***************************************************************************/

   // repeatedly call the step routine to receive and process FRI packets
   while (success)
   {
		// initialize tracking data
		igtl::TrackingDataElement::Pointer element;
		element = igtl::TrackingDataElement::New();

		// receive tracking data
		for (int i = 0; i < 2; i++) {
			element = tracker.loop();
			if ("FemurTrackerToRef" == std::string(element->GetName())) {
				//std::cerr << "Saved FemurTrackerToRef" << std::endl;
				element->GetMatrix(femur);
			}
			if ("TibiaTrackerToRef" == std::string(element->GetName())) {
				//std::cerr << "Saved TibiaTrackerToRef" << std::endl;
				element->GetMatrix(tibia);
			}
		}
		// Convert to double
		double femur_d[4][4], tibia_d[4][4];
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				femur_d[i][j] = static_cast<double>(femur[i][j]);
				tibia_d[i][j] = static_cast<double>(tibia[i][j]);
			}
		}


		// transform to general coordinates (tracker to angles)
		// input vector to controller_comp:
		// leg_angles (rad): hip adduction, hip external rotation, hip flexion, knee flexion
		double angles[4] = { 0 };
		transform2angles(femur_d, tibia_d, angles);
		std::cout << "Angles: " << angles[0] << ", " << angles[1] << ", " << angles[2] << ", " << angles[3] << std::endl;


		// calculate new forces in x and y direction
		// Call the control law function controller_comp:
		controller_comp(angles, wrench);
		std::cout << "Force : " << wrench[0] << ", " << wrench[1] << ", " << wrench[2] << std::endl;
		std::cout << "Torque: " << wrench[3] << ", " << wrench[4] << ", " << wrench[5] << std::endl;


	  // send the wrench values
	  trafoClient.setWrench(wrench);
      success = app.step();
      
      // check if we are in IDLE because the FRI session was closed
      if (trafoClient.robotState().getSessionState() == IDLE)
      {
         // In this demo application we simply quit.
         // Waiting for a new FRI session would be another possibility.
         break;
      }
   }

   /***************************************************************************/
   /*                                                                         */
   /*   Standard application structure                                        */
   /*   Dispose                                                               */
   /*                                                                         */
   /***************************************************************************/

   // disconnect from controller
   app.disconnect();
   
   printf("Exit LBRWrenchLegOverlay Client Application\n");
   
   return 1;
}
