/*=========================================================================

  Program:   OpenIGTLink -- Example for Tracker Client Program
  Language:  C++

  Copyright (c) Insight Software Consortium. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include <iostream>
#include <math.h>
#include <cstdlib>
#include <cstring>

#include "openigtlink/igtlOSUtil.h"
#include "openigtlink/igtlTrackingDataMessage.h"
#include "openigtlink/igtlClientSocket.h"
#include "TrackingDataClient.h"


int ReceiveTrackingData(igtl::ClientSocket::Pointer& socket, igtl::MessageHeader::Pointer& header);

void Tracking_data_client::init()
{
    //------------------------------------------------------------
    // Parse Arguments

    //if (argc != 4) // check number of arguments
    //{
    //    // If not correct, print usage
    //    std::cerr << "Usage: " << argv[0] << " <hostname> <port> <fps>" << std::endl;
    //    std::cerr << "    <hostname> : IP or host name" << std::endl;
    //    std::cerr << "    <port>     : Port # (18944 in Slicer default)" << std::endl;
    //    std::cerr << "    <fps>      : Frequency (fps) to send coordinate" << std::endl;
    //    //exit(0);
    //}

    //char*  hostname = argv[1];
    //int    port     = atoi(argv[2]);
    //double fps      = atof(argv[3]);
    char*  hostname = "127.0.0.1";
    int    port = 18945;
    double fps = 2;
    int    interval = (int)(1000.0 / fps);

    //------------------------------------------------------------
    // Establish Connection

    //igtl::ClientSocket::Pointer socket;
    socket = igtl::ClientSocket::New();
    int r = socket->ConnectToServer(hostname, port);

    if (r != 0)
    {
        std::cerr << "Cannot connect to the server." << std::endl;
        exit(0);
    }

    //------------------------------------------------------------
    // Ask the server to start pushing tracking data
    /*std::cerr << "Sending STT_TDATA message....." << std::endl;
    igtl::StartTrackingDataMessage::Pointer startTrackingMsg;
    startTrackingMsg = igtl::StartTrackingDataMessage::New();
    startTrackingMsg->SetDeviceName("TDataClient");
    startTrackingMsg->SetResolution(interval);
    startTrackingMsg->SetCoordinateName("Patient");
    startTrackingMsg->Pack();
    socket->Send(startTrackingMsg->GetPackPointer(), startTrackingMsg->GetPackSize());

    int loop = 0;*/
}

void Tracking_data_client::loop()
{
    //------------------------------------------------------------
    // Wait for a reply
    igtl::MessageHeader::Pointer headerMsg;
    headerMsg = igtl::MessageHeader::New();
    headerMsg->InitPack();
    bool timeout(false);
    igtlUint64 rs = socket->Receive(headerMsg->GetPackPointer(), headerMsg->GetPackSize(), timeout);

    if (rs == 0)
    {
        std::cerr << "Connection closed." << std::endl;
        socket->CloseSocket();
        exit(0);
    }
    if (rs != headerMsg->GetPackSize())
    {
        std::cerr << "Message size information and actual data size don't match." << std::endl; 
        socket->CloseSocket();
        exit(0);
    }
    
    headerMsg->Unpack();

    if (strcmp(headerMsg->GetDeviceType(), "TRANSFORM") == 0)
    {   
        ReceiveTrackingData(socket, headerMsg);
        //std::cerr << "Receiving : " << headerMsg->GetDeviceType() << std::endl;
        socket->Skip(headerMsg->GetBodySizeToRead(), 0);
    }
    else
    {
        std::cerr << "Receiving : " << headerMsg->GetDeviceType() << std::endl;
        std::cerr << " msg pack : " << headerMsg->GetPackSize() << "byte" << " , address : " << headerMsg->GetPackPointer() << std::endl;

        socket->Skip(headerMsg->GetBodySizeToRead(), 0);
    }
}


int ReceiveTrackingData(igtl::ClientSocket::Pointer& socket, igtl::MessageHeader::Pointer& header)
{
  std::cerr << "Receiving TRANSFORM data type." << std::endl;
  std::cerr << " msg pack : " << header->GetPackSize() << "byte" << " , address : " << header->GetPackPointer() << std::endl;
  
  //------------------------------------------------------------
  // Allocate TrackingData Message Class

  igtl::TrackingDataMessage::Pointer trackingData;
  trackingData = igtl::TrackingDataMessage::New();
  (igtl::TrackingDataMessage*) trackingData->SetMessageHeader(header);
  trackingData->AllocatePack();

  // Receive body from the socket
  bool timeout(false);
  socket->Receive(trackingData->GetPackBodyPointer(), trackingData->GetPackBodySize(), timeout);

  std::cerr << " msg body : " << trackingData->GetPackBodySize() << "byte" << " , address : " << trackingData->GetPackBodyPointer() <<std::endl;

  // Deserialize the transform data
  // If you want to skip CRC check, call Unpack() without argument.
  int c = trackingData->Unpack(1);
  c = 2; // skip CRC and go straight into unpacking

  if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
  {
    int nElements = trackingData->GetNumberOfTrackingDataElements();
	std::cerr << " message: " << nElements << " elements." << std::endl;

    for (int i = 0; i < nElements; i ++)
      {
      igtl::TrackingDataElement::Pointer trackingElement;
      trackingData->GetTrackingDataElement(i, trackingElement);

      igtl::Matrix4x4 matrix;
      trackingElement->GetMatrix(matrix);


      std::cerr << "========== Element #" << i << " ==========" << std::endl;
      std::cerr << " Name       : " << trackingElement->GetName() << std::endl;
      std::cerr << " Type       : " << (int) trackingElement->GetType() << std::endl;
      std::cerr << " Matrix : " << std::endl;
      igtl::PrintMatrix(matrix);
      std::cerr << "================================" << std::endl;
      }
    return 1;
  }
  else     
  {
    std::cerr << " CRC error" << std::endl;
  }
  return 0;
}


