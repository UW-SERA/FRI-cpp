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
#include <cstdio>
#include "friMonitoringMessageDecoder.h"
#include "pb_decode.h"
#include "crc32.h"

using namespace KUKA::FRI;

//******************************************************************************
MonitoringMessageDecoder::MonitoringMessageDecoder(FRIMonitoringMessage* pMessage, int num)
   : m_nNum(num)
   , m_pMessage(pMessage)
{
   initMessage();
}

//******************************************************************************
MonitoringMessageDecoder::~MonitoringMessageDecoder()
{
}

//******************************************************************************
void MonitoringMessageDecoder::initMessage()
{
   // set initial data
   // it is assumed that no robot information and monitoring data is available and therefore the 
   // optional fields are initialized with false
   m_pMessage->has_robotInfo = false;
   m_pMessage->has_monitorData = false;
   m_pMessage->has_connectionInfo = true;
   m_pMessage->has_ipoData = false;
   m_pMessage->requestedTransformations_count = 0;

   m_pMessage->header.messageIdentifier = 0;
   m_pMessage->header.reflectedSequenceCounter = 0;
   m_pMessage->header.sequenceCounter = 0;

   m_pMessage->connectionInfo.sessionState = FRISessionState_IDLE;
   m_pMessage->connectionInfo.quality = FRIConnectionQuality_POOR;

   m_pMessage->monitorData.readIORequest_count = 0;
   m_pMessage->monitorData.measuredCartesianPose.element_count = 0;
   m_pMessage->monitorData.externalForce.element_count = 0;

   // allocate and map memory for protobuf repeated structures
   map_repeatedDouble(FRI_MANAGER_NANOPB_DECODE, m_nNum, 
         &m_pMessage->monitorData.measuredJointPosition.value,
         &m_tSendContainer.m_AxQMsrLocal);
   
   map_repeatedDouble(FRI_MANAGER_NANOPB_DECODE, m_nNum, 
         &m_pMessage->monitorData.measuredTorque.value,
         &m_tSendContainer.m_AxTauMsrLocal);
   
   map_repeatedDouble(FRI_MANAGER_NANOPB_DECODE, m_nNum, 
         &m_pMessage->monitorData.commandedTorque.value,
         &m_tSendContainer.m_AxTauCmdLocal);
   
   map_repeatedDouble(FRI_MANAGER_NANOPB_DECODE, m_nNum, 
         &m_pMessage->monitorData.externalTorque.value,
         &m_tSendContainer.m_AxTauExtMsrLocal);
   
   map_repeatedDouble(FRI_MANAGER_NANOPB_DECODE,m_nNum,
         &m_pMessage->ipoData.jointPosition.value,
         &m_tSendContainer.m_AxQCmdIPO);

   map_repeatedInt(FRI_MANAGER_NANOPB_DECODE, m_nNum, 
         &m_pMessage->robotInfo.driveState,
         &m_tSendContainer.m_AxDriveStateLocal);
}

//******************************************************************************
bool MonitoringMessageDecoder::decode(char* buffer, unsigned int size)
{
   pb_istream_t stream = pb_istream_from_buffer(reinterpret_cast<uint8_t*>(buffer), size);
   
   const size_t beforeLength = stream.bytes_left;
   uint32_t length;
   if (!pb_decode_fixed32(&stream, &length))
   {
      printf("Decoding error on monitor message length: %s\n", PB_GET_ERROR(&stream));
      return false;
   }
   const size_t afterLength = stream.bytes_left;

   const unsigned int lengthSize = beforeLength - afterLength;
   char* msgBuffer = buffer + lengthSize;
   const unsigned int msgSize = length;

   pb_istream_t msgStream = pb_istream_from_buffer(reinterpret_cast<uint8_t*>(msgBuffer), msgSize);
   if (!pb_decode(&msgStream, FRIMonitoringMessage_fields, m_pMessage))
   {
       printf("Decoding error on monitor message: %s\n", PB_GET_ERROR(&msgStream));
       return false;
   }

   uint32_t msgChecksum;
   char* checksumBuffer = msgBuffer + msgSize;
   pb_istream_t checksumStream = pb_istream_from_buffer(reinterpret_cast<uint8_t*>(checksumBuffer), msgSize);
   if (!pb_decode_fixed32(&checksumStream, &msgChecksum))
   {
       printf("Decoding error on monitor message checksum: %s\n", PB_GET_ERROR(&checksumStream));
       return false;
   }

   uint32_t bufferChecksum = KUKA::FRI::CRC32::calculate(reinterpret_cast<uint8_t*>(msgBuffer), msgSize);
   if (bufferChecksum != msgChecksum)
   {
      printf("Checksum msg: %x, checksum calculated: %x don't match\n", 
            static_cast<unsigned int>(msgChecksum), static_cast<unsigned int>(bufferChecksum));
      return false;
   }

   return true;
}
