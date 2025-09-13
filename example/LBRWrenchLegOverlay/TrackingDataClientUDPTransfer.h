#include "openigtlink/igtlOSUtil.h"
#include "openigtlink/igtlTrackingDataMessage.h"
#include "openigtlink/igtlMessageRTPWrapper.h"
#include "openigtlink/igtlUDPClientSocket.h"

class Tracking_data_client_udp_transfer
{	
public:
	igtl::UDPClientSocket::Pointer socket;
	unsigned char* bufferPKT;
	igtl::MessageRTPWrapper::Pointer rtpWrapper;
	igtl::SimpleMutexLock* glock;
	void init();
	void loop();
	int ReceiveTrackingData(igtl::TrackingDataMessage::Pointer& msgData);
};
