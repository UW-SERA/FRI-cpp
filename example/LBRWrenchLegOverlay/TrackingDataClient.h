#include "openigtlink/igtlOSUtil.h"
#include "openigtlink/igtlTrackingDataMessage.h"
#include "openigtlink/igtlClientSocket.h"

class Tracking_data_client
{
public:
	igtl::ClientSocket::Pointer socket;
	void init();
	void loop();
};
