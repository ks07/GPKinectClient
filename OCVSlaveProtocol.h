#include <opencv2/opencv.hpp>
#include <vector>
#include "GPKinectAPI/OCVSPacket.h"
#include "KinectInterface.h"

class OCVSlaveProtocol
{
public:
	OCVSlaveProtocol(char *host, char *port);
	~OCVSlaveProtocol();

	void Connect();

	bool CallVision(std::vector<cv::RotatedRect> &found);

private:
	const char * const host;
	const char * const port;

	uint8_t *imgarr = NULL;

	std::vector<char> recvBuff;
	std::vector<OCVSPacket> pktSendBuff;

	KinectInterface *kinect;

	bool initSuccess = false;
};