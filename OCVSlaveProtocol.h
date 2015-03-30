#include <opencv2/opencv.hpp>
#include "libkoki/include/koki.h"
#include <vector>
#include "GPKinectAPI/OCVSPacket.h"
#include "KinectInterface.h"

class OCVSlaveProtocol
{
public:
	OCVSlaveProtocol(char *host, char *port);
	~OCVSlaveProtocol();

	void Connect();

	bool CallDepthVision(std::vector<cv::RotatedRect> &found);
    bool CallRGBVision(std::vector<libkoki::Marker> &found);


private:
	const char * const host;
	const char * const port;

	uint8_t *dimgarr = NULL;

	std::vector<char> recvBuff;
	std::vector<OCVSPacket> pktSendBuff;

	KinectInterface *kinect;

	bool initSuccess = false;
};