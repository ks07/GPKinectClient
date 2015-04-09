#include <opencv2/opencv.hpp>
#include <vector>
#include "GPKinectAPI/OCVSPacket.h"
#include "KinectInterface.h"
#include "OpenARScanner.h"

class OCVSlaveProtocol
{
public:
	OCVSlaveProtocol(char *host, char *port);
	~OCVSlaveProtocol();

	void Connect();

	bool CallDepthVision(std::vector<cv::RotatedRect> &found);
	bool CallRGBVision(ARMarkers &found);


private:
	const char * const host;
	const char * const port;

	uint8_t *dimgarr = NULL;	// Depth image array
	uint8_t *cimgarr = NULL;	// Color image array (I feel dirty using that spelling...)

	std::vector<char> recvBuff;
	std::vector<OCVSPacket> pktSendBuff;

	KinectInterface *kinect;
	OpenARScanner* scanner;

	bool initSuccess = false;
};