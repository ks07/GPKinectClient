#pragma once
#include <Windows.h>
#include <Ole2.h>

#ifndef DISABLE_KINECT
#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>
#endif

#include <vector>
#include <stdint.h>

class KinectInterface
{
public:
	KinectInterface();
	~KinectInterface();

	void RunOpenCV(cv::Mat &gray, std::vector<cv::RotatedRect> &found);

	bool initKinect();

	// TODO: Decide what to do wrt debug viewing of Kinect input
	bool getKinectData(/*GLubyte* dest,*/ int *rawdest, uint8_t *scaled_dest, bool blocking);

	void filterArray(int *depthArray, int *filteredData);

	static const int width = 640;
	static const int height = 480;

	static const int depthMin = 923;
	static const int depthMax = 1307;
	static const int depthRange = depthMax - depthMin;

private:

#ifndef DISABLE_KINECT
	// Kinect variables
	HANDLE depthStream;
	HANDLE depthFrameEvent;
	INuiSensor* sensor = NULL;
#endif

	DWORD depthFrameTimeoutMillis = 15 * 1000;

	short frameCounter = 0;
};

