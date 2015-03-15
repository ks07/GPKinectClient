#pragma once
#include <Windows.h>
#include <Ole2.h>

#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>

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

	const int width = 640;
	const int height = 480;

private:

	// Kinect variables
	HANDLE depthStream;
	HANDLE depthFrameEvent;
	INuiSensor* sensor = NULL;

	DWORD depthFrameTimeoutMillis = 15 * 1000;

	short frameCounter = 0;
};

