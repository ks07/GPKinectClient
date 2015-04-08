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
	// TODO: We might want to include some geometry information, e.g. a min area.
	struct BoxLimits
	{
		BoxLimits(uint8_t low, uint8_t high)
			: low(low)
			, high(high)
		{}
		uint8_t low;
		uint8_t high;
	};

public:
	KinectInterface();

	KinectInterface(cv::Mat calib_src);

	~KinectInterface();

	void CalibrateDepth(cv::Mat &calib_src);

	static void RangeThreshold(cv::Mat &src, byte low, byte high, cv::Mat &dst);
	static void TrackbarCallback(int value, void *data);

	void ProcessDepthImage(cv::Mat &raw, std::vector<cv::RotatedRect> &found, const bool debug_window = false);

	int FindRectanglesInLayer(cv::Mat &bw, std::vector<cv::RotatedRect> &found, const bool debug_window = false);

	void DebugLoop();

	bool initKinect();

	bool GetWrappedData(cv::Mat &out, bool blocking = true, std::string fallback = "");

	void filterArray(int *depthArray, int *filteredData);

	void ApplyCalibration(cv::Mat &src, cv::Mat &dest);

	static const int width = 640;
	static const int height = 480;

	static const int depthMin = 923;
	static const int depthMax = 1307;
	static const int depthRange = depthMax - depthMin;

	int dbg_lower_thresh = 0;
	int dbg_upper_thresh = 170;
	cv::Mat *dbg_bw_img = NULL;
	cv::Mat *dbg_src_img = NULL;

private:
	void DefineBoxes();
	std::vector<BoxLimits> boxes;

	// Byte array to store raw pixel values.
	uint8_t *imgarr = NULL;

#ifndef DISABLE_KINECT
	// Kinect variables
	HANDLE depthStream;
	HANDLE depthFrameEvent;
	INuiSensor* sensor = NULL;
#endif

	DWORD depthFrameTimeoutMillis = 15 * 1000;

	short frameCounter = 0;

	cv::Mat calibMask;

	void PreprocessDepthFrame(cv::Mat &raw, cv::Mat &out);

	// TODO: Decide what to do wrt debug viewing of Kinect input
	bool getKinectData(/*GLubyte* dest,*/ uint8_t *scaled_dest, bool blocking = true, int *rawdest = NULL);
};

