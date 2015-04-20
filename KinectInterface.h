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
#include <ostream>

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

		// The operator needs to be overloaded in the std::ostream namespace, thus marked as friend.
		friend std::ostream& operator<<(std::ostream &os, const BoxLimits &m) {
			return os << "{ low : " << (int)m.low << " high : " << (int)m.high << " }";
		}
	};

public:
	KinectInterface();

	KinectInterface(cv::Mat calib_src);

	~KinectInterface();

	void CalibrateDepth(cv::Mat &calib_src);

	void ProcessDepthImage(cv::Mat &raw, std::vector<cv::RotatedRect> &found, const bool debug_window = false);

	// Shows live depth input for calibration of box defs.
	void DebugCalibrationLoop();

	// Shows live depth input overlayed with detection for live debugging.
	void DebugInteractiveLoop(std::vector<cv::RotatedRect> &found);

	bool initKinect();

	bool GetWrappedData(cv::Mat &out, bool blocking = true, std::string fallback = "", uint8_t extraFrames = 3);

	void filterArray(int *depthArray, int *filteredData);

	static const int width = 640;
	static const int height = 480;

	static const int depthMin = 923;
	static const int depthMax = 1307;
	static const int depthRange = depthMax - depthMin;

	// Public variables for the slider callback in the calibration loop.
	static void TrackbarCallback(int value, void *kinectInstance);
	int dbg_lower_thresh = 0;
	int dbg_upper_thresh = 170;
	cv::Mat *dbg_bw_img = NULL;
	cv::Mat *dbg_src_img = NULL;

private:
	static const char * const CALIB_WINDOW_TITLE;

	static void RangeThreshold(cv::Mat &src, byte low, byte high, cv::Mat &dst);

	void ApplyCalibration(cv::Mat &src, cv::Mat &dest);

	int FindRectanglesInLayer(cv::Mat &bw, std::vector<cv::RotatedRect> &found, const bool debug_window = false);
	void DrawDetectedGeometry(const cv::Mat &base, cv::Mat &out, std::vector<cv::RotatedRect> &found);
	void DefineBoxes();
	std::vector<BoxLimits> boxes;

	// Byte array to store raw pixel values.
	uint8_t *imgarr = NULL;

#ifndef DISABLE_KINECT
	// Kinect variables
	HANDLE depthStream;
	HANDLE depthFrameEvent;
	INuiSensor* sensor = NULL;

	DWORD depthFrameTimeoutMillis = 15 * 1000;
#endif

	cv::Mat calibMask;

	void PreprocessDepthFrame(cv::Mat &raw, cv::Mat &out);
	void LayerPreprocessDepthFrame(cv::Mat &raw, cv::Mat &out, byte low, byte high);

	bool getKinectData(uint8_t *scaled_dest, bool blocking = true, int *rawdest = NULL);
};
