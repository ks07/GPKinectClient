#include <opencv2/opencv.hpp>

#include "KinectInterface.h"

const char * const KinectInterface::CALIB_WINDOW_TITLE = "Calibration";

KinectInterface::KinectInterface()
	: calibMask(width * height, CV_16SC1, 0)
	, imgarr((uint8_t *)calloc(KinectInterface::width * KinectInterface::height, sizeof(uint8_t)))
{
	DefineBoxes();
}


KinectInterface::KinectInterface(cv::Mat calib_src)
	: imgarr((uint8_t *)calloc(KinectInterface::width * KinectInterface::height, sizeof(uint8_t)))
{
	DefineBoxes();
	CalibrateDepth(calib_src);
}

void KinectInterface::DefineBoxes()
{
	boxes.emplace_back(12, 30); // Biggie C Box (tall, large near-square)
	boxes.emplace_back(60, 74); // Pinkie hole box (tall, narrow rectangle, covered side)
	boxes.emplace_back(86, 97); // Vans box (narrow rectangle)
	boxes.emplace_back(95, 107); // Flav box (shortened shoebox) IFFY!!!
	boxes.emplace_back(103, 117); // Denim box (open bottom shoebox)
	boxes.emplace_back(123, 137); // 100 box (flat 16:9-ish box)
	boxes.emplace_back(146, 158); // IC book
	boxes.emplace_back(157, 173); // Wood
}


KinectInterface::~KinectInterface()
{
#ifndef DISABLE_KINECT
	if (sensor != NULL) {
		sensor->Release();
		sensor = NULL;
	}
#endif
	free(imgarr);
}


bool KinectInterface::initKinect() {
#ifndef DISABLE_KINECT
	// Get a working kinect sensor
	int numSensors;
	if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1) return false;
	if (NuiCreateSensorByIndex(0, &sensor) < 0) return false;

	// Initialize sensor
	HRESULT res = sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
	if (res != S_OK) {
		sensor->Release();
		sensor = NULL;
		return false;
	}
	depthFrameEvent = CreateEvent(NULL, true, false, NULL);
	res = sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, // Depth camera or rgb camera?
		NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
		0,         // Image stream flags, e.g. near mode
		2,        // Number of frames to buffer
		depthFrameEvent,     // Event handle
		&depthStream);
	return (res == S_OK);
#else
	return false;
#endif
}


bool KinectInterface::getKinectData(uint8_t *scaled_dest, bool blocking, int *rawdest) {
#ifndef DISABLE_KINECT
	NUI_IMAGE_FRAME imageFrame;
	NUI_LOCKED_RECT LockedRect;

	if (blocking) {
		WaitForSingleObject(depthFrameEvent, depthFrameTimeoutMillis);
	}

	if (sensor == NULL || sensor->NuiImageStreamGetNextFrame(depthStream, 0, &imageFrame) < 0) return false;
	INuiFrameTexture* texture = imageFrame.pFrameTexture;
	texture->LockRect(0, &LockedRect, NULL, 0);

	if (LockedRect.Pitch != 0) {
		bool first = true;
		const USHORT* curr = (const USHORT*)LockedRect.pBits;
		const USHORT* dataEnd = curr + (width*height);

		while (curr < dataEnd) {
			// Get depth in millimeters
			USHORT depth = NuiDepthPixelToDepth(*curr++);

			if (rawdest != NULL) {
				*rawdest++ = depth;
			}

			// TODO: Don't filter here!!!
			if (depth < depthMin || depth >= depthMax) {
				first ? *scaled_dest++ = (uint8_t)128 : *scaled_dest++ = *(scaled_dest - 1);
			}
			else {
				// Scale to predefined range (max is 800 - 4000, we only want a subset due to Kinect positioning)
				*scaled_dest++ = (uint8_t)(((float)(depth - depthMin) / float(depthRange)) * 256.0);
			}

			first = false;
		}
	}

	texture->UnlockRect(0);
	sensor->NuiImageStreamReleaseFrame(depthStream, &imageFrame);
	return true;
#else
	return false;
#endif
}


bool KinectInterface::GetWrappedData(cv::Mat &out, bool blocking, std::string fallback, uint8_t extraFrames) {
	static int *rawarr[] = { (int *)malloc(width*height*sizeof(int)), (int *)malloc(width*height*sizeof(int)), (int *)malloc(width*height*sizeof(int)) };
	if (getKinectData(imgarr, blocking, rawarr[0]))
	{
		if (!blocking) {
			cv::Mat input(480, 640, CV_8UC1, imgarr);

			// X and Y are inverted in the game world, so we should tranpose here
			cv::transpose(input, out);
			input.release();

			return true;
		}
		else {
			for (uint8_t i = 1; i < extraFrames; ++i) {
				getKinectData(imgarr, blocking, rawarr[i]);
			}
			for (size_t i = 0; i < 640 * 480; ++i) {
				int sum = 0;
				int cnt = 0;

				for (uint8_t f = 0; f < extraFrames; ++f) {
					int dat = rawarr[f][i];
					if (dat >= depthMin && dat <= depthMax) {
						sum += dat;
						cnt++;
					}
				}

				if (cnt != 0) {
					// If none of the pixels are valid, fallback to the filtered input, else average.
					float avg = (float)sum / cnt;
					imgarr[i] = (uint8_t)(((avg - depthMin) / depthRange) * 256.0);
				}
			}
			cv::Mat input(480, 640, CV_8UC1, imgarr);

			// X and Y are inverted in the game world, so we should tranpose here
			cv::transpose(input, out);
			input.release();

			return true;
		}
	}
	else if (fallback != "")
	{
		std::cerr << "[ERROR] Falling back to fixed image input!" << std::endl;
		cv::Mat src = cv::imread(fallback);
		// Convert to grayscale, assuming input image is a standard color image
		cv::Mat input;
		cv::cvtColor(src, input, cv::COLOR_BGR2GRAY);
		src.release();

		// X and Y are inverted in the game world, so we should tranpose here if the input image is not already
		if (input.size().height == height && input.size().width == width) {
			cv::transpose(input, out);
			input.release();
		}
		else {
			out = input;
			input.release();
		}

		// Again, the image should be transposed.
		assert(out.size().height == width && out.size().width == height);
		return false;
	}

	return false;
}


// Filter out 0's to the mode of the surrounding pixels
void KinectInterface::filterArray(int *depthArray, int *filteredData)
{
	int widthBound = width - 1;
	int heightBound = height - 1;
	for (int depthArrayRowIndex = 0; depthArrayRowIndex < height; depthArrayRowIndex++)
	{
		for (int depthArrayColumnIndex = 0; depthArrayColumnIndex < width; depthArrayColumnIndex++)
		{
			int depthIndex = depthArrayColumnIndex + (depthArrayRowIndex * width);
			if (depthArray[depthIndex] == 0)
			{
				int x = depthIndex % width;
				int y = (depthIndex - x) / width;
				int filterCollection[24][2];
				for (int i = 0; i < 24; i++)
				{
					filterCollection[i][0] = 0;
					filterCollection[i][1] = 0;
				}

				int innerBandCount = 0;
				int outerBandCount = 0;

				for (int yi = -2; yi < 3; yi++)
				{
					for (int xi = -2; xi < 3; xi++)
					{
						if (xi != 0 || yi != 0)
						{
							int xSearch = x + xi;
							int ySearch = y + yi;

							if (xSearch >= 0 && xSearch <= widthBound && ySearch >= 0 && ySearch <= heightBound)
							{
								int index = xSearch + (ySearch * width);
								if (depthArray[index] != 0)
								{
									for (int i = 0; i < 24; i++)
									{
										if (filterCollection[i][0] == depthArray[index])
										{
											filterCollection[i][1]++;
											break;
										}
										else if (filterCollection[i][0] == 0)
										{
											filterCollection[i][0] = depthArray[index];
											filterCollection[i][1]++;
											break;
										}
									}

									if (yi != -2 && yi != 2 && xi != -2 && xi != 2)
									{
										innerBandCount++;
									}
									else {
										outerBandCount++;
									}
								}
							}
						}
					}
				}
				short depth = 0;
				if (innerBandCount >= 3 || outerBandCount >= 6)
				{
					short frequency = 0;
					for (int i = 0; i < 24; i++)
					{
						if (filterCollection[i][0] == 0)
						{
							break;
						}
						if (filterCollection[i][1] > frequency)
						{
							depth = filterCollection[i][0];
							frequency = filterCollection[i][1];
						}
					}
				}
				filteredData[depthIndex] = depth;
			}
			else {
				filteredData[depthIndex] = depthArray[depthIndex];
			}
		}
	}
}

// Capture an image of the empty play area, and use this to create a mask that we can apply whenever a frame is processed.
void KinectInterface::CalibrateDepth(cv::Mat &calib_src) {
	try {
		// Use fixed approx floor value as the target for calibration.
		auto correct = 180;
		cv::Mat meanMat(calib_src.size(), CV_8UC1, correct);

		// Compute the mask as a simple difference operation. TODO: Is this linear scaling appropriate? Should it be non-linear somehow?
		cv::subtract(meanMat, calib_src, calibMask, cv::noArray(), CV_16SC1);
	}
	catch (cv::Exception &e)
	{
		std::cout << e.what() << std::endl;
	}
}

void KinectInterface::ApplyCalibration(cv::Mat &src, cv::Mat &dest) {
	cv::add(src, calibMask, dest, cv::noArray(), CV_8UC1);
}

void KinectInterface::RangeThreshold(cv::Mat &src, byte low, byte high, cv::Mat &dst) {
	cv::Mat upperBound(src.size(), CV_8UC1, high);
	cv::Mat lowerBound(src.size(), CV_8UC1, low);
	cv::inRange(src, lowerBound, upperBound, dst);
}

// Applies preprocessing to a depth frame to prepare it for box detection. Currently blurs and calibrates.
void KinectInterface::PreprocessDepthFrame(cv::Mat &raw, cv::Mat &out) {
	cv::Mat blurred;
	// As it turns out, the blur operation is far more effective than just filtering erroneous values
	// TODO: compare to/combine with averaging frames and other blur methods/sizes, and windowed filtering?
	cv::medianBlur(raw, blurred, 5);
	ApplyCalibration(blurred, out);
}

// Applies stage 2 of depth frame preprocessing, creating a thresholded layer. Masks (fixed) bases.
void KinectInterface::LayerPreprocessDepthFrame(cv::Mat &src, cv::Mat &out, byte low, byte high) {
	// Convert to binary image using range threshold.
	RangeThreshold(src, low, high, out);

	// Block the corner zones that we are using as bases
	cv::rectangle(out, cv::Rect(0, 0, 24, 32), 0, CV_FILLED);
	cv::rectangle(out, cv::Rect(out.size().width - 24, out.size().height - 32, 24, 32), 0, CV_FILLED);
}

void KinectInterface::TrackbarCallback(int value, void *kinectInstance) {
	if (kinectInstance != NULL) {
		KinectInterface *owner = (KinectInterface *)kinectInstance;

		// Need to ensure lower <= upper, and that the user's currently selected slider is moved!
		bool sliding_lower = (value == owner->dbg_lower_thresh);

		if (owner->dbg_lower_thresh > owner->dbg_upper_thresh) {
			if (sliding_lower) {
				owner->dbg_upper_thresh = owner->dbg_lower_thresh;
			}
			else {
				owner->dbg_lower_thresh = owner->dbg_upper_thresh;
			}
		}

		if (owner->dbg_src_img != NULL && owner->dbg_bw_img != NULL) {
			owner->LayerPreprocessDepthFrame(*owner->dbg_src_img, *owner->dbg_bw_img, owner->dbg_lower_thresh, owner->dbg_upper_thresh);
			cv::imshow(CALIB_WINDOW_TITLE, *owner->dbg_bw_img);
		}
	}
}


void KinectInterface::DebugCalibrationLoop() {
	bool run = true;
	DCLmode dispMode = DCLmode::FILTERED;
	size_t selectedBox = 0;

	while (run) {
		cv::Mat bw;
		cv::Mat raw;
		GetWrappedData(raw, true, "closebox.png");

		// Apply some preprocessing steps to the input frame.
		cv::Mat srcb;
		PreprocessDepthFrame(raw, srcb);
		LayerPreprocessDepthFrame(srcb, bw, dbg_lower_thresh, dbg_upper_thresh);
		
		switch (dispMode)
		{
		case KinectInterface::DCLmode::RAW:
			cv::putText(raw, "RAW", cv::Point(0, width - 5), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255));
			cv::imshow(CALIB_WINDOW_TITLE, raw);
			break;
		case KinectInterface::DCLmode::PREPROCESSED:
			cv::putText(srcb, "PRE", cv::Point(0, width - 5), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255));
			cv::imshow(CALIB_WINDOW_TITLE, srcb);
			break;
		case KinectInterface::DCLmode::FILTERED:
		default:
			cv::imshow(CALIB_WINDOW_TITLE, bw);
			break;
		}

		// TODO: We really want to avoid these if possible.
		dbg_bw_img = &bw;
		dbg_src_img = &srcb;

		const int sliderMax = 255;
		cv::createTrackbar("lowbar", CALIB_WINDOW_TITLE, &dbg_lower_thresh, sliderMax, &KinectInterface::TrackbarCallback, (void *)this);
		cv::createTrackbar("highbar", CALIB_WINDOW_TITLE, &dbg_upper_thresh, sliderMax, &KinectInterface::TrackbarCallback, (void *)this);

		int keyPressed = cv::waitKey(10);

		if (keyPressed == 'j') {
			// j => adjust both scrollbars down/left
			if (dbg_lower_thresh == 0) {
				// If the lower thresh hits 0, wraparound both sliders to the top.
				int diff = dbg_upper_thresh - dbg_lower_thresh;
				dbg_upper_thresh = sliderMax;
				dbg_lower_thresh = sliderMax - diff;
			}
			else {
				dbg_lower_thresh--;
				dbg_upper_thresh--;
			}
		}
		else if (keyPressed == 'k') {
			// j => adjust both scrollbars down/left
			if (dbg_upper_thresh == sliderMax) {
				int diff = dbg_upper_thresh - dbg_lower_thresh;
				dbg_lower_thresh = 0;
				dbg_upper_thresh = diff;
			}
			else {
				dbg_lower_thresh++;
				dbg_upper_thresh++;
			}
		}
		else if (keyPressed == 'c') {
			// c => recalibrate sensor
			cv::Mat calib_src;
			if (GetWrappedData(calib_src, true)) {
				cv::imshow("Calibration Image", calib_src);
				cv::waitKey();
				cv::destroyWindow("Calibration Image");
				CalibrateDepth(calib_src);
			}
		}
		else if (keyPressed == 'n') {
			// n# => Switch to preset box def.
			int nKey = cv::waitKey(30 * 1000);
			if (nKey >= '0' && nKey <= '9') {
				unsigned int n = nKey - '0';
				if (n < boxes.size()) {
					BoxLimits sel = boxes.at(n);
					dbg_lower_thresh = sel.low;
					dbg_upper_thresh = sel.high;
					selectedBox = n;
				}
			}
		}
		else if (keyPressed == 't') {
			// t => Test
			if (!bw.empty()) {
				// Ensure we don't try passing a failed input for whatever reason
				std::vector<cv::RotatedRect> found;
				FindRectanglesInLayer(bw, found, boxes.at(selectedBox), true);
				std::cout << "Found" << found.size() << std::endl;
			}
		}
		else if (keyPressed == 'q') {
			// q => Quit
			run = false;
		}
		else if (keyPressed == 'm') {
			// m => cycle input mode
			switch (dispMode)
			{
			case KinectInterface::DCLmode::FILTERED:
				dispMode = DCLmode::RAW;
				break;
			case KinectInterface::DCLmode::RAW:
				dispMode = DCLmode::PREPROCESSED;
				break;
			case KinectInterface::DCLmode::PREPROCESSED:
			default:
				dispMode = DCLmode::FILTERED;
				break;
			}
		}
		else if (keyPressed == 's') {
			// s => save box def
			boxes.emplace_back(dbg_lower_thresh, dbg_upper_thresh);
			std::cout << "Added new box def #" << boxes.size() - 1 << " = " << boxes.back();
		}
	}

	// Make sure all windows are closed.
	cv::destroyAllWindows();
}


void KinectInterface::DebugInteractiveLoop(std::vector<cv::RotatedRect> &found) {
	bool contLoop = true;

	cv::Mat disp;

	while (contLoop) {
		cv::Mat raw;
		GetWrappedData(raw, true, "closebox.png");

		ProcessDepthImage(raw, found, false);
		DrawDetectedGeometry(raw, disp, found);

		cv::imshow("test", disp);

		int keyPressed = cv::waitKey(10);

		// TODO: Add a stepping mode?
		if (keyPressed == 'r') {
			// r => recalibrate sensor
			cv::Mat calib_src;
			if (GetWrappedData(calib_src, true)) {
				CalibrateDepth(calib_src);
			}
		}
		else if (keyPressed == 'a') {
			// a => Accept current input
			contLoop = false;
		}

		raw.release();
	}

	// Make sure all windows are closed.
	cv::destroyAllWindows();
	disp.release();
}


void KinectInterface::DrawDetectedGeometry(const cv::Mat &base, cv::Mat &out, std::vector<cv::RotatedRect> &found) {
	// Display all the found geometries over the original image.
	cv::cvtColor(base, out, cv::COLOR_GRAY2BGR);

	// Colour to draw the boxes in.
	const cv::Scalar yellow = cv::Scalar(100, 255, 255);

	// Use the min area bounding rectangle to get us a quick approx that we can use. TODO: This is not ideal in the slightest if our bounding contour is off... we should check them!
	for (std::vector<cv::RotatedRect>::iterator rectit = found.begin(); rectit != found.end(); ++rectit)
	{
		cv::Point2f vertices[4]; // The mind boggles why OpenCV doesn't have a function to draw it's own shapes...
		rectit->points(vertices);
		for (int i = 0; i < 4; i++) {
			cv::line(out, vertices[i], vertices[(i + 1) % 4], yellow);
		}
	}
}


// TODO: Rename to frame?
void KinectInterface::ProcessDepthImage(cv::Mat &raw, std::vector<cv::RotatedRect> &found, const bool debug_window) {
	// We must be given a greyscale input.
	assert(raw.type() == CV_8UC1);

	// Clear out found before we start filling it, allows us to reuse the same one.
	found.clear();

	// Apply some preprocessing steps to the input frame.
	cv::Mat src;
	PreprocessDepthFrame(raw, src);

	// Need to split the image into layers and process each layer for rectangles. Currently, we expect only a single rectangle in each layer (TODO: see issue #9)
	for (std::vector<BoxLimits>::iterator boxit = boxes.begin(); boxit != boxes.end(); ++boxit) {
		cv::Mat bw;

		// Perform the binary conversion.
		LayerPreprocessDepthFrame(src, bw, boxit->low, boxit->high);

		// Run the OpenCV detection on this thresholded layer.
		int layerCnt = FindRectanglesInLayer(bw, found, *boxit, debug_window);

		if (debug_window) {
			std::cout << "Layer added " << layerCnt << " objects" << std::endl;
		}
	}

	if (debug_window) {
		std::cout << "Final object count: " << found.size() << std::endl;

		cv::Mat outputdisp;
		// Display all the found geometries over the original image.
		DrawDetectedGeometry(raw, outputdisp, found);
		cv::imshow("Combined Layer Output", outputdisp);
		cv::waitKey();
		cv::destroyAllWindows();
	}
}

bool KinectInterface::AreaApproxEqual(float measured, float defined) {
	const float factor = 0.1;
	const float areafactor = (1.0 + factor) * (1.0 + factor) - 1.0;

	float smaller = MIN(measured, defined);
	float larger = MAX(measured, defined);

	float diff = larger - smaller;
	return diff <= larger * areafactor;
}

int KinectInterface::FindRectanglesInLayer(cv::Mat &bw, std::vector<cv::RotatedRect> &found, BoxLimits targetBox, const bool debug_window) {

	int foundStartSize = found.size();

	cv::Mat contourImg = bw.clone(); // TODO: Possibly unnecessary
	std::vector<std::vector<cv::Point>> contoursFound;
	std::vector<std::vector<cv::Point>> approxFakeContours; // TODO: Rename
	std::vector<cv::Vec4i> heirarchy; // Unused when in RETR_EXTERNAL mode

	cv::findContours(contourImg, contoursFound, heirarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_KCOS);

	//if (debug_window) {
	cv::Mat contourImage(bw.size(), CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Scalar colors[3];
	colors[0] = cv::Scalar(255, 0, 0);
	colors[1] = cv::Scalar(0, 255, 0);
	colors[2] = cv::Scalar(0, 0, 255);
	//}

	for (size_t idx = 0; idx < contoursFound.size(); idx++) {
		if (debug_window) {
			std::cout << contoursFound.at(idx).size() << std::endl;
			cv::drawContours(contourImage, contoursFound, idx, colors[idx % 3]);
		}

		// Approximate a closed poly from contours
		std::vector<cv::Point> approxFound;
		cv::approxPolyDP(cv::Mat(contoursFound.at(idx)), approxFound, 20, true);

		// Stick this closed poly into the list of them
		approxFakeContours.push_back(std::vector<cv::Point>(approxFound));
	}

	if (debug_window) {
		cv::Mat approxImage(bw.size(), CV_8UC3, cv::Scalar(0, 0, 0));

		// Draw the contours onto a black background in cycling colours.
		for (size_t idx = 0; idx < approxFakeContours.size(); idx++)
		{
			cv::drawContours(approxImage, approxFakeContours, idx, colors[idx % 3]);
		}

		cv::imshow("test", contourImage);
		cv::waitKey();
		cv::imshow("test", approxImage);
		cv::waitKey();
	}

	// TODO: This needs to come out.
	cv::Mat outputdisp; // Uninit'ed if we aren't in debug
	if (debug_window) {
		cv::cvtColor(bw, outputdisp, cv::COLOR_GRAY2BGR);
	}

	// Use the min area bounding rectangle to get us a quick approx that we can use. TODO: This is not ideal in the slightest if our bounding contour is off... we should check them!
	for (size_t idx = 0; idx < approxFakeContours.size(); idx++)
	{
		// Only look at contours with 4 corners
		if (approxFakeContours.at(idx).size() == 4)
		{
			if (AreaApproxEqual(cv::contourArea(approxFakeContours.at(idx)), targetBox.area)) {
				cv::RotatedRect box = cv::minAreaRect(approxFakeContours.at(idx));
				found.push_back(box);

				if (debug_window) {
					cv::Scalar yellow = cv::Scalar(100, 255, 255);
					cv::Point2f vertices[4]; // The mind boggles why OpenCV doesn't have a function to draw it's own shapes...
					box.points(vertices);
					for (int i = 0; i < 4; i++) {
						cv::line(outputdisp, vertices[i], vertices[(i + 1) % 4], yellow);
					}
				}
			}
		}
	}

	if (debug_window) {
		cv::imshow("test bbox", outputdisp);
		cv::waitKey();
		cv::destroyAllWindows();
	}

	// Return the number of geometries added by this call.
	return found.size() - foundStartSize;
}
