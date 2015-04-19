#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

typedef struct {
	std::vector<CvPoint> centres;
	std::vector<int> values;
	int count;
} ARMarkers;

class OpenARScanner
{
	public:
	static ARMarkers scanImage(IplImage* img);

	private:
	// Should be static, but c++ doesn't like that...
	static int    OpenARScanner::CV_AR_MARKER_SIZE;			// Marker decoding size = 160 * 160 Pixels
	static double OpenARScanner::CV_AR_DISP_SCALE_FIT;		// Distort (& Fit) the Display Image
	static double OpenARScanner::CV_AR_DISP_SCALE_DEF;		// Scale the Display Image

	static void   cv_adjustBox(int x, int y, CvPoint& A, CvPoint& B);
	static bool   cv_checkCorner(char* img_data, int img_width, int x, int y);
	static void   cv_updateCorner(CvPoint quad, CvPoint box, double& dist, CvPoint& corner);
	static void   cv_ARgetMarkerPoints(int points_total, CvPoint corners[10000], CvPoint START, CvPoint END, CvPoint& P, CvPoint& Q, CvPoint& R, CvPoint& S);
	static void   cv_ARgetMarkerPoints2(int points_total, CvPoint corners[10000], CvPoint START, CvPoint END, CvPoint& L, CvPoint& M, CvPoint& N, CvPoint& O);
	static void   cv_ARoutlineMarker(CvPoint Top, CvPoint Bottom, CvPoint A, CvPoint B, CvPoint C, CvPoint D, IplImage* raw_img);
	static void   cv_ARgetMarkerID_16b(IplImage* img, int& marker_id);
	static void   cv_ARgetMarkerNum(int marker_id, int& marker_num);
	static void   cv_ARaugmentImage(IplImage* display, IplImage* img, CvPoint2D32f srcQuad[4], double scale);
	static void   cv_lineEquation(CvPoint p1, CvPoint p2, double(&c)[3]);
	static double cv_distanceFormula(double c[], CvPoint p);
};
