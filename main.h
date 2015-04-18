#include <iostream>

//#include "OCVSlaveProtocol.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
	std::vector<CvPoint> centres;
	std::vector<int> values;
	int count;
} ARMarkers;

const int CV_AR_MARKER_SIZE = 160;				// Marker decoding size = 160 * 160 Pixels
const double CV_AR_DISP_SCALE_FIT = 0.0;		// Distort (& Fit) the Display Image
const double CV_AR_DISP_SCALE_DEF = 0.5;		// Scale the Display Image

ARMarkers openARLoop(int argc, char **argv);
bool cv_checkCorner(char* img_data, int img_width, int x, int y);		// Routine to check whether a particular pixel is an Edgel or not
void cv_adjustBox(int x, int y, CvPoint& A, CvPoint& B); 			// Routine to update Bounding Box corners
void cv_ARgetMarkerPoints(int points_total, CvPoint corners[10000], CvPoint START, CvPoint END, CvPoint& P, CvPoint& Q, CvPoint& R, CvPoint& S);// Routine to get 4-corners wrt (+)region partitioning
void cv_ARgetMarkerPoints2(int points_total, CvPoint corners[10000], CvPoint START, CvPoint END, CvPoint& L, CvPoint& M, CvPoint& N, CvPoint& O);// Routine to get 4-corners wrt (x)region partitioning
void cv_updateCorner(CvPoint quad, CvPoint box, double& dist, CvPoint& corner); 	// Distance algorithm for 4-corner validation
void cv_ARgetMarkerNum(int marker_id, int& marker_num);				// Routine to identify User specified number for Marker
void cv_ARgetMarkerID_16b(IplImage* img, int& marker_id);			// Routine to calculate Marker 16 bit ID
void cv_ARaugmentImage(IplImage* display, IplImage* img, CvPoint2D32f srcQuad[4], double scale = CV_AR_DISP_SCALE_DEF);	// Augment the display object on the Raw image
void cv_ARoutlineMarker(CvPoint Top, CvPoint Bottom, CvPoint A, CvPoint B, CvPoint C, CvPoint D, IplImage* raw_img); 	// Routine to Mark and draw line on identified Marker Boundaries
void cv_lineEquation(CvPoint p1, CvPoint p2, double(&c)[3]);		// Equation of the line ax+by+c=0; a=c[0], b=c[1], c=c[2] for (x)region 4-corner detection
double cv_distanceFormula(double c[3], CvPoint p);					// Perpendicular distance of a point wrt a line ax+by+c=0; will be +ve or -ve depending upon position of point wrt line
