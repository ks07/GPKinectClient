#include <iostream>

#include "OCVSlaveProtocol.h"

int openARLoop(CvCapture* capture,
				IplImage* img,
				IplImage* marker_transposed_img,
				IplImage* gray,
				IplImage* thres,
				IplImage* prcs_flg,
				IplImage* display_img1,
				CvMat*     warp_matrix);
int openARSetup(CvCapture* &capture,
				IplImage*  &img,
				IplImage*  &marker_transposed_img,
				IplImage*  &gray,
				IplImage*  &thres,
				IplImage*  &prcs_flg,
				IplImage*  &display_img1,
				CvMat*     &warp_matrix);
void openARCleanup(CvCapture* capture,
 					IplImage*  img,
					IplImage*  marker_transposed_img,
					IplImage*  gray,
					IplImage*  thres,
					IplImage*  prcs_flg,
					IplImage*  display_img1,
					CvMat*     warp_matrix);
void cv_adjustBox(int x, int y, CvPoint& A, CvPoint& B);
bool cv_checkCorner(char* img_data, int img_width, int x, int y);
void cv_updateCorner(CvPoint quad, CvPoint box, double& dist, CvPoint& corner);
void cv_ARgetMarkerPoints(int points_total, CvPoint corners[10000], CvPoint START, CvPoint END, CvPoint& P, CvPoint& Q, CvPoint& R, CvPoint& S);
void cv_ARgetMarkerPoints2(int points_total, CvPoint corners[10000], CvPoint START, CvPoint END, CvPoint& L, CvPoint& M, CvPoint& N, CvPoint& O);
void cv_ARoutlineMarker(CvPoint Top, CvPoint Bottom, CvPoint A, CvPoint B, CvPoint C, CvPoint D, IplImage* raw_img);
void cv_ARgetMarkerID_16b(IplImage* img, int& marker_id);
void cv_ARgetMarkerNum(int marker_id, int& marker_num);
void cv_ARaugmentImage(IplImage* display, IplImage* img, CvPoint2D32f srcQuad[4], double scale);
void cv_lineEquation(CvPoint p1, CvPoint p2, double(&c)[3]);
double cv_distanceFormula(double c[], CvPoint p);
