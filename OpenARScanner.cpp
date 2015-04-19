#include "OpenARScanner.h"

OpenARScanner::OpenARScanner()
{

}


OpenARScanner::~OpenARScanner()
{

}




/*
-----------------------------------------------------------------------------------------------------------
OpenAR - OpenCV Augmented Reality
-----------------------------------------------------------------------------------------------------------
OpenAR is a very simple C++ implementation to achieve Marker based Augmented Reality. OpenAR is based on
OpenCV and solely dependent on the library. OpenAR decodes markers in a frame of image. OpenAR does not
implement Marker tracking across frames. Also OpenAR does not implement Template matching for Marker decoding.

For more information, Visit http://dsynflo.blogspot.com

License
---------------
OpenAR is under Zero License
Zero License is a license meant for free distribution in public domain.
Zero License allows commercial use of materials with or without permissions/modifications/customizations
Zero License allows use with or without mention of source or contributors.


Release History
---------------
Epoch		: March 20, 2010, Initial Release
Authors		: Aditya KP & Bharath Prabhuswamy @ Virtual Logic Systems Pvt. Ltd., Bangalore

Date		: June 20, 2014, Re-release
Authors		: Bharath Prabhuswamy


DISCLAIMER
---------------
THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

-----------------------------------------------------------------------------------------------------------*/

//______________________________________________________________________________________
// Program : OpenAR - OpenCV based 2D Augmented Reality Program
// Author  : Bharath Prabhuswamy
//______________________________________________________________________________________

ARMarkers OpenARScanner::scanImage(IplImage* img)
{
	ARMarkers markers;
	markers.count = 0;

	// List of Images to be augmented on the Marker

	return markers;
}

// Routines used in Main loops

// Routine to update Bounding Box corners with farthest corners in that Box
void OpenARScanner::cv_adjustBox(int x, int y, CvPoint& A, CvPoint& B)
{
	if (x < A.x)
		A.x = x;

	if (y < A.y)
		A.y = y;

	if (x > B.x)
		B.x = x;

	if (y > B.y)
		B.y = y;
}

// Routine to check whether a particular pixel is an Edgel or not
bool OpenARScanner::cv_checkCorner(char* img_data, int img_width, int x, int y)
{
	const int wind_sz = 5;
	int wind_bnd = (wind_sz - 1) / 2;
	int sum = 0;
	bool result = false;
	uchar* ptr[wind_sz];
	int index = 0;

	for (int k = (0 - wind_bnd); k <= wind_bnd; ++k)
	{
		ptr[index] = (uchar*)(img_data + (y + k) *  img_width);
		index = index + 1;
	}

	for (int i = 0; i <= (wind_sz - 1); ++i)
	{
		if ((i == 0) || (i == (wind_sz - 1)))
		{
			for (int j = (0 - wind_bnd); j <= wind_bnd; ++j)
			{
				if (ptr[i][x + j] == 0)
					sum += 1;
				else
					continue;
			}
		}
		else
		{
			if (ptr[i][x - wind_bnd] == 0)
				sum += 1;
			else
				continue;

			if (ptr[i][x + wind_bnd] == 0)
				sum += 1;
			else
				continue;
		}
	}

	if ((sum >= 4) && (sum <= 12))
	{
		result = true;
	}
	return result;
}

// Distance algorithm for 4-corner validation
void OpenARScanner::cv_updateCorner(CvPoint quad, CvPoint box, double& dist, CvPoint& corner)
{
	double temp_dist;
	temp_dist = sqrt(((box.x - quad.x) * (box.x - quad.x)) + ((box.y - quad.y) * (box.y - quad.y)));

	if (temp_dist > dist)
	{
		dist = temp_dist;
		corner = quad;
	}
}

// Routine to calculate 4 Corners of the Marker in Image Space using (+) Region partitioning
void OpenARScanner::cv_ARgetMarkerPoints(int points_total, CvPoint corners[10000], CvPoint START, CvPoint END, CvPoint& P, CvPoint& Q, CvPoint& R, CvPoint& S)
{
	CvPoint A = START;
	CvPoint B;
	B.x = END.x;
	B.y = START.y;

	CvPoint C = END;
	CvPoint D;
	D.x = START.x;
	D.y = END.y;

	int halfx = (A.x + B.x) / 2;
	int halfy = (A.y + D.y) / 2;


	double dmax[4];
	dmax[0] = 0.0;
	dmax[1] = 0.0;
	dmax[2] = 0.0;
	dmax[3] = 0.0;

	for (int i = 0; i < points_total; ++i)
	{
		if ((corners[i].x < halfx) && (corners[i].y <= halfy))
		{
			cv_updateCorner(corners[i], C, dmax[2], P);
		}
		else if ((corners[i].x >= halfx) && (corners[i].y < halfy))
		{
			cv_updateCorner(corners[i], D, dmax[3], Q);
		}
		else if ((corners[i].x > halfx) && (corners[i].y >= halfy))
		{
			cv_updateCorner(corners[i], A, dmax[0], R);
		}
		else if ((corners[i].x <= halfx) && (corners[i].y > halfy))
		{
			cv_updateCorner(corners[i], B, dmax[1], S);
		}
	}

}

// Routine to calculate 4 Corners of the Marker in Image Space using (x) Region partitioning
void OpenARScanner::cv_ARgetMarkerPoints2(int points_total, CvPoint corners[10000], CvPoint START, CvPoint END, CvPoint& L, CvPoint& M, CvPoint& N, CvPoint& O)
{
	CvPoint A = START;
	CvPoint B;
	B.x = END.x;
	B.y = START.y;
	CvPoint C = END;
	CvPoint D;
	D.x = START.x;
	D.y = END.y;

	CvPoint W, X, Y, Z;

	double line1[3], line2[3];
	double pd1 = 0.0;
	double pd2 = 0.0;

	W.x = (START.x + END.x) / 2;
	W.y = START.y;

	X.x = END.x;
	X.y = (START.y + END.y) / 2;

	Y.x = (START.x + END.x) / 2;
	Y.y = END.y;

	Z.x = START.x;
	Z.y = (START.y + END.y) / 2;

	cv_lineEquation(C, A, line1);
	cv_lineEquation(B, D, line2);

	double rdmax[4];
	rdmax[0] = 0.0;
	rdmax[1] = 0.0;
	rdmax[2] = 0.0;
	rdmax[3] = 0.0;

	for (int i = 0; i < points_total; ++i)
	{
		pd1 = cv_distanceFormula(line1, corners[i]);
		pd2 = cv_distanceFormula(line2, corners[i]);

		if ((pd1 >= 0.0) && (pd2 > 0.0))
		{
			cv_updateCorner(corners[i], W, rdmax[2], L);
		}
		else if ((pd1 > 0.0) && (pd2 <= 0.0))
		{
			cv_updateCorner(corners[i], X, rdmax[3], M);
		}
		else if ((pd1 <= 0.0) && (pd2 < 0.0))
		{
			cv_updateCorner(corners[i], Y, rdmax[0], N);
		}
		else if ((pd1 < 0.0) && (pd2 >= 0.0))
		{
			cv_updateCorner(corners[i], Z, rdmax[1], O);
		}
		else
			continue;
	}
}


void OpenARScanner::cv_ARoutlineMarker(CvPoint Top, CvPoint Bottom, CvPoint A, CvPoint B, CvPoint C, CvPoint D, IplImage* raw_img)
{
	cvRectangle(raw_img, Top, Bottom, CV_RGB(255, 0, 0), 1);	// Draw untransfromed/flat rectangle on Bounding Box with Marker

	cvLine(raw_img, A, B, CV_RGB(255, 0, 0), 2);		// Draw rectangle by joining 4 corners of the Marker via CVLINE
	cvLine(raw_img, B, C, CV_RGB(0, 255, 0), 2);
	cvLine(raw_img, C, D, CV_RGB(0, 0, 255), 2);
	cvLine(raw_img, D, A, CV_RGB(255, 255, 0), 2);

	cvCircle(raw_img, A, 4, CV_RGB(128, 255, 128), 1, 8);		// Mark 4 corners of the Marker in Green		
	cvCircle(raw_img, B, 4, CV_RGB(128, 255, 128), 1, 8);
	cvCircle(raw_img, C, 4, CV_RGB(128, 255, 128), 1, 8);
	cvCircle(raw_img, D, 4, CV_RGB(128, 255, 128), 1, 8);
}


// Routine to calculate Binary Marker Idendifier 
void OpenARScanner::cv_ARgetMarkerID_16b(IplImage* img, int& marker_id)
{
	// Black is 1/True and White is 0/False for following Binary calculation

	int i = 0;
	int value = 0;
	for (int y = 50; y<120; y = y + 20)
	{
		uchar* pointer_scanline = (uchar*)(img->imageData + (y - 1)*img->width);

		for (int x = 50; x<120; x = x + 20)
		{
			if (pointer_scanline[x - 1] == 0)
			{
				value += (int)pow(2, i); 		// Covert to decimal by totaling 2 raised to power of 'Bitmap position'
			}
			i++;
		}
	}
	marker_id = value;
}

void OpenARScanner::cv_ARgetMarkerNum(int marker_id, int& marker_num)
{
	switch (marker_id)
	{
		case 0xE41B:
		case 0x39C9:
		case 0xD827:
		case 0x939C:
			marker_num = 1;
			break;
		case 63729:
		case 47790:
		case 36639:
		case 30045:
			marker_num = 2;
			break;
		case 0xE700:
		case 0x1332:
		case 0x00E7:
		case 0X4CC8:
			marker_num = 3;
			break;
		case 0x9E1F:
		case 0xBAAD:
		case 0xF879:
		case 0xB55D:
			marker_num = 4;
			break;
		case 0xF559:
		case 0xF8E9:
		case 0x9AAF:
		case 0x971F:
			marker_num = 5;
			break;
		case 0xC800:		//So it turns out it *REALLY* likes this particular one
		case 0x3100:		//To the point where it will recognise it from an almost blank wall
		case 0x0013:		//Keeping this here to ensure we don't use the same marker in the future.
		case 0x8C00:
			marker_num = -1;
			break;
		default:
			marker_num = -1;
			break;
	}
}

void OpenARScanner::cv_ARaugmentImage(IplImage* display, IplImage* img, CvPoint2D32f srcQuad[4], double scale)
{

	IplImage* cpy_img = cvCreateImage(cvGetSize(img), 8, 3);	// To hold Camera Image Mask 
	IplImage* neg_img = cvCreateImage(cvGetSize(img), 8, 3);	// To hold Marker Image Mask
	IplImage* blank;						// To assist Marker Pass

	blank = cvCreateImage(cvGetSize(display), 8, 3);
	cvZero(blank);
	cvNot(blank, blank);

	CvPoint2D32f dispQuad[4];
	CvMat* disp_warp_matrix = cvCreateMat(3, 3, CV_32FC1);    // Warp matrix to store perspective data required for display

	if (scale == CV_AR_DISP_SCALE_FIT)
	{
		dispQuad[0].x = 0;				// Positions of Display image (not yet transposed)
		dispQuad[0].y = 0;

		dispQuad[1].x = (float)display->width;
		dispQuad[1].y = 0;

		dispQuad[2].x = 0;
		dispQuad[2].y = (float)display->height;

		dispQuad[3].x = (float)display->width;
		dispQuad[3].y = (float)display->height;
	}
	else
	{
		dispQuad[0].x = (float)((display->width / 2) - (CV_AR_MARKER_SIZE / scale));			// Positions of Display image (not yet transposed)
		dispQuad[0].y = (float)((display->height / 2) - (CV_AR_MARKER_SIZE / scale));

		dispQuad[1].x = (float)((display->width / 2) + (CV_AR_MARKER_SIZE / scale));
		dispQuad[1].y = (float)((display->height / 2) - (CV_AR_MARKER_SIZE / scale));

		dispQuad[2].x = (float)((display->width / 2) - (CV_AR_MARKER_SIZE / scale));
		dispQuad[2].y = (float)((display->height / 2) + (CV_AR_MARKER_SIZE / scale));

		dispQuad[3].x = (float)((display->width / 2) + (CV_AR_MARKER_SIZE / scale));
		dispQuad[3].y = (float)((display->height / 2) + (CV_AR_MARKER_SIZE / scale));
	}

	cvGetPerspectiveTransform(dispQuad, srcQuad, disp_warp_matrix);	// Caclculate the Warp Matrix to which Display Image has to be transformed

	// Note the jugglery to augment due to OpenCV's limiation passing two images [- Marker Img and Raw Img] of DIFFERENT sizes 
	// while using "cvWarpPerspective".  

	cvZero(neg_img);
	cvZero(cpy_img);
	cvWarpPerspective(display, neg_img, disp_warp_matrix);
	cvWarpPerspective(blank, cpy_img, disp_warp_matrix);
	cvNot(cpy_img, cpy_img);
	cvAnd(cpy_img, img, cpy_img);
	cvOr(cpy_img, neg_img, img);

	// Release images
	cvReleaseImage(&cpy_img);
	cvReleaseImage(&neg_img);
	cvReleaseImage(&blank);

	cvReleaseMat(&disp_warp_matrix);

}


// Equation of the line ax+by+c=0; a=c[0], b=c[1], c=c[2] for (x)region 4-corner detection
void OpenARScanner::cv_lineEquation(CvPoint p1, CvPoint p2, double(&c)[3])
{
	c[0] = -((double)(p2.y - p1.y) / (double)(p2.x - p1.x));
	c[1] = (double)1.0;
	c[2] = (((double)(p2.y - p1.y) / (double)(p2.x - p1.x)) * (double)p1.x) - (double)p1.y;

	return;
}

// Perpendicular distance of a point wrt a line ax+by+c=0; will be +ve or -ve depending upon position of point wrt linen
double OpenARScanner::cv_distanceFormula(double c[], CvPoint p)
{
	double pdist = 0.0;
	pdist = ((double)(c[0] * p.x) + (double)(c[1] * p.y) + (c[2])) / (sqrt((double)(c[0] * c[0]) + (double)(c[1] * c[1])));
	return pdist;
}

// EOF
//______________________________________________________________________________________