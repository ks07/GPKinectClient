#include "OpenARScanner.h"

// Funny c++ rules, plus MS isn't up to date with c++11
int    OpenARScanner::CV_AR_MARKER_SIZE = 160;			// Marker decoding size = 160 * 160 Pixels
double OpenARScanner::CV_AR_DISP_SCALE_FIT = 0.0;		// Distort (& Fit) the Display Image
double OpenARScanner::CV_AR_DISP_SCALE_DEF = 0.5;		// Scale the Display Image

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

	IplImage* gray = 0;
	IplImage* thres = 0;
	IplImage* prcs_flg = 0;					// Process flag to flag whether the current pixel is already processed as part blob detection

	int h, w;								// Variables to store Image Height and Width

	int ihist[256];							// Array to store Histogram values
	float hist_val[256];					// Array to store Normalised Histogram values

	int blob_count;
	int n;									// Number of pixels in a blob
	int pos;								// Position or pixel value of the image

	int rectw, recth;						// Width and Height of the Bounding Box
	double aspect_ratio;					// Aspect Ratio of the Bounding Box

	int min_blob_sze = 50;					// Minimum Blob size limit 
	// int max_blob_sze = 150000;			// Maximum Blob size limit

	CvPoint P, Q, R, S;						// Corners of the Marker
	printf("Here 0\n");
	// Note: All Markers are tranposed to 160 * 160 pixels Image for decoding
	IplImage* marker_transposed_img = cvCreateImage(cvSize(CV_AR_MARKER_SIZE, CV_AR_MARKER_SIZE), 8, 1);
	printf("Here 0.1\n");
	CvPoint2D32f srcQuad[4], dstQuad[4];	// Warp matrix Parameters: Source, Destination
	printf("Here 0.2\n");
	dstQuad[0].x = 0;						// Positions of Marker image (to where it has to be transposed)
	dstQuad[0].y = 0;
	dstQuad[1].x = CV_AR_MARKER_SIZE;
	dstQuad[1].y = 0;
	dstQuad[2].x = 0;
	dstQuad[2].y = CV_AR_MARKER_SIZE;
	dstQuad[3].x = CV_AR_MARKER_SIZE;
	dstQuad[3].y = CV_AR_MARKER_SIZE;


	//Step	: Capture a frame from Camera for creating and initializing manipulation variables
	//Info	: Inbuit functions from OpenCV
	//Note	: 

	printf("Here 0.5\n");

	if (img == NULL)
	{
		printf("Loading image\n");
		img = cvLoadImage("markerphoto.jpg");
		if (!img)
		{
			printf("Load failed\n");
			return markers;
		}
	}

	printf("Here 1\n");

	// Creation of Intermediate 'Image' Objects required later
	gray = cvCreateImage(cvGetSize(img), 8, 1);				// To hold Grayscale Image
	thres = cvCreateImage(cvGetSize(img), 8, 1);			// To hold OTSU thresholded Image
	prcs_flg = cvCreateImage(cvGetSize(img), 8, 1);			// To hold Map of 'per Pixel' Flag to keep track while identifing Blobs

	int* clr_flg = (int*)malloc(sizeof(int) * img->width);		// Array representing elements of entire current row to assign Blob number
	int* clrprev_flg = (int*)malloc(sizeof(int) *img->width);	// Array representing elements of entire previous row to assign Blob number

	h = img->height;											// Height and width of the Image
	w = img->width;

	bool corner_flag = false;      								// Flag to check whether the current pixel is a Edgel or not
	CvPoint corners[10000];         							// Array to store all the Edgels.If the size of the array is small then there may be abrupt termination of the program

	CvMat* warp_matrix = cvCreateMat(3, 3, CV_32FC1);			// Warp matrix to store perspective data


	double t;				// variable to calculate timing

	int marker_id;
	int marker_num;
	bool valid_marker_found;

	printf("About to enter the Great Loop\n");
	bool newMarkerFound = true;
	while (newMarkerFound)		// While loop to query for Camera frame
	{
		newMarkerFound = false;

		cvCvtColor(img, gray, CV_RGB2GRAY);	// Convert RGB image to Gray

		memset(ihist, 0, 256);

		for (int j = 0; j < gray->height; ++j)	// Use Histogram values from Gray image
		{
			uchar* hist = (uchar*)(gray->imageData + j * gray->widthStep);
			for (int i = 0; i < gray->width; i++)
			{
				pos = hist[i];				// Check the pixel value
				ihist[pos] += 1;			// Use the pixel value as the position/"Weight"
			}
		}

		//Parameters required to calculate threshold using OTSU Method
		float prbn = 0.0;                   // First order cumulative
		float meanitr = 0.0;                // Second order cumulative
		float meanglb = 0.0;                // Global mean level
		int OPT_THRESH_VAL = 0;             // Optimum threshold value
		float param1, param2;               // Parameters required to work out OTSU threshold algorithm
		double param3 = 0.0;

		//Normalise histogram values and calculate global mean level
		for (int i = 0; i < 256; ++i)
		{
			hist_val[i] = ihist[i] / (float)(w * h);
			meanglb += ((float)i * hist_val[i]);
		}

		// Implementation of OTSU algorithm
		for (int i = 0; i < 255; i++)
		{
			prbn += (float)hist_val[i];
			meanitr += ((float)i * hist_val[i]);

			param1 = (float)((meanglb * prbn) - meanitr);
			param2 = (float)(param1 * param1) / (float)(prbn * (1.0f - prbn));

			if (param2 > param3)
			{
				param3 = param2;
				OPT_THRESH_VAL = i; 				// Update the "Weight/Value" as Optimum Threshold value
			}
		}

		cvThreshold(gray, thres, OPT_THRESH_VAL, 255, CV_THRESH_BINARY);	//Threshold the Image using the value obtained from OTSU method


		//Step	: Identify Blobs in the OTSU Thresholded Image
		//Info	: Custom Algorithm to Identify blobs
		//Note	: This is a complicated method. Better refer the presentation, documentation or the Demo

		blob_count = 0;				// Current Blob number used to represent the Blob
		CvPoint cornerA, cornerB; 	// Two Corners to represent Bounding Box

		memset(clr_flg, 0, w);		// Reset all the array elements ; Flag for tracking progress
		memset(clrprev_flg, 0, w);

		cvZero(prcs_flg);			// Reset all Process flags

		printf("About to start going through the image\n");
		for (int y = 0; y < thres->height; ++y)		//Start full scan of the image by incrementing y
		{
			uchar* prsnt = (uchar*)(thres->imageData + y * thres->widthStep);
			uchar* pntr_flg = (uchar*)(prcs_flg->imageData + y * prcs_flg->widthStep);  // pointer to access the present value of pixel in Process flag
			uchar* scn_prsnt;															// pointer to access the present value of pixel related to a particular blob
			uchar* scn_next;															// pointer to access the next value of pixel related to a particular blob

			for (int x = 0; x < thres->width; ++x)			//Start full scan of the image by incrementing x
			{
				int c = 0;									// Number of edgels in a particular blob
				marker_id = 0;								// Identification number of the particular pattern
				if ((prsnt[x] == 0) && (pntr_flg[x] == 0))	// If current pixel is black and has not been scanned before - continue
				{
					blob_count += 1;						// Increment at the start of processing new blob
					clr_flg[x] = blob_count;				// Update blob number
					pntr_flg[x] = 255;						// Mark the process flag

					n = 1;									// Update pixel count of this particular blob / this iteration

					cornerA.x = x;							// Update Bounding Box Location for this particular blob / this iteration
					cornerA.y = y;
					cornerB.x = x;
					cornerB.y = y;

					int lx, ly;								// Temp location to store the initial position of the blob
					int belowx = 0;

					bool checkbelow = true;					// Scan the below row to check the continuity of the blob

					ly = y;

					bool below_init = 1;					// Flags to facilitate the scanning of the entire blob once
					bool start = 1;

					//printf("Blob scan time\n");
					while (ly < h)							// Start the scanning of the blob
					{
						if (checkbelow == true)				// If there is continuity of the blob in the next row & checkbelow is set; continue to scan next row
						{
							if (below_init == 1) 			// Make a copy of Scanner pixel position once / initially
							{
								belowx = x;
								below_init = 0;
							}

							checkbelow = false;				// Clear flag before next flag

							scn_prsnt = (uchar*)(thres->imageData + ly * thres->widthStep);
							scn_next = (uchar*)(thres->imageData + (ly + 1) * thres->widthStep);

							pntr_flg = (uchar*)(prcs_flg->imageData + ly * prcs_flg->widthStep);

							bool onceb = 1;					// Flag to set and check blbo continuity for next row

							//printf("About move scanner pixel whatever that means\n");
							// Loop to move Scanner pixel to the extreme left pixel of the blob
							while ((scn_prsnt[belowx - 1] == 0) && ((belowx - 1) > 0) && (pntr_flg[belowx - 1] == 0))
							{
								cv_adjustBox(belowx, ly, cornerA, cornerB);    // Update Bounding Box corners
								pntr_flg[belowx] = 255;

								clr_flg[belowx] = blob_count;

								corner_flag = cv_checkCorner(thres->imageData, thres->widthStep, belowx, ly);
								if (corner_flag == true)		// Check for the Edgel and update Edgel storage
								{
									if (c < 10000)			// Make sure the allocated array size does not exceed
									{
										corners[c].x = belowx;
										corners[c].y = ly;
										c++;
									}
									corner_flag = false;
								}
								n = n + 1;
								belowx--;
							}
							//printf("Done moving scanner pixel whatever that means\n");
							//printf("About to scan a row of the blob\n");
							//Scanning of a particular row of the blob
							for (lx = belowx; lx < thres->width; ++lx)
							{
								if (start == 1)                 	// Initial/first row scan
								{
									//printf("First row scan\n");
									cv_adjustBox(lx, ly, cornerA, cornerB);
									pntr_flg[lx] = 255;

									clr_flg[lx] = blob_count;
									printf("Before cv_checkcorner\n");
									corner_flag = cv_checkCorner(thres->imageData, thres->widthStep, lx, ly);
									printf("After cv_checkcorner\n");
									if (corner_flag == true)
									{
										//printf("Corner flag true\n");
										if (c < 10000)					// Make sure the allocated array size does not exceed
										{
											//printf("c < 10000\n");
											corners[c].x = lx;
											corners[c].y = ly;
											c++;
										}
										corner_flag = false;
									}

									start = 0;
									if ((onceb == 1) && (scn_next[lx] == 0))                 // Check for the continuity
									{
										//printf("Something about continuity\n");
										belowx = lx;
										checkbelow = true;
										onceb = 0;
									}
									//printf("At the end of the first row stuff\n");
								}
								else if ((scn_prsnt[lx] == 0) && (pntr_flg[lx] == 0))        // Present pixel is black and has not been processed
								{
									//printf("Pixel black and not processed.\n");
									if ((clr_flg[lx - 1] == blob_count) || (clr_flg[lx + 1] == blob_count))        //Check for the continuity with previous scanned data
									{
										cv_adjustBox(lx, ly, cornerA, cornerB);

										pntr_flg[lx] = 255;

										clr_flg[lx] = blob_count;

										corner_flag = cv_checkCorner(thres->imageData, thres->widthStep, lx, ly);
										if (corner_flag == true)
										{
											if (c < 10000)					// Make sure the allocated array size does not exceed
											{
												corners[c].x = lx;
												corners[c].y = ly;
												c++;
											}
											corner_flag = false;
										}
										n = n + 1;

										if ((onceb == 1) && (scn_next[lx] == 0))
										{
											belowx = lx;
											checkbelow = true;
											onceb = 0;
										}
									}
									else if ((scn_prsnt[lx] == 0) && (clr_flg[lx - 2] == blob_count))	// Check for the continuity with previous scanned data
									{
										cv_adjustBox(lx, ly, cornerA, cornerB);

										pntr_flg[lx] = 255;

										clr_flg[lx] = blob_count;

										corner_flag = cv_checkCorner(thres->imageData, thres->widthStep, lx, ly);
										if (corner_flag == true)
										{
											if (c < 10000)					// Make sure the allocated array size does not exceed
											{
												corners[c].x = lx;
												corners[c].y = ly;
												c++;
											}
											corner_flag = false;
										}
										n = n + 1;

										if ((onceb == 1) && (scn_next[lx] == 0))
										{
											belowx = lx;
											checkbelow = true;
											onceb = 0;
										}
									}
									// Check for the continuity with previous scanned data
									else if ((scn_prsnt[lx] == 0) && ((clrprev_flg[lx - 1] == blob_count) || (clrprev_flg[lx] == blob_count) || (clrprev_flg[lx + 1] == blob_count)))
									{
										cv_adjustBox(lx, ly, cornerA, cornerB);

										pntr_flg[lx] = 255;

										clr_flg[lx] = blob_count;

										corner_flag = cv_checkCorner(thres->imageData, thres->widthStep, lx, ly);
										if (corner_flag == true)
										{
											if (c < 10000)					// Make sure the allocated array size does not exceed
											{
												corners[c].x = lx;
												corners[c].y = ly;
												c++;
											}
											corner_flag = false;
										}
										n = n + 1;

										if ((onceb == 1) && (scn_next[lx] == 0))
										{
											belowx = lx;
											checkbelow = true;
											onceb = 0;
										}

									}
									else
									{
										continue;
									}

								}
								else
								{
									clr_flg[lx] = 0;	// Current pixel is not a part of any blob
								}
								//printf("After the great big if\n");
							}				// End of scanning of a particular row of the blob
							//printf("Done scanning a row of the blob\n");
						}
						else				// If there is no continuity of the blob in the next row break from blob scan loop
						{
							break;
						}

						for (int q = 0; q < thres->width; ++q)	// Blob numbers of current row becomes Blob number of previous row for the next iteration of "row scan" for this particular blob
						{
							clrprev_flg[q] = clr_flg[q];
						}
						ly++;
					}
					// End of the Blob scanning routine 
					//printf("End of blob scan time\n");

					// At this point after scanning image data, A blob (or 'connected component') is obtained. We use this Blob for further analysis to confirm it is a Marker.


					// Get the Rectangular extent of the blob. This is used to estimate the span of the blob
					// If it too small, say only few pixels, it is too good to be true that it is a Marker. Thus reducing erroneous decoding
					rectw = abs(cornerA.x - cornerB.x);
					recth = abs(cornerA.y - cornerB.y);
					aspect_ratio = (double)rectw / (double)recth;

					if ((n > min_blob_sze))// && (n < max_blob_sze))		// Reduces chances of decoding erroneous 'Blobs' as markers
					{
						if ((aspect_ratio > 0.33) && (aspect_ratio < 3.0))	// Increases chances of identified 'Blobs' to be close to Square 
						{

							// Step	: Identify 4 corners of the blob assuming it be a potential Marker
							// Info	: Custom Algorithm to detect Corners using Pixel data || similar to FAST algorithm
							// Note	: 

							cv_ARgetMarkerPoints(c, corners, cornerA, cornerB, P, Q, R, S);      // 4-corners of the pattern obtained usig (+)region calculations

							// CvPoint to CvPoint2D32f conversion for Warp Matrix calculation

							srcQuad[0].x = P.x;				// Positions of the Marker in Image | "Deformed" Marker
							srcQuad[0].y = P.y;
							srcQuad[1].x = Q.x;
							srcQuad[1].y = Q.y;
							srcQuad[2].x = S.x;
							srcQuad[2].y = S.y;
							srcQuad[3].x = R.x;
							srcQuad[3].y = R.y;


							// Note: dstQuad[4];				// Positions to where Marker has to be transposed to | "Aligned" Marker

							// Note: All Markers are tranposed to 160 * 160 pixels Image for decoding

							cvGetPerspectiveTransform(srcQuad, dstQuad, warp_matrix);		// Warp Matrix Calculations
							cvWarpPerspective(thres, marker_transposed_img, warp_matrix);	// SMART! Clip and Transform the deformed Marker simultaneously using a Mask (Marker catcher) and Warp Matrix 


							// Step	: Decode 16bit Marker to Identify marker uniquely and Get associated Marker Number
							// Info	: 
							// Note	: The Marker ID is valid in any 4 Direction of looking

							cv_ARgetMarkerID_16b(marker_transposed_img, marker_id);	// Get Marker ID
							cv_ARgetMarkerNum(marker_id, marker_num);		// Get Marker Number Corrosponding to ID


							if (marker_num > 0)
							{
								valid_marker_found = true;
							}
							else
							{
								// If 4-Corners are not obtained from (+) region partitioning ; try to calculate corners from (x) region partitioning
								cv_ARgetMarkerPoints2(c, corners, cornerA, cornerB, P, Q, R, S);

								srcQuad[0].x = P.x;			// Positions of the Marker in Image | "Deformed" Marker
								srcQuad[0].y = P.y;
								srcQuad[1].x = Q.x;
								srcQuad[1].y = Q.y;
								srcQuad[2].x = S.x;
								srcQuad[2].y = S.y;
								srcQuad[3].x = R.x;
								srcQuad[3].y = R.y;

								cvGetPerspectiveTransform(srcQuad, dstQuad, warp_matrix);		// Warp Matrix Calculations
								cvWarpPerspective(thres, marker_transposed_img, warp_matrix);

								cv_ARgetMarkerID_16b(marker_transposed_img, marker_id);	// Get Marker ID
								cv_ARgetMarkerNum(marker_id, marker_num);		// Get Marker Number Corrosponding to I

							}

							if (marker_num > 0)				// Now check if still marker is valid
							{
								valid_marker_found = true;
							}

							if (valid_marker_found == true)			// Show Display image corrosponding to the Marker Number
							{


								// Step	: Augment the "Display object" in position of the marker over Camera Image using the Warp Matrix
								// Info	: 
								// Note	: Marker number used to make it easlier to change 'Display' image accordingly, 
								// Also Note the jugglery to augment due to OpenCV's limiation passing two images of DIFFERENT sizes  
								// while using "cvWarpPerspective".  
								//printf("Marker: %d,\n", marker_num);

								for (int i = 0; i < markers.values.size(); i++)
								{
									if (markers.values[i] == marker_num)
									{
										marker_num = -1;
									}
								}

								if (/*std::find(markers.values.begin(), markers.values.end(), marker_num) != markers.values.end() && markers.values.size() > 0*/
									marker_num > 0)
								{
									printf("New marker %d found in image\n", marker_num);
									CvPoint centre;
									centre.x = (cornerA.x + cornerB.x) / 2;
									centre.y = (cornerA.y + cornerB.y) / 2;
									markers.centres.push_back(centre);
									markers.values.push_back(marker_num);
									markers.count++;

									newMarkerFound = true;
								}
							}

							// If a valid marker was detected, then a Image will be augmented on that blob and process will continue to analysis of next blob

						}
						else	// Discard the blob data
						{
							blob_count = blob_count - 1;
						}
					}
					else  		// Discard the blob data               
					{
						blob_count = blob_count - 1;
					}
				}
				else     // If current pixel is not black do nothing
				{
					continue;
				}
			}	// End full scan of the image by incrementing x
		}	// End full scan of the image by incrementing y


		t = cvGetTickCount() - t;
		//printf("Calc. = %.4gms : FPS = %.4g\n",t/((double)cvGetTickFrequency()*1000.), ((double)cvGetTickFrequency()*1000.*1000.)/t);

		//cvShowImage("Camera", img);

		//cvShowImage("Camera", img);

		//key = cvWaitKey(1000);			// OPENCV: wait for 1ms before accessing next frame
	}
	//cvWriteFrame( writer, img );		// Save frame to output

	//cvDestroyWindow("Camera");			// Release various parameters

	printf("Near the end of the scan\n");

	cvReleaseImage(&img);
	cvReleaseImage(&gray);
	cvReleaseImage(&thres);
	cvReleaseImage(&prcs_flg);
	cvReleaseImage(&marker_transposed_img);

	cvReleaseMat(&warp_matrix);

	free(clr_flg);
	free(clrprev_flg);

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
				printf("Before the suspected bit.\n");
				printf("Accessing ptr[%d][%d].\n", i, x + j);
				printf("x = %d, j = %d.\n", x, j);
				printf("wind_bnd = %d, wind_sz = %d\n", wind_bnd, wind_sz);
				getchar();
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