/* * IAP - Interactive Perception System - Segment visually observable environment
 * into rigid bodies and estimate type and properties of joints between them by
 * means of interaction.
 * Copyright (C) 2012 Technische Universitaet Berlin - RBO
 * <robotics@robotics.tu-berlin.de>
 * 
 * This file is part of IAP.
 * 
 * IAP is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * IAP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with IAP.  If not, see <http://www.gnu.org/licenses/>.
 */
/** ***************************************************************************
* ImageInStream Model (implementation)
******************************************************************************
* $Author: dubik $
* $Date: 2009/03/10 15:01:14 $
* $Revision: 1.1 $
*****************************************************************************/

#include "imageInStream.h"

#include <iostream>

using namespace vision;

ImageInStream::ImageInStream()
{
	capture = cvCaptureFromCAM(0);

	if(!capture){
		throw std::string("Could not initialize capture from camera");
	}
}
// ============================================================================

ImageInStream::ImageInStream(std::string location)
{
	capture = cvCaptureFromAVI(location.c_str());

	if(!capture){
		throw std::string("Could not initialize capture from video");
	}
}
// ============================================================================

ImageInStream::ImageInStream(std::vector< std::string > fileList)
{
	capture = 0;
	images = fileList;
}
// ============================================================================

ImageInStream::~ImageInStream()
{
	if(capture != 0) cvReleaseCapture( &capture );
}
// ============================================================================

Image* ImageInStream::getNextImage()
{
	// This IplImage will get deleted by the capture device
	IplImage* frame = 0;
	// This Image makes a clone the above image, the requesting function
	// will delete the image
	Image* img;
	if(capture != 0)
	{
		frame = cvQueryFrame( capture );
	}else if(!images.empty()){
		std::string pic = images.back();
		images.pop_back();
		std::cout << pic << std::endl;
		frame = cvLoadImage(pic.c_str());
	}

	if(!frame){
		throw std::string("could not get frame");
	}
	img = new Image(frame);

	return img;
}

double ImageInStream::getFps() const
{
	return cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
}

double ImageInStream::getFrameCount() const
{
	double frameCount= cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_COUNT);
	printf("Frame Count in Image In Stream = %f", frameCount);
	return frameCount;
}

CvSize ImageInStream::getSize() const {
	CvSize retSize;
	retSize.height = (int) cvGetCaptureProperty(capture,
		CV_CAP_PROP_FRAME_HEIGHT);
	retSize.width = (int) cvGetCaptureProperty(capture,
		CV_CAP_PROP_FRAME_WIDTH);

	return retSize;
}

bool ImageInStream::calibrateCamera(int columns, int rows) {
	const int board_dt = 2;
	static int number_of_camera = 0;
	int board_w = 8;
	int board_h = 6;
	int n_boards = 8; // Number of boards
	int board_n = board_w * board_h;
	CvMat* intrinsic_matrix = cvCreateMat(3, 3, CV_32FC1);
	CvMat* distortion_coeffs = cvCreateMat(5, 1, CV_32FC1);
	CvSize board_sz = cvSize(board_w, board_h);
	CvMat* image_points = cvCreateMat(n_boards * board_n, 2, CV_32FC1);
	CvMat* object_points = cvCreateMat(n_boards * board_n, 3, CV_32FC1);
	CvMat* point_counts = cvCreateMat(n_boards, 1, CV_32SC1);

	CvPoint2D32f* corners = new CvPoint2D32f[board_n];
	int corner_count;
	int successes = 0;
	int step, frame = 0;
	IplImage *image = (this->getNextImage())->getIplImage();
	IplImage *gray_image = cvCreateImage(cvGetSize(image), 8, 1);

	// Capture Corner views loop until we've got n_boards
	// succesful captures (all corners on the board are found)
	std::string window_name = std::string("Calibration");
	std::stringstream ss;
	std::string sss;
	ss << number_of_camera;
	ss >> sss;
	window_name+=sss;
	while (successes < n_boards) {
		// Skp every board_dt frames to allow user to move chessboard
		if (frame++ % board_dt == 0) {
			int found = cvFindChessboardCorners(image, board_sz, corners,
				&corner_count, CV_CALIB_CB_ADAPTIVE_THRESH
				| CV_CALIB_CB_FILTER_QUADS);
			// Get subpixel accuracy on those corners
			cvCvtColor(image, gray_image, CV_BGR2GRAY);
			cvFindCornerSubPix(gray_image, corners, corner_count,
				cvSize(11, 11), cvSize(-1, -1), cvTermCriteria(
				CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			cvDrawChessboardCorners(image, board_sz, corners, corner_count,
				found);

			cvShowImage(window_name.c_str(), image);
			// If we got a good board, add it to our data
			if (corner_count == board_n) {
				step = successes * board_n;
				for (int i = step, j = 0; j < board_n; ++i, ++j) {
					CV_MAT_ELEM( *image_points, float, i, 0 ) = corners[j].x;
					CV_MAT_ELEM( *image_points, float, i, 1 ) = corners[j].y;
					CV_MAT_ELEM( *object_points, float, i, 0 ) = j / board_w;
					CV_MAT_ELEM( *object_points, float, i, 1 ) = j % board_w;
					CV_MAT_ELEM( *object_points, float, i, 2 ) = 0.0f;
				}
				CV_MAT_ELEM( *point_counts, int, successes, 0 ) = board_n;
				successes++;
			}
		}
		image = (this->getNextImage())->getIplImage(); // Get next image
	} // End collection while loop

	// Allocate matrices according to how many chessboards found
	CvMat* object_points2 = cvCreateMat(successes * board_n, 3, CV_32FC1);
	CvMat* image_points2 = cvCreateMat(successes * board_n, 2, CV_32FC1);
	CvMat* point_counts2 = cvCreateMat(successes, 1, CV_32SC1);

	// Transfer the points into the correct size matrices
	for (int i = 0; i < successes * board_n; ++i) {
		CV_MAT_ELEM( *image_points2, float, i, 0)
			= CV_MAT_ELEM( *image_points, float, i, 0 );
		CV_MAT_ELEM( *image_points2, float, i, 1)
			= CV_MAT_ELEM( *image_points, float, i, 1 );
		CV_MAT_ELEM( *object_points2, float, i, 0)
			= CV_MAT_ELEM( *object_points, float, i, 0 );
		CV_MAT_ELEM( *object_points2, float, i, 1)
			= CV_MAT_ELEM( *object_points, float, i, 1 );
		CV_MAT_ELEM( *object_points2, float, i, 2)
			= CV_MAT_ELEM( *object_points, float, i, 2 );
	}

	for (int i = 0; i < successes; ++i) {
		CV_MAT_ELEM( *point_counts2, int, i, 0 )
			= CV_MAT_ELEM( *point_counts, int, i, 0 );
	}
	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
	cvReleaseMat(&point_counts);
	// At this point we have all the chessboard corners we need
	// Initiliazie the intrinsic matrix such that the two focal lengths
	// have a ratio of 1.0
	CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0;
	CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0;
	// Calibrate the camera
	cvCalibrateCamera2(object_points2, image_points2, point_counts2, cvGetSize(
		image), intrinsic_matrix, distortion_coeffs, NULL, NULL,
		CV_CALIB_FIX_ASPECT_RATIO);
	// Save the intrinsics and distortions
	std::string file_name_int = std::string("Intrinsics");
	std::string file_name_dist = std::string("Distortion");
	sss+=std::string(".txt");
	file_name_int+=sss;
	file_name_dist += sss;
	cvSave(file_name_int.c_str(), intrinsic_matrix);
	cvSave(file_name_dist.c_str(), distortion_coeffs);
	number_of_camera+=1;
	return true;
}
/** ***************************************************************************
End of File
******************************************************************************/

