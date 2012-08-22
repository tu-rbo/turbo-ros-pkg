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
/**
 * An implementation of a Gaussian Mixture Model with EM for segmentation
 * 
 * $RCSfile: Gmm3DSegmentation.cpp,v $
 * $Author: dubik $
 * $Date: 2009/03/10 15:00:57 $
 * $Revision: 1.1 $
 */

#include "Gmm3DSegmentation.h"
#include "ml.h"
#include "highgui.h"
#include <iostream>

using namespace std;

Gmm3DSegmentation::Gmm3DSegmentation(const IplImage* rawImg, bool blur) : Segmentation(rawImg, blur) {}

IplImage* Gmm3DSegmentation::segment(void) {
	const int numGaussians = 2;
	
	IplImage* floatImage = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 3 );
	cvConvert(img, floatImage);				// Convert to IPL_DEPTH_32F
	// cvCvtColor(floatImage, floatImage, CV_BGR2YCrCb);			// Convert to a YCbCr Image
	// cvSetImageCOI( floatImage, 1 ); 		// Set the channel of interest to intensity
	// IplImage* intensityChannel = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1);
	// 	cvCopy(floatImage, intensityChannel);	// Copy over the intensityChannel
	
	CvMat* samples = cvCreateMat( floatImage->height, floatImage->width, CV_32FC3 );
	int selectedCOI;
	samples = cvGetMat( floatImage, samples , &selectedCOI );
	// cvReshape( samples, samples, 0, samples->rows * samples->cols );
	cvReshape( samples, samples, 1, samples->rows * samples->cols );
	CvMat* labels = cvCreateMat( floatImage->height * floatImage->width, 1, CV_32SC1 );
	
	CvEM em_model;
	CvEMParams params;
//	CvMat samples_part;

	// initialize model's parameters
	params.covs      = NULL;
	params.means     = NULL;
	params.weights   = NULL;
	params.probs     = NULL;
	params.nclusters = numGaussians;
	params.cov_mat_type       = CvEM::COV_MAT_SPHERICAL;
	params.start_step         = CvEM::START_AUTO_STEP;
	params.term_crit.max_iter = 10;
	params.term_crit.epsilon  = 0.1;
	params.term_crit.type     = CV_TERMCRIT_ITER|CV_TERMCRIT_EPS;

	// cluster the data
	em_model.train( samples, 0, params, labels );

	cvReshape( labels, labels, 0, floatImage->height);

	// TODO: Probably fix this to be smarter
	// Assume the background is lighter than the foreground and
	// figure out which label has a higher mean.
	const CvMat* means = em_model.get_means();
	if (cvGetReal2D(means, 0, 0) < cvGetReal2D(means, 1, 0)) {
		cvSubRS( labels, cvScalar(1), labels);
	}

	// Convert labels from a 0..1 image to a 0..255 image
	cvConvertScale(labels, labels, 255);

	IplImage* returnImage = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );
	cvConvert(labels, returnImage);
	
	// TODO: Remove components that are connected to the border.
	
	
	// Housekeeping
	cvReleaseImage(&floatImage); floatImage = NULL;
	
	cvReleaseMat(&samples); samples = NULL;
	cvReleaseMat(&labels); labels = NULL;
	
	return returnImage;
}
