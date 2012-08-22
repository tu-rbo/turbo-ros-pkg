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
 * ShiTomasiDetector Model (implementation)
 ******************************************************************************
 * $Author: dubik $
 * $Date: 2009/05/29 19:56:40 $
 * $Revision: 1.2 $
 *****************************************************************************/

#include "shiTomasiDetector.h"
#include "stFeature.h"
#include "image.h"

#include "cv.h"
#include <vector>
#include <iostream>


using namespace vision;

ShiTomasiDetector::ShiTomasiDetector(int maxFeatures, int _min_distance=10) :
FeatureDetector(maxFeatures)
{
  min_distance=_min_distance;
}
// ============================================================================

ShiTomasiDetector::~ShiTomasiDetector()
{}
// ============================================================================

std::vector<FeaturePtr> ShiTomasiDetector::detect(const Image *img,
                                                const Image *mask ) const
{
    std::vector<FeaturePtr> features;

    if(FeatureDetector::_maxFeatures > 0)
    {
        features.reserve(FeatureDetector::_maxFeatures);

        const IplImage* frame = img->getIplImage();
        CvSize imgSize = cvGetSize(frame);

        double quality = 0.01;
        int win_size = 5;

        CvPoint2D32f* points =
      (CvPoint2D32f*)cvAlloc(FeatureDetector::_maxFeatures*sizeof(points[0]));

        IplImage* image =   cvCreateImage( imgSize, 8,  3 );	
        image->origin   =   frame->origin;
        IplImage* grey  =   cvCreateImage( imgSize, 8,  1 );
        IplImage* eig   =   cvCreateImage( imgSize, 32, 1 );
        IplImage* temp  =   cvCreateImage( imgSize, 32, 1 );

        cvCopy( img->getIplImage(), image, 0 );
        cvCvtColor( image, grey, CV_BGR2GRAY );


        const IplImage* IplMask = 0;
        if(mask){
            IplMask = mask->getIplImage();
        }


        int count=FeatureDetector::_maxFeatures;
        //The call to find the "good features"
        cvGoodFeaturesToTrack( grey, eig, temp, points, &count,
                                quality, min_distance, IplMask, 3, 0, 0.04 );

		if(count<FeatureDetector::_maxFeatures){
			throw("unable to create enough features...");
		}

        //The call to improve the features
        cvFindCornerSubPix( grey, points, count,
        cvSize(win_size,win_size), cvSize(-1,-1),
        cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));

        //FeatureDetector::_maxFeatures = count;

        cvReleaseImage( &eig );
        cvReleaseImage( &temp );
        cvReleaseImage( &image );
        cvReleaseImage( &grey );

        for(int i = 0; i < FeatureDetector::_maxFeatures; i++){
			Pixel p=img->get2D(imgSize.height - (int)points[i].y -1, (int)points[i].x);
			STFeaturePtr feat = STFeaturePtr(new STFeature(points[i].x, imgSize.height - points[i].y, p, false, -1));
            features.push_back( feat );
        }

        cvFree( &points );
    }
    return features;
}
/** ***************************************************************************
                                End of File
 ******************************************************************************/
