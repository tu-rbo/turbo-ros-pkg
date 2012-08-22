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
 * SURFDetector Model (implementation)
 ******************************************************************************
 * $Author: roberto $
 * $Date: 2010/09/03 18:00:29 $
 * $Revision: 1.0 $
 *****************************************************************************/

#include "surfDetector.h"

using namespace std;
using namespace vision;

SURFDetector::SURFDetector(int maxFeatures, double hessianThreshold, int extended) :
FeatureDetector(maxFeatures)
{
	_surfParameters = cvSURFParams(hessianThreshold, extended);
	_storage = cvCreateMemStorage(0);
}
// ============================================================================

SURFDetector::~SURFDetector()
{
	cvReleaseMemStorage(&_storage);
}
// ============================================================================

std::vector<FeaturePtr> SURFDetector::detect(const Image *img,
                                                const Image *mask ) const
{
    vector<FeaturePtr> features;

	//clear pointers and memory storage
	cvClearMemStorage(_storage);
	CvSeq *keypoints = NULL;
	CvSeq *descriptors = NULL;
	
	//create SURF parameters
    CvSURFParams params = cvSURFParams(500, 1);
	
	//extract SURF features from the object
	const IplImage* iplimage= img->getIplImage();
	IplImage* iplimage_bw = cvCreateImage(cv::Size(iplimage->width,iplimage->height), IPL_DEPTH_8U, 1);
	cvCvtColor(iplimage, iplimage_bw, CV_BGR2GRAY);
    cvExtractSURF(iplimage_bw, 0, &keypoints, &descriptors, _storage, params);
	
    for(int i=0;i<keypoints->total;i++){
        CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem(keypoints, i);
		int length = (int)(descriptors->elem_size/sizeof(float));
		vector<float> desc;
		for(int j=0;j<length;j++){
			desc.push_back(((float*)cvGetSeqElem(descriptors,i))[j]);
		}

		FeaturePtr sf(new SURFFeature(r->pt.x, r->pt.y, r->laplacian, r->size, r->dir, r->hessian, desc, false));
		//if(r->hessian>2000)
		features.push_back(sf);
	}

	if(_maxFeatures!=0){
		//0 means we want to get as many as possible
		while((int)features.size()>_maxFeatures){
			//we have detected more features than asked, so we need to remove the ones with low score
		    SURFFeaturePtr minSurfFt = boost::dynamic_pointer_cast<SURFFeature>(*features.begin());
			float min = minSurfFt->hessian;

			vector<FeaturePtr>::iterator minIt = features.begin();
			for(vector<FeaturePtr>::iterator it=features.begin();it!=features.end();it++){
			    SURFFeaturePtr surfFt = boost::dynamic_pointer_cast<SURFFeature>(*it);
				if(surfFt->hessian<min){
					min=surfFt->hessian;
					minIt=it;
				}
			}
			features.erase(minIt);
		}
	}
	cvReleaseImage(&iplimage_bw);
	return features;  
}

/** ***************************************************************************
                                End of File
 ******************************************************************************/
