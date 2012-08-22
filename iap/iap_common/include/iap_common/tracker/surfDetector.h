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
#ifndef SURF_DETECTOR_H
#define SURF_DETECTOR_H

#include <vector>
#include "surfFeature.h"
#include "image.h"
#include "featureDetector.h"

namespace vision{

/**
 *\class    SURFDetector
 *\brief
 *$Author: roberto $
 *$Date: 2010/09/03 18:00:56 $
 *$Revision: 1.0 $
 */
 class SURFDetector : public FeatureDetector{
  public:

    /**
     * @param maxFeatures - The detector will return a vector of size equal or
     * lower than 'maxFeatures'
     * @param hessianThreshold - only features with keypoint.hessian
	 * larger than that are extracted. Good default value is ˜300-500 
	 * (can depend on the average local contrast and sharpness of the image).
     * @param extended - 0 means basic descriptors (64 elements each),
	 * 1 means extended descriptors (128 elements each)
     */
    SURFDetector(int maxFeatures, double hessianThreshold=500, int extended=0);

    /**
     * default destructor
     */
    virtual ~SURFDetector();

    /**
     * detects SURF features on the image
     *
     * @param img - the image on which to detect features
     * @param mask - optional mask describing the region where detection occurs
     *
     * @return the vector of SURF features detected by the algorithm
     */
	virtual std::vector<FeaturePtr> detect(const Image *img,
                                                const Image *mask = NULL ) const;
	CvSURFParams getSurfParameters() const;

  private:
	CvSURFParams _surfParameters;
	CvMemStorage* _storage;
 };
};
#endif
