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
#ifndef FEATURE_DETECTOR_H
#define FEATURE_DETECTOR_H

#include <vector>
#include <boost/shared_ptr.hpp>

namespace vision {

class Feature;
typedef boost::shared_ptr<Feature> FeaturePtr;
class Image;
/**
 *\class    FeatureDetector
 *\brief    Abstract class. A strategy for detecting features on an image
 * Following the strategy design pattern, a class that detects features should
 * inherit from this class and implement the "detect(..)" method. This class can
 * then be used by a Tracker class when calling its "reset()" function. 
 *$Author: dubik $
 *$Date: 2009/03/10 15:00:49 $
 *$Revision: 1.1 $
 */
class FeatureDetector {
public:
    /**
     * sets the desired number of features
     *
     * @param maxFeatures - desired number of features
     */
    FeatureDetector(int maxFeatures) {
        _maxFeatures = maxFeatures;
    }

    FeatureDetector() {
        _maxFeatures = 0;
    }
    /**
     * default destructor
     */
    virtual ~FeatureDetector() {
    }

    /**
     * Given an image returns a vector of interesting features.
     * 
     * @param img - the image on wich the features are detected
     * @param mask - optional mask describing the region where detection occurs
     * @return - a vector of pointers to features discovered on the image
     */
    virtual std::vector<FeaturePtr> detect(const Image *img, const Image *mask =
            0) const = 0;

protected:
    int _maxFeatures; /**< maximum number of features that should be detected */
};
}
;
#endif
