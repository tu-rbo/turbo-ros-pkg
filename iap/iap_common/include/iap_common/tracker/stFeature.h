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
#ifndef ST_FEATURE_H
#define ST_FEATURE_H

#include "feature.h"

namespace vision{

class STFeature;
typedef boost::shared_ptr<STFeature> STFeaturePtr;


/**
 *\class    STFeature
 *\brief    Implements Feature. Point feature with extentions for the 
 * ShiTomasiDetector
 *$Author: dubik $
 *$Date: 2009/03/10 15:00:53 $
 *$Revision: 1.1 $
 */
 class STFeature : public Feature{
  public:
  /**
   * STFeature constructor, creates a Shi and Tomasi Feature
   *
   * @param x - floating point x position of the center of the feature
   * @param y - floating point y position of the center of the feature
   * @param isLost - if true the feature will be considered to be no longer
   * on the image
   * @param error - the measurment of how well the feature is matched up for
   * the current frame from the last.
   */
    STFeature(float x=0, float y=0, bool isLost=false, float error=0.0,
	      float quality = -1.0);

	STFeature(float x, float y, Pixel &p, bool isLost=false, float error=0.0,
	      float quality = -1.0);


    /**
     * Default destructor
     */
    ~STFeature();

    /**
     * Creates a new STFeature object with the same values as this one and
     * passes its refrance cast as a Feature
     *
     * @return - a referance to the clone of this Feature object
     */
    virtual FeaturePtr clone() const;

    virtual float compareTo(std::pair<float, float> pt,
                             const Image* img, const Image* mask);

    virtual bool isFeature(std::pair<float, float> pt,
                             const Image* img, const Image* mask);

 };
};
#endif
