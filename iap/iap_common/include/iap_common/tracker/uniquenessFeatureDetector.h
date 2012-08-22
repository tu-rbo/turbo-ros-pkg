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
#ifndef UNIQUENESS_TOMASI_DETECTOR_H
#define UNIQUENESS_TOMASI_DETECTOR_H

#include "featureDetector.h"

namespace vision{

/**
 *\class    UniquenessFeatureDetector
 *\brief
 *$Author: dubik $
 *$Date: 2009/03/10 15:00:56 $
 *$Revision: 1.1 $
 */
 class UniquenessFeatureDetector : public FeatureDetector{
  public:

    /**
     * @param maxFeatures - The detector will return a vector of size ewual to
     * 'maxFeatures'
     * @param winSize - The size of the square window used to determine 
     * uniquness (quality) of a feature
     * @param dummyFt - A concrete feature vector; each Feature is used as a
     * template to find more features of its kind in the image. The Feature
     * detector assumes that each feature is tepresents a different concrete
     * type.
     */
    UniquenessFeatureDetector(int maxFeatures, int winSize,
                              const std::vector<FeaturePtr> prototypeFt);

    /**
     * default destructor
     */
    virtual ~UniquenessFeatureDetector();

    /**
     * detects features on the image based on the algorithm by shi and tomasi
     *
     * @param img - the image on wich to detect features
     * @param mask - optional mask describing the region where detection occurs
     *
     * @return the vector of features detected by the algorithm
     */
    virtual std::vector<FeaturePtr> detect(const Image *img,
                                         const Image *mask = 0) const;
  private:
    int _winSize;
    std::vector<FeaturePtr> _prototypeFt;
 };
};
#endif
