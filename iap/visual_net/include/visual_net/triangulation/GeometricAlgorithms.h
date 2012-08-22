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
#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <vector>
#include <iostream>
#include "Triangulate.h"
#include "feature.h"

#include "image.h"

#ifndef _GeometricAlgorithms_H_
#define _GeometricAlgorithms_H_

class GeometricAlgorithms{
  public:
    void removeIntersections(std::vector<vision::FeaturePtr> features, bool **connectivityMatrix);
    void removeTrianglesArea(std::vector<vision::FeaturePtr> prevFeatures, std::vector<vision::FeaturePtr> currFeatures, bool **connectivityMatrix);

//  private:
    bool checkIntersection(vision::FeaturePtr f, std::vector<vision::FeaturePtr> neighbors, std::vector<vision::FeaturePtr> features, bool **connectivityMatrix);
    bool checkIntersection(vision::FeaturePtr f1a, vision::FeaturePtr f1b, vision::FeaturePtr f2a, vision::FeaturePtr f2b);
    bool same_sign(double a, double b);

    std::vector<double> computeTrianglesArea(vision::FeaturePtr f, std::vector<vision::FeaturePtr> neighbors, std::vector<vision::FeaturePtr> features, bool **connectivityMatrix);
    bool areaChanged(vision::FeaturePtr prevF, vision::FeaturePtr currF, std::vector<vision::FeaturePtr> prevNeighbors, std::vector<vision::FeaturePtr> currNeighbors,
                     std::vector<vision::FeaturePtr> prevFeatures,  std::vector<vision::FeaturePtr> currFeatures, bool **connectivityMatrix);
};

#endif
