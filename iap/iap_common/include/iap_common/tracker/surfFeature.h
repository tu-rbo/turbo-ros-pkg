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

#ifndef SURF_FEARURE_H
#define SURF_FEARURE_H

#include "feature.h"
#include <vector>

namespace vision{

class SURFFeature;
typedef boost::shared_ptr<SURFFeature> SURFFeaturePtr;

/**
	@author Dubi Katz <dubik@cs.umass.edu>
*/
class SURFFeature : public Feature{
public:

	int laplacian;				   // -1, 0 or +1. sign of the laplacian at the point. 
								   // can be used to speedup feature comparison 
								   // (features with laplacians of different signs cannot match) 
    int size;					   // size of the feature 
    float dir;					   // orientation of the feature: 0..360 degrees 
    float hessian;				   // value of the hessian (can be used to 
								   // approximately estimate the feature strengths; 
								   // see also params.hessianThreshold) 

	std::vector<float> descriptor;      //SURF descriptor 

	SURFFeature(float x, float y, int laplacian, int size, float dir, float hessian, std::vector<float> descriptor, bool lost);

    ~SURFFeature(){}

    FeaturePtr clone();
};
};
#endif
