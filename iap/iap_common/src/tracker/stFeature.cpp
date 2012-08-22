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
 * STFeature Model (implementation)
 ******************************************************************************
 * $Author: dubik $
 * $Date: 2009/03/10 17:00:29 $
 * $Revision: 1.2 $
 *****************************************************************************/

#include "stFeature.h"
#include <string>
using namespace vision;

STFeature::STFeature(float x, float y, bool lost, float error, float quality)
    : Feature(x,y,lost,error, quality, 0) //last param is groupId =0 by default
{}
// ============================================================================

STFeature::STFeature(float x, float y, Pixel &p, bool lost, float error, float quality)
    : Feature(x,y,p,lost,error, quality, 0) //last param is groupId =0 by default
{}
// ============================================================================

STFeature::~STFeature()
{}
// ============================================================================

FeaturePtr STFeature::clone() const
{
    FeaturePtr copy(new STFeature(Feature::_x, Feature::_y, Feature::_lost, Feature::_error));
    return copy;
}

float STFeature::compareTo(std::pair<float, float> pt,
                 const Image* img, const Image* mask)
{
    throw std::string("compareTo functionality is not defined for STFeature");
}

bool STFeature::isFeature(std::pair<float, float> pt,
               const Image* img, const Image* mask)
{
    throw std::string("isFeature functionality is not defined for STFeature");
}
/** ***************************************************************************
                                End of File
 ******************************************************************************/
