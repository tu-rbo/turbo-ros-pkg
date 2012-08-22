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
/*
 * cluster_generator.cpp
 *
 *  Created on: Dec 21, 2011
 *      Author: roberto
 */

#include "cluster_generator.h"

namespace vision
{

vision::FeaturePtr generateRandFeature(double scale)
{
  return vision::FeaturePtr(
                            new Feature(scale * ((double)rand() / (double)RAND_MAX - 0.5),
                                        scale * ((double)rand() / (double)RAND_MAX - 0.5),
                                        scale * ((double)rand() / (double)RAND_MAX - 0.5)));
}

vision::FeaturePtr generateRandFeature(vision::FeaturePtr& position, double scale)
{
  vision::FeaturePtr result = position->clone();
  vision::FeaturePtr addition = generateRandFeature(scale);
  addition->setX(addition->getX() + result->getX());
  addition->setY(addition->getY() + result->getY());
  addition->setZ(addition->getZ() + result->getZ());
  return addition;
}
}
