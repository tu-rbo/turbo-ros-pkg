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
 * ShortDistance3D.cpp
 *
 *  Created on: May 8, 2012
 *      Author: roberto
 */

#include "Graph.h"
#include <iostream>
#include "ShortDistance3D.h"
#include <time.h>
#include <cmath>

using namespace std;
using namespace vision;

namespace VisualGraph
{

ShortDistance3D::ShortDistance3D(double minDistance) :
  Predictor(string("ShortDistance3D"))
{
  this->minDistance = minDistance;
}

ShortDistance3D::~ShortDistance3D(void)
{
}

void ShortDistance3D::compute()
{
  for (int i = 0; i < numVertices; i++)
  {
    for (int j = i + 1; j < numVertices; j++)
    {
      FeaturePtr f1a = vList[i]->getFeatureData(frame1);
      FeaturePtr f1b = vList[j]->getFeatureData(frame1);
      FeaturePtr f2a = vList[i]->getFeatureData(frame2);
      FeaturePtr f2b = vList[j]->getFeatureData(frame2);

      //the distance between the two features in frame1 and in frame2
      float dist1 = sqrt(
                         (f1a->getX() - f1b->getX()) * (f1a->getX() - f1b->getX()) + (f1a->getY() - f1b->getY())
                             * (f1a->getY() - f1b->getY())+ (f1a->getZ() - f1b->getZ())
                                 * (f1a->getZ() - f1b->getZ()));
      float dist2 = sqrt(
                         (f2a->getX() - f2b->getX()) * (f2a->getX() - f2b->getX()) + (f2a->getY() - f2b->getY())
                             * (f2a->getY() - f2b->getY())+ (f2a->getZ() - f2b->getZ())
                             * (f2a->getZ() - f2b->getZ()));

      if (dist1 <= minDistance && dist2 <= minDistance)
      {
        adjMatrix[i][j]->setCapacity(predictorName, 1.0);
        adjMatrix[j][i]->setCapacity(predictorName, 1.0);
      }
    }
  }
}

}
;
