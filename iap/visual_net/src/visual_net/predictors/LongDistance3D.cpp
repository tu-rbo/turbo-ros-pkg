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
 * LongDistance3D.cpp
 *
 *  Created on: May 8, 2012
 *      Author: roberto
 */

#include "Graph.h"
#include <iostream>
#include "LongDistance3D.h"
#include <time.h>
#include <cmath>

using namespace std;
using namespace vision;

namespace VisualGraph
{

LongDistance3D::LongDistance3D(double minDistance, double maxDistance) :
  Predictor(string("LongDistance3D"))
{
  this->minDistance = minDistance;
  this->maxDistance = maxDistance;
}

LongDistance3D::~LongDistance3D(void)
{
}

void LongDistance3D::compute()
{
  std::cout << "min dist " << minDistance << " max dist " << maxDistance << std::endl;
  for (int i = 0; i < numVertices; i++)
  {
    for (int j = i + 1; j < numVertices; j++)
    {
      if (adjMatrix[i][j]->isConnected() && adjMatrix[j][i]->isConnected())
      {
        FeaturePtr f1a = vList[i]->getFeatureData(frame1);
        FeaturePtr f1b = vList[j]->getFeatureData(frame1);
        FeaturePtr f2a = vList[i]->getFeatureData(frame2);
        FeaturePtr f2b = vList[j]->getFeatureData(frame2);

        float distA = sqrt(
                           (f1a->getX() - f1b->getX()) * (f1a->getX() - f1b->getX()) + (f1a->getY() - f1b->getY())
                               * (f1a->getY() - f1b->getY()) + (f1a->getZ() - f1b->getZ())
                               * (f1a->getZ() - f1b->getZ()));
        float distB = sqrt(
                           (f2a->getX() - f2b->getX()) * (f2a->getX() - f2b->getX()) + (f2a->getY() - f2b->getY())
                               * (f2a->getY() - f2b->getY()) + (f2a->getZ() - f2b->getZ())
                               * (f2a->getZ() - f2b->getZ()));

        if (distA > maxDistance || distB > maxDistance)
        {
          adjMatrix[i][j]->setCapacity(predictorName, 0.);
          adjMatrix[j][i]->setCapacity(predictorName, 0.);
        }
        else
        {
          // Under the threshold, probability is inverse proportional to distance
          double minimum_distance = (distA >= distB ? distA : distB);
          if (minimum_distance < minDistance)
          {
            adjMatrix[i][j]->setCapacity(predictorName, 1.0);
            adjMatrix[j][i]->setCapacity(predictorName, 1.0);
          }
          else
          {
            adjMatrix[i][j]->setCapacity(
                                         predictorName,
                                         1.0 - ((double)(minimum_distance - minDistance) / (double)(maxDistance
                                             - minDistance)));
            adjMatrix[j][i]->setCapacity(
                                         predictorName,
                                         1.0 - ((double)(minimum_distance - minDistance) / (double)(maxDistance
                                             - minDistance)));
          }
        }
      }
    }
  }
}

}
;
