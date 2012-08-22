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
#include "Graph.h"
#include <iostream>
#include "RelativeMotion.h"
#include <time.h>
#include <cmath>

using namespace std;
using namespace vision;

namespace VisualGraph
{

RelativeMotion::RelativeMotion(double minDistance) :
  Predictor(string("RelativeMotion"))
{
  this->minDistance = minDistance;
}

RelativeMotion::~RelativeMotion(void)
{
}

void RelativeMotion::compute()
{
  for (int i = 0; i < numVertices; i++)
  {
    for (int j = i + 1; j < numVertices; j++)
    {
      double relative_motion = 0;
      for (int t1 = frame1; t1 < frame2; t1++)
      {
        for (int t2 = t1; t2 < frame2; t2++)
        {
          FeaturePtr f1a = vList[i]->getFeatureData(t1);
          FeaturePtr f1b = vList[j]->getFeatureData(t1);
          FeaturePtr f2a = vList[i]->getFeatureData(t2);
          FeaturePtr f2b = vList[j]->getFeatureData(t2);

          //the distance between the two features in frame1 and in frame2
          float dist1 = sqrt(
                             (f1a->getX() - f1b->getX()) * (f1a->getX() - f1b->getX()) + (f1a->getY() - f1b->getY())
                                 * (f1a->getY() - f1b->getY()));
          float dist2 = sqrt(
                             (f2a->getX() - f2b->getX()) * (f2a->getX() - f2b->getX()) + (f2a->getY() - f2b->getY())
                                 * (f2a->getY() - f2b->getY()));

          relative_motion = (relative_motion < fabs(dist1 - dist2) ? fabs(dist1 - dist2) : relative_motion);
        }
      }

      if (relative_motion <= minDistance)
      {
        adjMatrix[i][j]->setCapacity(predictorName, 1.0);
        adjMatrix[j][i]->setCapacity(predictorName, 1.0);
      }
      else if (relative_motion <= 2 * this->minDistance)
      {
        //                adjMatrix[i][j]->setCapacity(predictorName, 0.0);
        //                adjMatrix[j][i]->setCapacity(predictorName, 0.0);
        // Changed: Now it sets a value according to the amount of relative motion
        adjMatrix[i][j]->setCapacity(predictorName, 1 - (relative_motion - this->minDistance) / (this->minDistance));
        adjMatrix[j][i]->setCapacity(predictorName, 1 - (relative_motion - this->minDistance) / (this->minDistance));
      }
    }
  }
}

}
;
