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
 * RelativeMotion3D.cpp
 *
 *  Created on: May 7, 2012
 *      Author: roberto
 */

#include "Graph.h"
#include <iostream>
#include "RelativeMotion3D.h"
#include <time.h>
#include <cmath>

using namespace std;
using namespace vision;

namespace VisualGraph
{

RelativeMotion3D::RelativeMotion3D(double minDistance) :
  Predictor(string("RelativeMotion"))
{
  this->minDistance = minDistance;
}

RelativeMotion3D::~RelativeMotion3D(void)
{
}

void RelativeMotion3D::compute()
{
  for (int i = 0; i < numVertices; i++)
  {
    for (int j = i + 1; j < numVertices; j++)
    {
      double relative_motion = 0;
      for (int f1 = frame1; f1 < frame2; f1+=4)
      {
        for (int f2 = f1; f2 < frame2; f2+=4)
        {
          FeaturePtr featA_frame1 = vList[i]->getFeatureData(f1);
          FeaturePtr featB_frame1 = vList[j]->getFeatureData(f1);
          FeaturePtr featA_frame2 = vList[i]->getFeatureData(f2);
          FeaturePtr featB_frame2 = vList[j]->getFeatureData(f2);

          //the distance between the two features in frame1 and in frame2
          float dist1 = sqrt(
                             pow((featA_frame1->getX() - featB_frame1->getX()), 2)
                                 + pow((featA_frame1->getY() - featB_frame1->getY()), 2)
                                 + pow((featA_frame1->getZ() - featB_frame1->getZ()), 2));
          float dist2 = sqrt(
                             pow((featA_frame2->getX() - featB_frame2->getX()), 2)
                                 + pow((featA_frame2->getY() - featB_frame2->getY()), 2)
                                 + pow((featA_frame2->getZ() - featB_frame2->getZ()), 2));

          relative_motion = (relative_motion < fabs(dist1 - dist2) ? fabs(dist1 - dist2) : relative_motion);
        }
      }

      if (relative_motion <= minDistance)
      {
        adjMatrix[i][j]->setCapacity(predictorName, 1.0);
        adjMatrix[j][i]->setCapacity(predictorName, 1.0);
      }
      else
      {
        adjMatrix[i][j]->setCapacity(predictorName, 0.);
        adjMatrix[j][i]->setCapacity(predictorName, 0.);
      }
    }
  }
}

}
;

