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
#include "Triangulation.h"
#include <time.h>
#include <cmath>

using namespace std;
using namespace vision;

namespace VisualGraph
{

Triangulation::Triangulation() :
  Predictor(string("Triangulation"))
{
  this->minDistance = minDistance;
}

Triangulation::~Triangulation(void)
{
}

void Triangulation::compute()
{
  //computes triangulation in frame1
  //analyze violations based on features' position in frame2
  vector<FeaturePtr> features1, features2;
  for (int i = 0; i < numVertices; i++)
  {
    features1.push_back(vList[i]->getFeatureData(frame1));
    features2.push_back(vList[i]->getFeatureData(frame2));
  }

  Triangulate T(features1);
  bool **TRI1 = T.getTriangulation();
  bool **TRI2 = new bool*[numVertices];
  for (int i = 0; i < numVertices; i++)
  {
    TRI2[i] = new bool[numVertices];
  }

  bool **edgesToRemove = T.getIntersectingEdges(features2);

  for (int i = 0; i < numVertices; i++)
  {
    for (int j = i + 1; j < numVertices; j++)
    {
      if (edgesToRemove[i][j])
      {
        TRI2[i][j] = false;
        TRI2[j][i] = false;
      }
      else
      {
        TRI2[i][j] = TRI1[i][j];
        TRI2[j][i] = TRI1[j][i];
      }
    }
  }

  for (int i = 0; i < numVertices; i++)
  {
    for (int j = i + 1; j < numVertices; j++)
    {
      if (!TRI2[i][j])
      {
        //not connected == edge not in triangulation or edge was removed because of crossing
        if (TRI1[i][j])
        {
          //edge was connected before, and now removed because of crossing
          if (adjMatrix[i][j]->isConnected() && adjMatrix[j][i]->isConnected())
          {
            adjMatrix[i][j]->setCapacity(predictorName, 0);
            adjMatrix[j][i]->setCapacity(predictorName, 0);
          }
        }
      }
      else
      {
        //edge was connected and remained connected
        //adjMatrix[i][j]->setCapacity(predictorName, 1);
        //adjMatrix[j][i]->setCapacity(predictorName, 1);
      }
    }
  }

  for (int i = 0; i < numVertices; i++)
  {
    delete (edgesToRemove[i]);
    delete (TRI1[i]);
    delete (TRI2[i]);
  }
  delete (edgesToRemove);
  delete (TRI1);
  delete (TRI2);
}

}
;
