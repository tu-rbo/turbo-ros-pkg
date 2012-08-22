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
#include "Predictor.h"
#include <iostream>

using namespace std;
using namespace vision;

namespace VisualGraph
{

Predictor::Predictor(string name)
{
  this->frame1 = -1;
  this->frame2 = -1;
  this->predictorName = name;
}

Predictor::~Predictor(void)
{
}

void Predictor::setEdgeList(Edge ***adjMatrix)
{
  this->adjMatrix = adjMatrix;
}

void Predictor::setVertexList(Vertex **vList)
{
  this->vList = vList;
}

void Predictor::setNumberVertices(int numVertices)
{
  this->numVertices = numVertices;
}

void Predictor::setFirstFrame(int first_frame)
{
  this->frame1 = first_frame;
}

void Predictor::setLastFrame(int last_frame)
{
  this->frame2 = last_frame;
}

}
;
