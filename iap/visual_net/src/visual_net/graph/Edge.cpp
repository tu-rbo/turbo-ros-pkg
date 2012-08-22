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
#include "Edge.h"

#include <iostream>
using namespace std;

namespace VisualGraph
{

Edge::Edge(Vertex *vi, Vertex *ve)
{
  this->vi = vi;
  this->ve = ve;
  this->connected = true;
}

Edge::~Edge(void)
{
}

void Edge::setCapacity(string &s, double val)
{
  //if this type of predictor doesn't exist yet in the list, add & assign capacity
  //else update the current capacity
  capacities[s] = val;
  if(val == 0.0)
  {
    this->connected = false;
  }
}

double Edge::getCapacity()
{
  if (!connected)
  {
    return 0.0;
  }

  double capacity = 1.0;
  for (map<string, double>::iterator it = capacities.begin(); it != capacities.end(); it++)
  {
    if(it->second > 1.0)
    {
      std::cout << "ERROR: Capacity set by predictor " << it->first << " is over 0!" << std::endl;
    }
    capacity *= (it->second);
  }
  return capacity;
}

}
;
