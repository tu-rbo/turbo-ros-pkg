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
 * RelativeMotion3D.h
 *
 *  Created on: May 7, 2012
 *      Author: roberto
 */

#ifndef RELATIVEMOTION3D_H_
#define RELATIVEMOTION3D_H_

#include <vector>
#include <string>
#include "Vertex.h"
#include "Edge.h"
#include "Predictor.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

namespace VisualGraph
{

/**
 \class    RelativeMotion3D
 \brief    A predictor.
 It measures the change in distance between 2 features in two different frames
 If the change is small, the features are likely to belong together
 **/
class RelativeMotion3D : public Predictor
{
private:
  double minDistance;
public:
  RelativeMotion3D(double minDistance);
  virtual ~RelativeMotion3D(void);
  virtual void compute();
};
}

#endif /* RELATIVEMOTION3D_H_ */
