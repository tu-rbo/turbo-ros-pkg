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
 * Estimator.h
 *
 *  Created on: 28.07.2010
 *      Author: dermax
 */

#ifndef ESTIMATOR_H_
#define ESTIMATOR_H_

#include "feature.h"
#include "CameraModel.h"

namespace SFM
{
/**
 * The Estimator class should return a vector of xyz positions for all features in all frames.
 */
class Estimator
{
public:
  /**
   * Constructor
   * @param camera - A model of the camera used to record the features
   */
  Estimator(vision::CameraModel &camera);

  /**
   * Destructor
   */
  virtual ~Estimator();

  /**
   * Run the Estimator. It iterates over all Features and all frames using the initial structure as initial 3D value
   * and refining it in successive frames
   * @param init_structure - xyz positions for the features in the first frame
   * @param features - pixel position of the features in all frames
   * @return - xyz features of every Feature in every frame
   */
  virtual std::vector<std::vector<vision::FeaturePtr> > run(std::vector<vision::FeaturePtr> &init_structure,
                                                            std::vector<std::vector<vision::FeaturePtr> > &features)=0;
protected:
  vision::CameraModel cam;

};

}

#endif /* ESTIMATOR_H_ */
