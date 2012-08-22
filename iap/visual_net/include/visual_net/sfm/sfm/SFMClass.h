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
 * SFMClass.h
 *
 *  Created on: 19.08.2010
 *      Author: dermax
 */

#ifndef SFMCLASS_H_
#define SFMCLASS_H_

#include "Initializer.h"
#include "Estimator.h"

namespace SFM
{
/**
 * @brief	Combines the functionality of Initializer and Estimator
 */
class SFMClass
{
public:
  /**
   * Constructor
   * @param initializer - Pointer to the Initializer object
   * @param estimator - Pointer to the Estimator object
   */
  SFMClass(Initializer *initializer, Estimator *estimator);

  /**
   * Destructor
   */
  virtual ~SFMClass();

  /**
   * Run the SfM procedure using the Feature Trajectories given. First it executes the Initializer (if the Features moved enough to
   * execute it) and then it refines the first estimation using the Estimator
   * @param features - Pixel coordinates of all features in every frame
   * @param moving - Flag to know if the Features moved enough to run the Initializer algorithm or if the result is the static result
   * @return - xyz position of the features in every frame with some scale factor
   */
  std::vector<std::vector<vision::FeaturePtr> > run(std::vector<std::vector<vision::FeaturePtr> > &features,
                                                    bool &moving);

  /**
   * Run the SfM procedure using the Feature Trajectories given. It uses the initial structure given, instead of using the Initializer
   * object to generate it.
   * @param features - Pixel coordinates of all features in every frame
   * @param previous_init_structure - 3D coordinates in the last frame of the previous iteration
   * @return - xyz position of the features in every frame with some scale factor
   */
  std::vector<std::vector<vision::FeaturePtr> > run(std::vector<std::vector<vision::FeaturePtr> > &features,
                                                    std::vector<vision::FeaturePtr> &previous_init_structure);

private:
  Initializer *initializer;
  Estimator *estimator;
};

}

#endif /* SFMCLASS_H_ */
