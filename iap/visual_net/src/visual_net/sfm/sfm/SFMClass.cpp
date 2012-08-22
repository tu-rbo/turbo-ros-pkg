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
 * SFMClass.cpp
 *
 *  Created on: 19.08.2010
 *      Author: dermax
 */

#include <SFMClass.h>
#include "ros/ros.h"

namespace SFM
{

SFMClass::SFMClass(Initializer *initializer, Estimator *estimator)
{
  this->estimator = estimator;
  this->initializer = initializer;
}

SFMClass::~SFMClass()
{
}

std::vector<std::vector<vision::FeaturePtr> > SFMClass::run(std::vector<std::vector<vision::FeaturePtr> > &features,
                                                            bool &moving)
{
  std::vector<std::vector<vision::FeaturePtr> > estimated_3d_structure;

  // If the 2D Features don't move enough we create a 3D structure using the default depth as z coord
  // if (initializer->isStatic(features))
  // TODO: New! We don't waste our time if we can't run Bundler, we give z=1 and don't run EKF
  if (initializer->isStatic(features) || features.at(0).size() < 20)
  {
    ROS_INFO("[SFMClass::run] Static or too small cluster");
    moving = false;
    estimated_3d_structure = initializer->getStaticStructure(features);
  }
  else          // If there is enough 2D motion of the Features we will run the Initializer to get a first estimation and then the Estimator to refine it
  {
    std::vector<vision::FeaturePtr> initial_3d_structure;
    // Run the Initializer with different sets of parameters until we find a set that works well
    for (int i = 0; i < initializer->getNumSettings(); i++)
    {
      initializer->setSetting(i);
      initial_3d_structure = initializer->run(features);
      if (initial_3d_structure.size() == features.at(0).size())
      {
        ROS_INFO("[SFMClass::run] Initial structure found!");
//        ROS_INFO("[SFMClass::run] Initial structure: ");
//        std::vector<vision::FeaturePtr>::iterator it = initial_3d_structure.begin();
//        std::vector<vision::FeaturePtr>::iterator it_end = initial_3d_structure.end();
//        for (; it != it_end; it++)
//        {
//          std::cout << (*it)->getX() << " " << (*it)->getY() << " " << (*it)->getZ() << std::endl;
//        }
        moving = true;
        break;
      }
    }
    estimated_3d_structure = estimator->run(initial_3d_structure, features);
  }
  return estimated_3d_structure;
}

std::vector<std::vector<vision::FeaturePtr> > SFMClass::run(std::vector<std::vector<vision::FeaturePtr> > &features,
                                                            std::vector<vision::FeaturePtr> &previous_init_structure)
{
  ROS_INFO("[SFMClass::run] Using previous estimation");
  std::vector<std::vector<vision::FeaturePtr> > estimated_3d_structure = estimator->run(previous_init_structure, features);
  if(estimated_3d_structure.size() == 0)
  {
//    ROS_INFO("[SFMClass::run] The initial structure is wrong. Try to find a new one.");
//    bool moving = false;
//    estimated_3d_structure = this->run(features, moving);
    //TODO: What is better? The old method (trying to run bundler and ekf again) or the new one (supposse the cluster is static)?
    ROS_INFO("[SFMClass::run] The initial structure is wrong. Considering the cluster as static.");
    estimated_3d_structure = initializer->getStaticStructure(features);
  }
  return estimated_3d_structure;
}

}
