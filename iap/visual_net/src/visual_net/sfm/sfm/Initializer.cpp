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
 * Initalizer.cpp
 *
 *  Created on: 28.07.2010
 *      Author: dermax
 */

#include "Initializer.h"
#include "ros/ros.h"

using namespace vision;

namespace SFM
{

Initializer::Initializer(CameraModel &camera, double min_motion_pix)
{
  cam = camera; // save camera model
  motion_threshold = 30.0 / (double)cam.getXresolution(); // used for isStatic()
  default_depth = 1.0; // default z position for a static cluster
  num_settings = 0;
}

Initializer::~Initializer()
{
}

int Initializer::getNumSettings()
{
  return num_settings;
}

bool Initializer::isStatic(std::vector<std::vector<vision::FeaturePtr> > &features)
{
  // save the first frame
  std::vector<vision::FeaturePtr> last_used_frame = features[0];
  // for all frames
  int frame_difference = 0;
  double max_dif = 0;

  std::vector<std::vector<vision::FeaturePtr> >::iterator it = features.begin();
  std::vector<std::vector<vision::FeaturePtr> >::iterator it_end = features.end();
  for (; it != it_end; it++)
  {
    double dif = 0.0;
    // for all features
    for (size_t i = 0; i < it->size(); i++)
    {
      double x1 = (*it)[i]->getX();
      double x2 = last_used_frame[i]->getX();
      double y1 = (*it)[i]->getY();
      double y2 = last_used_frame[i]->getY();
      dif += sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }
    // calculate mean distance
    dif = dif / (double)it->size();
    // check threshold
    max_dif = (dif > max_dif) ? dif : max_dif;
    if (dif > motion_threshold * cam.getXresolution())
    {
      ROS_INFO("[Initializer::isStatic] Enough motion of cluster = %f", dif);
      return false;
    }
    frame_difference++;
    // go to next frame
  }
  ROS_INFO("[Initializer::isStatic] NOT enough motion of cluster = %f and minimum motion = %f", max_dif, motion_threshold * cam.getXresolution());
  return true;
}

std::vector<std::vector<vision::FeaturePtr> > Initializer::getStaticStructure(
                                                                              std::vector<std::vector<
                                                                                  vision::FeaturePtr> > &features)
{
  std::vector<std::vector<vision::FeaturePtr> > ret_feats_traj;
  for (size_t i = 0; i < features.size(); i++)
  {
    std::vector<vision::FeaturePtr> one_frame;
    std::vector<vision::FeaturePtr>::iterator it = features[0].begin();
    std::vector<vision::FeaturePtr>::iterator it_end = features[0].end();
    for (; it != it_end; it++)
    {
      FeaturePtr one_feat = (*it)->cloneAndUpdate(
                                                  (float)(-((*it)->getX() - cam.getXresolution() / 2) * this->default_depth
                                                      / cam.getFocalLength()),
                                                  (float)(((*it)->getY() - cam.getYresolution() / 2) * this->default_depth
                                                      / cam.getFocalLength()), this->default_depth);
      one_feat->setLost(true);
      one_frame.push_back(one_feat);
    }
    ret_feats_traj.push_back(one_frame);
  }
  return ret_feats_traj;
}

std::vector<std::vector<vision::FeaturePtr> > Initializer::continueStaticStructure(
                                                                                   std::vector<std::vector<
                                                                                       vision::FeaturePtr> > &features,
                                                                                   std::vector<vision::FeaturePtr> &previous_init_structure)
{
  std::vector<std::vector<vision::FeaturePtr> > result;
  for (size_t i = 0; i < features.size(); i++)
  {
    std::vector<vision::FeaturePtr> result_frame = previous_init_structure;
    result.push_back(result_frame);
  }
  return result;
}

void Initializer::setMotionThreshold(double threshold)
{
  motion_threshold = threshold;
}

void Initializer::setDefaultDepth(double depth)
{
  default_depth = depth;
}

double Initializer::getDefaultDepth()
{
  return default_depth;
}

}

