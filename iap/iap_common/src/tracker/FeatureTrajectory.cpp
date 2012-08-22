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
 * FeatureTrajectory.cpp
 *
 *  Created on: Nov 28, 2011
 *      Author: shoefer
 */

#include "FeatureTrajectory.h"

#include "MathFunctions.h"

namespace vision
{

FeatureTrajectory::FeatureTrajectory()
{
}

FeatureTrajectory::FeatureTrajectory(const FeatureTrajectory& ft)
{
  FeatureTrajectory::const_iterator ft_it = ft.begin();
  FeatureTrajectory::const_iterator ft_it_end = ft.end();
  for(; ft_it != ft_it_end; ft_it++)
  {
    this->addFeature((*ft_it)->clone());
  }
}

FeatureTrajectoryPtr FeatureTrajectory::clone() const
{
  return FeatureTrajectoryPtr(doClone());
}

FeatureTrajectory::~FeatureTrajectory()
{
}

bool FeatureTrajectory::addFeature(const FeaturePtr & f)
{
  if (_trajectory.size() != 0 && f->getId() != _trajectory[0]->getId())
  {
    printf("The Id of Feature to be added (%d) doesn't match the id of this Feature Trajectory (%d).", f->getId(),
           _trajectory[0]->getId());
    return false;
  }
  _trajectory.push_back(f);
  return true;
}

FeaturePtr FeatureTrajectory::getFeature(const unsigned int frame_nr) const
{
  if (frame_nr >= this->getTrajectoryLength())
  {
    return FeaturePtr();
  }
  return _trajectory[frame_nr]->clone();

}

unsigned int FeatureTrajectory::getTrajectoryLength() const
{
  return _trajectory.size();
}

void FeatureTrajectory::printFT() const
{
  std::cout << "Feature Trajectory:" << std::endl;
  int index = 0;
  for (FeatureTrajectory::const_iterator it = _trajectory.begin(); it != _trajectory.end(); it++)
  {
    std::cout << "Frame " << index << " : " << *it << std::endl;
    index++;
  }
}

void FeatureTrajectory::printF(const int frame_number) const
{
  FeaturePtr feature = this->getFeature(frame_number);
  if (feature)
  {
    std::cout << "Feature " << feature->getId() << " in frame " << frame_number << " : " << feature << std::endl;
  }
  else
  {
    std::cout << "ERROR[FeatureTrajectory::print(const int frame_number)]: This frame is not in the Feature Trajectory."
        << std::endl;
  }

}

void FeatureTrajectory::writeToFileFT(std::string file_name) const
{
  std::ofstream file_to_write;
  file_to_write.open(file_name.c_str(), std::ios_base::trunc);
  if (!file_to_write.is_open())
  {
    std::cout << "ERROR[FeatureTrajectory::writeToFile(std::string file_name)]: The file couldn't be opened." << std::endl;
    return;
  }
  for (FeatureTrajectory::const_iterator it = _trajectory.begin(); it != _trajectory.end(); it++)
  {
    file_to_write << *it << std::endl;
  }
  file_to_write.close();
}

void FeatureTrajectory::appendToFileFT(std::string file_name) const
{
  std::ofstream file_to_write;
  file_to_write.open(file_name.c_str(), std::ios::out | std::ios::app);
  if (!file_to_write.is_open())
  {
    std::cout << "ERROR[FeatureTrajectory::writeToFile(std::string file_name)]: The file couldn't be opened." << std::endl;
    return;
  }
  // We add a first end-of-line to the file to separate the previous content
  file_to_write << std::endl;
  for (FeatureTrajectory::const_iterator it = _trajectory.begin(); it != _trajectory.end(); it++)
  {
    file_to_write << *it << std::endl;
  }
  file_to_write.close();

}

double FeatureTrajectory::estimateAmountOfMotion() const
{
  double sum = 0.0;
  for (FeatureTrajectory::const_iterator it = _trajectory.begin() + 1; it != _trajectory.end(); it++)
  {
    sum += math::abs((*(it - 1)), (*it));
    //std::cout << sum << std::endl;
  }
  return sum;
}

int FeatureTrajectory::findFrameOfMaxDistance(int comparing_frame) const
{
  std::vector<double> distances;
  if(comparing_frame > _trajectory.size())
  {
    return -1;
  }
  FeaturePtr comparing_feature = _trajectory.at(comparing_frame)->clone();
  for (int i = 0; i < this->_trajectory.size(); i++)
  {
    if (i != comparing_frame)
    {
      FeaturePtr compared_feature = _trajectory.at(i)->clone();
      double distance = math::abs(comparing_feature, compared_feature);
      distances.push_back(distance);
    }
    else
    {
      distances.push_back(0);
    }
  }
  int ret_val;
  math::max(distances, ret_val);
  return ret_val;
}

std::vector<FeaturePtr>& FeatureTrajectory::featureTrajectory2vector() const
{
  std::vector<FeaturePtr> ret_val;
  std::vector<FeaturePtr>::const_iterator f_it = this->_trajectory.begin();
  std::vector<FeaturePtr>::const_iterator f_it_end = this->_trajectory.end();
  for(; f_it != f_it_end; f_it++)
  {
    ret_val.push_back((*f_it)->clone());
  }
  return ret_val;
}

}
