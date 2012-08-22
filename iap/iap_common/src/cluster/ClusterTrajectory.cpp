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
 * ClusterTrajectory.cpp
 *
 *  Created on: Dec 20, 2011
 *      Author: shoefer
 */

#include "ClusterTrajectory.h"
#include "MathFunctions.h"

#include <numeric>
#include <boost/algorithm/string.hpp>

namespace vision
{

ClusterTrajectory::ClusterTrajectory()
{
}

ClusterTrajectory::ClusterTrajectory(const ClusterTrajectory & ct) :
  FeatureTrajectorySet(ct)
{
  this->_cluster_id = ct.getClusterId();
}

ClusterTrajectory::~ClusterTrajectory()
{
}

int ClusterTrajectory::getClusterId() const
{
  return this->_cluster_id;
}

void ClusterTrajectory::setClusterId(const int cluster_id)
{
  this->_cluster_id = cluster_id;
}

void ClusterTrajectory::printCT() const
{
  std::cout << "Trajectory of Cluster " << this->_cluster_id << " (calling FeatureSet::printFS()): " << std::endl;
  FeatureTrajectorySet::printFTS();
}

void ClusterTrajectory::writeToFileCT(std::string file_name) const
{
  std::ofstream file_to_write;
  file_to_write.open(file_name.c_str(), std::ios_base::trunc);
  if (!file_to_write.is_open())
  {
    std::cout << "ERROR[ClusterTrajectory::writeToFileCT(std::string file_name)]: The file couldn't be opened." << std::endl;
    return;
  }

  file_to_write << getClusterId();
  file_to_write << std::endl;

  // now add the FeatureTrajectorySet
  appendToFileFTS(file_name);
}

void ClusterTrajectory::appendToFileCT(std::string file_name) const
{
  std::ofstream file_to_write;
  file_to_write.open(file_name.c_str(), std::ios::out | std::ios::app);
  if (!file_to_write.is_open())
  {
    std::cout << "ERROR[ClusterTrajectory::appendToFileCT(std::string file_name)]: The file couldn't be opened." << std::endl;
    return;
  }

  file_to_write << getClusterId();
  file_to_write << std::endl;

  // now add the FeatureTrajectorySet
  appendToFileFTS(file_name);
}

void ClusterTrajectory::readFromFileCT(std::string file_name)
{
  std::ifstream file_to_read;
  file_to_read.open(file_name.c_str(), std::ios::in);

  readFromFileCT(file_to_read);
}

void ClusterTrajectory::readFromFileCT(std::ifstream& file_to_read)
{
  std::string line;
  getline(file_to_read, line);

  // read until eof or first empty line
  boost::algorithm::trim(line);

  setClusterId(atoi(line.c_str()));
  readFromFileFTS(file_to_read);
}


BodyTrajectoryPtr ClusterTrajectory::estimateBodyTrajectory(int start_frame, int end_frame) const
{
  if (end_frame > this->getTrajectoryLength() - 1) {
    ROS_ERROR_STREAM_NAMED("ClusterTrajectory.estimateBodyTrajectory",
                           "end_frame must not be higher thant the last frame");
    return BodyTrajectoryPtr();
  }

  if(end_frame == 0)
  {
    end_frame = this->getTrajectoryLength()-1;
  }
  std::vector<RigidTransformationPtr> transformations;
  RigidTransformationPtr initial_guess(new RigidTransformation(Eigen::Matrix4f::Identity()));
  RigidTransformationPtr current_rbt(new RigidTransformation(Eigen::Matrix4f::Identity()));
  for(int frame = start_frame; frame <= end_frame; frame++)
  {
    ROS_DEBUG_STREAM_NAMED("ClusterTrajectory.estimateBodyTrajectory", "ClusterId: " << this->_cluster_id << ", second frame: " << frame);
    current_rbt = this->getFeatureSet(start_frame)->estimateRigidTransformation(this->getFeatureSet(frame), initial_guess);
    transformations.push_back(current_rbt);
    initial_guess = current_rbt;
  }
  return BodyTrajectoryPtr(new BodyTrajectory(start_frame, transformations, this->_cluster_id));
}

double ClusterTrajectory::estimateAmountOfMotion() const
{
  std::vector<double> sums;
  for (FeatureTrajectory::const_iterator it = (*this->begin())->begin(); it != (*this->begin())->end(); it++)
  {
    sums.push_back((this->getFeatureTrajectory((*it)->getId()))->estimateAmountOfMotion());
  }

  double median_motion = math::max(sums);
  return median_motion / sums.size();

}

ClusterPtr ClusterTrajectory::getCluster(const unsigned int frame_nr) const
{
  ClusterPtr ret_val;
  if (frame_nr >= this->_feature_sets.size())
  {
    ret_val = ClusterPtr(new Cluster(0, 0));
    return ret_val;
  }
  ret_val = boost::dynamic_pointer_cast<Cluster>(this->_feature_sets[frame_nr]->clone());
  ret_val->setClusterId(this->_cluster_id);
  return ret_val;
}

ClusterTrajectoryPtr ClusterTrajectory::clone() const
{
  return ClusterTrajectoryPtr(doClone());
}

bool ClusterTrajectory::addCluster(const ClusterPtr& cluster)
{
  if (this->_cluster_id != cluster->getClusterId())
  {
    ROS_ERROR("[ClusterTrajectory::add] The Cluster to be added has a different cluster id (%d) than the other Clusters of this ClusterTrajectory (%d)", cluster->getClusterId(), this->_cluster_id);
    return false;
  }
  if (this->getNumberOfFeatures())
  {
    if ((*this->begin())->getNumberOfFeatures() != cluster->getNumberOfFeatures())
    {
      ROS_ERROR("[ClusterTrajectory::add] The Cluster to be added has a different number of Features (%d) than the other Clusters of this ClusterTrajectory (%d)", cluster->getNumberOfFeatures(), (*this->begin())->getNumberOfFeatures());
      return false;
    }
  }
  this->_feature_sets.push_back(cluster);
  return true;
}

}
