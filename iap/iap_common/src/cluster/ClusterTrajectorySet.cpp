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
 * ClusterTrajectorySet
 *
 *  Created on: Nov 28, 2011
 *      Author: shoefer
 */

#include "ClusterTrajectorySet.h"

namespace vision
{

ClusterTrajectorySet::ClusterTrajectorySet(const ClusterTrajectorySet & cts)
{
  ClusterTrajectorySet::const_iterator cts_it = cts.begin();
  ClusterTrajectorySet::const_iterator cts_it_end = cts.end();
  for (; cts_it != cts_it_end; cts_it++)
  {
    this->addClusterTrajectory((*cts_it)->clone());
  }
}

ClusterTrajectorySet::ClusterTrajectorySet(const FeatureTrajectorySetPtr &fts, const std::vector<int> & clusters)
{
  // check sizes
  if (!clusters.size() == fts->getNumberOfFeatures())
  {
    const char* msg =
        "[ClusterTrajectorySet] clusters does not contain the same amount of features as the FeatureTrajectorySet";
    ROS_ERROR(msg);
    throw msg;
  }

  // Populate the vector of ClusterTrajectories
  std::vector<int>::const_iterator vector_int_it = clusters.begin();
  std::vector<int>::const_iterator vector_int_it_end = clusters.end();
  for (; vector_int_it != vector_int_it_end; vector_int_it++)
  {
    if ((*vector_int_it) != -1)
    {
      bool has_a_ct = false;
      ClusterTrajectorySet::iterator cts_it = this->begin();
      ClusterTrajectorySet::iterator cts_it_end = this->end();
      for (; cts_it != cts_it_end; cts_it++)
      {
        if ((*cts_it)->getClusterId() == (*vector_int_it))
        {
          has_a_ct = true;
          break;
        }
      }
      if (!has_a_ct)
      {
        ClusterTrajectoryPtr ct_temp = ClusterTrajectoryPtr(new ClusterTrajectory());
        ct_temp->setClusterId((*vector_int_it));
        this->addClusterTrajectory(ct_temp);
      }
    }
  }

  // Populate each ClusterTrajectory of the vector
  FeatureTrajectorySet::const_trajectory_set_iterator fts_it = fts->begin();
  FeatureTrajectorySet::const_trajectory_set_iterator fts_it_end = fts->end();
  int frame_ctr = 0;
  for (; fts_it != fts_it_end; fts_it++, frame_ctr++)
  {
    ClusterTrajectorySet::iterator cts_it = this->begin();
    ClusterTrajectorySet::iterator cts_it_end = this->end();
    for (; cts_it != cts_it_end; cts_it++)
    {
      int cluster_id = (*cts_it)->getClusterId();
      ClusterPtr c_temp = ClusterPtr(new Cluster(frame_ctr, cluster_id));
      for (int feature_offset = 0; feature_offset < clusters.size(); feature_offset++)
      {
        if (clusters.at(feature_offset) == cluster_id)
        {
          c_temp->addFeature((*((*fts_it)->begin() + feature_offset))->clone());
        }
      }
      (*cts_it)->addCluster(c_temp);
    }
  }
}

ClusterTrajectorySet::ClusterTrajectorySet()
{

}

ClusterTrajectorySet::~ClusterTrajectorySet()
{

}

int ClusterTrajectorySet::getClusterId(const int feature_id) const
{
  ClusterTrajectorySet::const_iterator cts_it = this->begin();
  ClusterTrajectorySet::const_iterator cts_it_end = this->end();
  for (; cts_it != cts_it_end; cts_it++)
  {
    if (((*cts_it)->isContained(feature_id)))
    {
      return (*cts_it)->getClusterId();
    }
  }
}

bool ClusterTrajectorySet::addClusterTrajectory(const ClusterTrajectoryPtr& ct)
{
  if (this->_cluster_trajectories.size() != 0)
  {
    if (this->_cluster_trajectories.at(0)->getTrajectoryLength() != ct->getTrajectoryLength())
    {
      ROS_ERROR("[ClusterTrajectorySet::add] The ClusterTrajectory to be added has a different length (%d) than the other ClusterTrajectories in this ClusterTrajectorySet (%d)",ct->getTrajectoryLength() ,this->_cluster_trajectories.at(0)->getTrajectoryLength());
      return false;
    }
  }
  this->_cluster_trajectories.push_back(ct);
  return true;
}

int ClusterTrajectorySet::getNumberOfClusters() const
{
  return _cluster_trajectories.size();
}

ClusterPtr ClusterTrajectorySet::getCluster(const int frame_nr, const int cluster_id) const
{
  return (this->getClusterTrajectory(cluster_id))->getCluster(frame_nr);
}

ClusterTrajectoryPtr ClusterTrajectorySet::getClusterTrajectory(const int cluster_id) const
{
  ClusterTrajectorySet::const_iterator cts_it = this->begin();
  ClusterTrajectorySet::const_iterator cts_it_end = this->end();
  for (; cts_it != cts_it_end; cts_it++)
  {
    if ((*cts_it)->getClusterId() == cluster_id)
    {
      return (*cts_it)->clone();
    }
  }
  ROS_ERROR("[ClusterTrajectorySet::getClusterTrajectory] The requested cluster with id %d doesn't belong to this ClusterTrajectorySet", cluster_id);
  throw std::string(
                    "[ClusterTrajectorySet::getClusterTrajectory] The requested cluster doesn't belong to this ClusterTrajectorySet");
}

std::vector<ClusterTrajectoryPtr>& ClusterTrajectorySet::getClusterTrajectories() const
{
  std::vector<ClusterTrajectoryPtr> ret_ct_vector;
  ClusterTrajectorySet::const_iterator cts_it = this->begin();
  ClusterTrajectorySet::const_iterator cts_it_end = this->end();
  for (; cts_it != cts_it_end; cts_it++)
  {
    ret_ct_vector.push_back((*cts_it)->clone());
  }
  return ret_ct_vector;
}

ClusterSetPtr ClusterTrajectorySet::getClusterSet(const unsigned int frame_nr) const
{
  if (frame_nr > (*(this->begin()))->getTrajectoryLength())
  {
    ROS_ERROR("[ClusterTrajectorySet::getClusterSet] The requested frame %d doesn't belong to this ClusterTrajectorySet", frame_nr);
    throw std::string(
                      "[ClusterTrajectorySet::getClusterSet] The requested frame doesn't belong to this ClusterTrajectorySet");
  }

  ClusterSetPtr ret_cs = ClusterSetPtr(new ClusterSet());
  ClusterTrajectorySet::const_iterator cts_it = this->begin();
  ClusterTrajectorySet::const_iterator cts_it_end = this->end();
  for (; cts_it != cts_it_end; cts_it++)
  {
    ret_cs->addCluster((*cts_it)->getCluster(frame_nr));
  }

  return ret_cs;
  //  if (frame_nr > this->_cluster_trajectories.begin()->second->getTrajectoryLength())
  //  {
  //    const char* msg = "[getClusterSet] The requested frame doesn't belong to this ClusterTrajectorySet";
  //    ROS_ERROR(msg);
  //    throw msg;
  //  }
  //  ClusterSetPtr ret = ClusterSetPtr(new ClusterSet());
  //  for (std::map<int, ClusterTrajectoryPtr>::const_iterator ct_it = _cluster_trajectories.begin(); ct_it
  //      != _cluster_trajectories.end(); ct_it++)
  //  {
  //    ret->add(boost::dynamic_pointer_cast<Cluster>(ct_it->second->getFeatureSet(frame_nr)));
  //  }
  //  return ret;
}

unsigned int ClusterTrajectorySet::getTrajectoryLength() const
{
  return (*(this->begin()))->getTrajectoryLength();
}

ClusterTrajectorySetPtr ClusterTrajectorySet::clone() const
{
  return ClusterTrajectorySetPtr(doClone());
}

bool ClusterTrajectorySet::isContained(const int & cluster_id) const
{
  ClusterTrajectorySet::const_iterator cts_it = this->begin();
  ClusterTrajectorySet::const_iterator cts_it_end = this->end();
  for (; cts_it != cts_it_end; cts_it++)
  {
    if ((*cts_it)->getClusterId() == cluster_id)
    {
      return true;
    }
  }
  return false;
}

std::vector<BodyTrajectoryPtr> ClusterTrajectorySet::estimateBodyTrajectories(int start_frame, int end_frame) const
{
  std::vector<BodyTrajectoryPtr> ret_val;
  ClusterTrajectorySet::const_iterator cts_it = this->begin();
  ClusterTrajectorySet::const_iterator cts_it_end = this->end();
  for (; cts_it != cts_it_end; cts_it++)
  {
    ret_val.push_back((*cts_it)->estimateBodyTrajectory(start_frame, end_frame));
  }
  return ret_val;
}

void ClusterTrajectorySet::writeToFileCTS(std::string file_name) const
{
  std::ofstream file_to_write;
  file_to_write.open(file_name.c_str(), std::ios_base::trunc);
  if (!file_to_write.is_open())
  {
    std::cout << "ERROR[ClusterTrajectorySet::writeToFileCTS(std::string file_name)]: The file couldn't be opened." << std::endl;
    return;
  }

  std::vector<ClusterTrajectoryPtr>::const_iterator it = _cluster_trajectories.begin();
  for (; it != _cluster_trajectories.end(); it++) {
    (*it)->appendToFileCT(file_name);
  }
}

void ClusterTrajectorySet::readFromFileCTS(std::string file_name)
{
  std::ifstream file_to_read;
  file_to_read.open(file_name.c_str(), std::ios::in);

  readFromFileCTS(file_to_read);
}


void ClusterTrajectorySet::readFromFileCTS(std::ifstream& file_to_read)
{
  int i =0;
  while (!file_to_read.eof())
  {
    ClusterTrajectoryPtr ct(new ClusterTrajectory);
    ct->readFromFileCT(file_to_read);

    if (ct->getTrajectoryLength() != 0)
      addClusterTrajectory(ct);
  }
}

};
