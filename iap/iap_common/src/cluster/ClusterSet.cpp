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
 * ClusterSet.cpp
 *
 *  Created on: Dec 20, 2011
 *      Author: shoefer
 */

#include "ClusterSet.h"

namespace vision
{

ClusterSet::ClusterSet()
{

}

ClusterSet::~ClusterSet()
{
}

ClusterSet::ClusterSet(const ClusterSet & cs)
{
  ClusterSet::const_iterator cs_it = cs.begin();
  ClusterSet::const_iterator cs_it_end = cs.end();
  for (; cs_it != cs_it_end; cs_it++)
  {
    this->addCluster((*cs_it));
  }
}

ClusterSet::ClusterSet(const FeatureSetPtr &fs, const std::vector<int> & clusters)
{

  if (fs->getNumberOfFeatures() != clusters.size())
  {
    ROS_ERROR("[ClusterSet::ClusterSet] Number of Features in the Set and the number of Id's in the vector of integers does NOT match. Number of features: "
        "%d Number of Id's: %d", fs->getNumberOfFeatures(), clusters.size());
    throw std::string(
                      "[ClusterSet::ClusterSet] Number of Features in the Set and the number of Id's in the vector of integers does NOT match.");
  }

  std::map<int, ClusterPtr> clusters_map;
  std::vector<int>::const_iterator int_it = clusters.begin();
  std::vector<int>::const_iterator int_it_end = clusters.end();
  FeatureSet::iterator fs_it = fs->begin();
  FeatureSet::iterator fs_it_end = fs->end();

  for (; int_it != int_it_end && fs_it != fs_it_end; int_it++, fs_it++)
  {
    if ((*int_it) != -1)
    {
      std::map<int, ClusterPtr>::iterator cs_find = clusters_map.find(*(int_it));
      if (cs_find == clusters_map.end())
      {
        ClusterPtr new_cluster = ClusterPtr(new Cluster(fs->getFrame(), *(int_it)));
        new_cluster->addFeature((*fs_it)->clone());
        clusters_map[*(int_it)] = new_cluster;
      }
      else
      {
        cs_find->second->addFeature((*fs_it)->clone());
      }
    }
  }

  std::map<int, ClusterPtr>::iterator c_map_it = clusters_map.begin();
  std::map<int, ClusterPtr>::iterator c_map_it_end = clusters_map.end();
  for (; c_map_it != c_map_it_end; c_map_it++)
  {
    this->_clusters.push_back(c_map_it->second->clone());
  }
}

bool ClusterSet::addCluster(const ClusterPtr& c)
{
  if (_clusters.size() > 0)
  {
    if (c->getFrame() != _clusters.at(0)->getFrame())
    {
      ROS_ERROR("[ClusterSet::add] The cluster to be added is defined in a different frame (%d) "
          "than the previously stored clusters (%d).", c->getFrame(),_clusters.at(0)->getFrame());
      return false;
    }
  }
  this->_clusters.push_back(c);
  return true;
}

int ClusterSet::getClusterId(const int feature_id) const
{
  for (std::vector<ClusterPtr>::const_iterator c_it = this->_clusters.begin(); c_it != _clusters.end(); c_it++)
  {
    if ((*c_it)->getFeature(feature_id))
    {
      return (*c_it)->getClusterId();
    }
  }
  return -1;
}

int ClusterSet::getNumberOfClusters() const
{
  return _clusters.size();
}

std::vector<FeaturePtr> ClusterSet::ClusterSet2Vector() const
{
  std::vector<FeaturePtr> ret_val;
  ClusterSet::const_iterator cs_it = this->begin();
  ClusterSet::const_iterator cs_it_end = this->end();
  for (; cs_it != cs_it_end; cs_it++)
  {
    std::vector<FeaturePtr> temp = (*cs_it)->FeatureSet2Vector();
    ret_val.insert(ret_val.end(), temp.begin(), temp.end());
  }
  return ret_val;
}

std::vector<FeaturePtr> ClusterSet::Cluster2Vector(const int cluster_id) const
{
  ClusterSet::const_iterator cs_it = this->begin();
  ClusterSet::const_iterator cs_it_end = this->end();
  for (; cs_it != cs_it_end; cs_it++)
  {
    if ((*cs_it)->getClusterId() == cluster_id)
    {
      std::vector<FeaturePtr> ret_val = (*cs_it)->FeatureSet2Vector();
      return ret_val;
    }
  }
  ROS_ERROR("[ClusterSet::Cluster2Vector] The Cluster with Id %d is not contained in this ClusterSet.", cluster_id);
}

ClusterPtr ClusterSet::getCluster(const int cluster_id) const
{
  ClusterSet::const_iterator cs_it = this->begin();
  ClusterSet::const_iterator cs_it_end = this->end();
  for (; cs_it != cs_it_end; cs_it++)
  {
    if ((*cs_it)->getClusterId() == cluster_id)
    {
      return ((*cs_it)->clone());
    }
  }
  ROS_ERROR("[ClusterSet::getCluster] The Cluster with Id %d is not contained in this ClusterSet.", cluster_id);
}

void ClusterSet::printCS() const
{
  std::cout << "Cluster Set: " << std::endl;
  std::cout << "Not implemented yet. ???????????" << std::endl;
}

ClusterSetPtr ClusterSet::clone() const
{
  return ClusterSetPtr(doClone());
}

}
