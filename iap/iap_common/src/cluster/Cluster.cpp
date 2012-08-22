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
 * Cluster.cpp
 *
 *  Created on: Dec 20, 2011
 *      Author: rmartin + shoefer
 */

#include "Cluster.h"

namespace vision
{

Cluster::Cluster(unsigned int frame, int cluster_id) :
  FeatureSet(frame), _cluster_id(cluster_id)
{
}

Cluster::Cluster(const sensor_msgs::PointCloud2 &feature_set) :
  FeatureSet(feature_set)
{
}

Cluster::Cluster(const FeatureSetPtr& fs, int cluster_id) :
  FeatureSet(fs->getFrame()), _cluster_id(cluster_id)
{
  FeatureSet::const_iterator fs_it = fs->begin();
  FeatureSet::const_iterator fs_it_end = fs->end();
  for (; fs_it != fs_it_end; fs_it++)
  {
    this->addFeature((*fs_it));
  }
}

Cluster::Cluster(const Cluster& c) :
  FeatureSet(c)
{
  this->_cluster_id = c.getClusterId();
}

Cluster::~Cluster()
{
}

int Cluster::getClusterId() const
{
  return this->_cluster_id;
}

void Cluster::setClusterId(const int cluster_id)
{
  this->_cluster_id = cluster_id;
}

void Cluster::printC() const
{
  std::cout << "Cluster " << this->_cluster_id << " (calling FeatureSet::printFS()): " << std::endl;
  FeatureSet::printFS();
}

ClusterPtr Cluster::clone() const
{
  return (ClusterPtr(doClone()));
}

}
