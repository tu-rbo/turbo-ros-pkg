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
#ifndef CLUSTER_H_
#define CLUSTER_H_

#include "feature.h"
#include "FeatureSet.h"
#include "sensor_msgs/PointCloud2.h"
#include <boost/shared_ptr.hpp>

namespace vision {
class Cluster;
typedef boost::shared_ptr<Cluster> ClusterPtr;

/*
 * Cluster
 *
 * A cluster is a set of Features all belonging to the group (cluster)
 * at a fixed time frame
 *
 * It is basically a FeatureSet + a cluster ID.
 *
 *  Created on: Dec 20, 2011
 *      Author: rmartin + shoefer
 */
class Cluster : public FeatureSet
{
public:
  /**
   * Constructor
   * @param frame - Frame in which this cluster is defined
   * @param cluster_id - Identifier of the cluster
   */
  Cluster(unsigned int frame, int cluster_id);

  /**
   * Constructor from a ROS PointCloud2 message
   * @param feature_set - ROS PointCloud2 message
   */
  Cluster(const sensor_msgs::PointCloud2 &feature_set);

  /**
   * Constructor from a FeatureSet
   * @param fs - FeatureSet
   * @param cluster_id
   */
  Cluster(const FeatureSetPtr& fs, int cluster_id);

  /**
   * Copy constructor
   * @param c - Cluster to be copied
   */
  Cluster(const Cluster& c);

  /**
   * Destructor
   */
  virtual ~Cluster();

  /**
   * Get function of the Cluster Identifier
   * @return - Cluster Identifier
   */
  virtual int getClusterId() const;

  /**
   * Set function of the Cluster Identifier
   * @param cluster_id - New Cluster Identifier
   */
  virtual void setClusterId(const int cluster_id);

  /**
   * Print the contains of the Cluster
   */
  virtual void printC() const;

  /**
   * Clone this Cluster and return the copy (not named clone() because of the similar function of the base class that returns a FeatureSetPtr)
   * @return - Copy of this Cluster
   */
  ClusterPtr clone() const;

protected:
  int _cluster_id;      // Cluster Identifier

  virtual Cluster* doClone() const
  {
    return (new Cluster(*this));
  }

public:
  // Iterators //////////////////////////////////////////////////////
  typedef std::vector<FeaturePtr>::iterator iterator;
  typedef std::vector<FeaturePtr>::const_iterator const_iterator;

};

}

#endif /* CLUSTER_H_ */
