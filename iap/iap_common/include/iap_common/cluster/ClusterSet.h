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
#ifndef CLUSTERSET_H_
#define CLUSTERSET_H_

#include "feature.h"
#include "Cluster.h"
#include <boost/shared_ptr.hpp>

namespace vision
{

class ClusterSet;
typedef boost::shared_ptr<ClusterSet> ClusterSetPtr;

class ClusterTrajectorySet;

/*
 * ClusterSet
 *
 * A ClusterSet is a set of clusters (vector of set of features)
 * at a fixed time frame
 *
 *  Created on: Dec 20, 2011
 *      Author: rmartin + shoefer
 */
class ClusterSet
{

public:

  /**
   * Constructor
   */
  ClusterSet();

  /**
   * Copy constructor
   * @param cs - ClusterSet to be copied
   */
  ClusterSet(const ClusterSet & cs);

  /**
   * Constructor using a FeatureSet
   * @param fs - FeatureSet to be clustered
   * @param clusters - Id's of the clusters
   */
  ClusterSet(const FeatureSetPtr &fs, const std::vector<int> & clusters);

  /**
   * Destructor
   */
  virtual ~ClusterSet();

  /**
   * Clone function
   * @return - COPY of this ClusterSet and its Clusters
   */
  ClusterSetPtr clone() const;

  /**
   * Add a Cluster to the Set of Clusters
   * @param c - Cluster to be added
   * @return - True if the adding operation was successful
   */
  virtual bool addCluster(const ClusterPtr& c);

  /**
   * Get the Id of the cluster that contains the Feature of interest
   * @param feature_id - Id of the Feature to be found
   * @return - Id of the Cluster that contains the Feature of interest. -1 if the Feature is not contained by any Cluster in this Set
   */
  virtual int getClusterId(const int feature_id) const;

  /**
   * @brief Returns a cluster, i.e. a set of features belonging to cluster cluster_id
   *  at time frame_nr of this ClusterSet
   *  @param cluster_id - Id of the Cluster to make a copy of
   *  @return - COPY of the Cluster with the desired Id
   */
  virtual ClusterPtr getCluster(const int cluster_id) const;

  /**
   * Get the number of different Clusters in this Set
   * @return - Number of Clusters in this Set
   */
  virtual int getNumberOfClusters() const;

  /**
   * Creates a vector < FeaturePtr > from this ClusterSet
   * @return - vector of Features with a COPY of the contents of this ClusterSet
   */
  virtual std::vector<FeaturePtr> ClusterSet2Vector() const;

  /**
   * Creates a vector < FeaturePtr > from one Cluster of this ClusterSet
   * @param cluster_id - Id of the Cluster to convert into a vector of Features
   * @return - vector of Features with a COPY of the contents of one Cluster of this ClusterSet
   */
  virtual std::vector<FeaturePtr> Cluster2Vector(const int cluster_id) const;

protected:

  std::vector<ClusterPtr> _clusters;

  virtual ClusterSet* doClone() const
  {
    return (new ClusterSet(*this));
  }

public:
  // Iterators /////////////////////////////////////////////////////////////////////////
  typedef std::vector<ClusterPtr>::iterator iterator;
  typedef std::vector<ClusterPtr>::const_iterator const_iterator;

  iterator begin()
  {
    return _clusters.begin();
  }

  iterator end()
  {
    return _clusters.end();
  }

  const_iterator begin() const
  {
    return _clusters.begin();
  }

  const_iterator end() const
  {
    return _clusters.end();
  }

  // Printing Functions ////////////////////////////////////////////////////////////////////
  virtual void printCS() const;

};

}

#endif /* CLUSTERSET_H_ */
