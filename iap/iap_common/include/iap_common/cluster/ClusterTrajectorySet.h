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
#ifndef CLUSTERTRAJECTORYSET_H_
#define CLUSTERTRAJECTORYSET_H_

#include "FeatureTrajectorySet.h"
#include "Cluster.h"
#include "ClusterSet.h"
#include "ClusterTrajectory.h"
#include <boost/shared_ptr.hpp>
#include <map>
#include <vector>
#include <set>
#include <string>

namespace vision
{

class ClusterTrajectorySet;
typedef boost::shared_ptr<ClusterTrajectorySet> ClusterTrajectorySetPtr;

/*
 * ClusterTrajectorySet
 *
 * A ClusterTrajectorySet is a set of features, each over a trajectory and each
 * assigned to a unique cluster.
 *
 * It is a full clustering containing the full trajectories of the features
 *
 *  Created on: Nov 28, 2011
 *      Author: rmartin + shoefer
 */
class ClusterTrajectorySet
{
public:

  /**
   * Default Constructor
   */
  ClusterTrajectorySet();

  /**
   * Copy Constructor
   * @param cts ClusterTrajectorySet to be copied
   */
  ClusterTrajectorySet(const ClusterTrajectorySet & cts);

  /**
   * Constructor from a FeatureTrajectorySet and the ClusterId of every Feature of it
   * @param fts - FeatureTrajectorySet with all Features of the Clusters
   * @param clusters - Ids of the Cluster each Feature belongs to
   */
  ClusterTrajectorySet(const FeatureTrajectorySetPtr &fts, const std::vector<int> & clusters);

  /**
   * Destructor
   */
  virtual ~ClusterTrajectorySet();

  /**
   * Clone Function
   * @return - COPY of this ClusterTrajectorySet
   */
  ClusterTrajectorySetPtr clone() const;

  /**
   * Add a ClusterTrajectory to the Set of ClusterTrajectories
   * @param ct - ClusterTrajectory to be added. It must be defined in the same number frames than the previously stored clusterTrajectories
   * @return - True if the adding operation was successful
   */
  virtual bool addClusterTrajectory(const ClusterTrajectoryPtr& ct);

  /**
   * Get the number of Clusters contained
   * @return - Number of Clusters
   */
  virtual int getNumberOfClusters() const;

  /**
   * Get the Id of the Cluster that contains the Feature of interest
   * @param feature_id - Id of the Feature to be found
   * @return - Id of the Cluster that contains the Feature of interest. -1 if the Feature is not contained in any Cluster
   */
  virtual int getClusterId(const int feature_id) const;

  /**
   * @brief Returns a cluster, i.e. a set of features belonging to cluster cluster_id
   *  at time frame_nr
   */
  virtual ClusterPtr getCluster(const int frame_nr, const int cluster_id) const;

  /**
   * @brief Returns a cluster trajectory, i.e. a set of FeatureTrajectories
   *  belonging to cluster cluster_id
   */
  virtual ClusterTrajectoryPtr getClusterTrajectory(const int cluster_id) const;

  /**
   * @brief Returns a vector with all cluster trajectories, i.e. a vector of sets of FeatureTrajectories
   */
  virtual std::vector<ClusterTrajectoryPtr>& getClusterTrajectories() const;

  /**
   * @brief Returns a cluster set, i.e. a set of Features
   *  assigned to their respective clusters
   */
  virtual ClusterSetPtr getClusterSet(const int unsigned frame_nr) const;

  /**
   * @brief Returns the trajectory length
   *
   * Warning: This method assumes consistency. If the internal state of the trajectory is changed
   *  be sure to run isConsistent
   */
  virtual unsigned int getTrajectoryLength() const;

  /**
   * Check if a Cluster with the given Id is contained in this Set
   * @param cluster_id - Id of the Cluster to be found
   * @return - True if the Cluster is contained in this Set
   */
  virtual bool isContained(const int & cluster_id) const;

  /**
   * Estimate the BodyTrajectory of each Cluster in the set
   * @return - vector containing the body trajectories
   */
  virtual std::vector<BodyTrajectoryPtr> estimateBodyTrajectories(int start_frame=0, int end_frame=0) const;

private:

  std::vector<ClusterTrajectoryPtr> _cluster_trajectories;
  //std::map<int, ClusterTrajectoryPtr> _cluster_trajectories;

  virtual ClusterTrajectorySet* doClone() const
  {
    return (new ClusterTrajectorySet(*this));
  }

public:
  // Iterators ///////////////////////////////////////////////////////////////////
  typedef std::vector<ClusterTrajectoryPtr>::iterator iterator;
  typedef std::vector<ClusterTrajectoryPtr>::const_iterator const_iterator;

  iterator begin()
  {
    return _cluster_trajectories.begin();
  }

  iterator end()
  {
    return _cluster_trajectories.end();
  }

  const_iterator begin() const
  {
    return _cluster_trajectories.begin();
  }

  const_iterator end() const
  {
    return _cluster_trajectories.end();
  }

  /**
   * Write the cluster trajectory set into the file
   *
   * The write methods of cluster trajectory are used
   *
   * @see appendToFileCT
   */
  virtual void writeToFileCTS(std::string file_name) const;

  /**
   * Read a cluster trajectory from a file
   *
   * @see writeToFileCTS
   */
  virtual void readFromFileCTS(std::string file_name);

  virtual void readFromFileCTS(std::ifstream& file_to_read);

};

}

#endif /* CLUSTERTRAJECTORYSET_H_ */
