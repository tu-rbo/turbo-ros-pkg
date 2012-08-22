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
#ifndef CLUSTERTRAJECTORY_H_
#define CLUSTERTRAJECTORY_H_

#include "FeatureTrajectorySet.h"
#include "Cluster.h"
#include "BodyTrajectory.h"
#include <boost/shared_ptr.hpp>

namespace vision
{

class ClusterTrajectory;
typedef boost::shared_ptr<ClusterTrajectory> ClusterTrajectoryPtr;

/*
 * ClusterTrajectory
 *
 * A ClusterTrajectory is a set of FeatureTrajectories all belonging to the group (cluster)
 * at a fixed time frame
 *
 * It is basically a FeatureTrajectorySet + a cluster ID.
 *
 *  Created on: Dec 20, 2011
 *      Author: rmartin + shoefer
 */
class ClusterTrajectory : public FeatureTrajectorySet
{
public:

  /**
   * Constructor
   */
  ClusterTrajectory();

  /**
   * Copy constructor
   * @param ct - ClusterTrajectory to be copied
   */
  ClusterTrajectory(const ClusterTrajectory & ct);

  /**
   * Destructor
   */
  virtual ~ClusterTrajectory();

  /**
   * Clone the whole object and return a copy
   * @return - Clone of the ClusterTrajectory
   */
  ClusterTrajectoryPtr clone() const;

  /**
   * Return the Cluster Id of this Cluster
   * @return - Cluster Id
   */
  virtual int getClusterId() const;

  /**
   * Set the Cluster Id of this Cluster
   * @param cluster_id - New Cluster Id
   */
  virtual void setClusterId(const int cluster_id);


  virtual BodyTrajectoryPtr estimateBodyTrajectory(int start_frame=0, int end_frame=0) const;


  /**
   * Estimate the amount of motion of this Cluster along the trajectory as the mean value of
   * the amount of motion of each Feature of the Cluster along the trajectory (is the motion of the feature between all consecutive
   * frames of the trajectory, NOT the motion between first and last frame!)
   * @return - Mean amount of motion of all Features of the Cluster
   */
  virtual double estimateAmountOfMotion() const;

  /**
   * Clone and return the Cluster at the selected frame
   * @param frame_nr - Frame at which we clone the Cluster
   * @return - Clone of the Cluster at the selected frame
   */
  virtual ClusterPtr getCluster(const unsigned int frame_nr) const;

  /**
   * Add a Cluster to the Trajectory
   * @param cluster - Cluster to be added. It must have the same Id than the previously stored ones
   * @return - True if the adding operation was successful
   */
  virtual bool addCluster(const ClusterPtr& cluster);

protected:
  int _cluster_id; // Identifier of the Cluster

  virtual ClusterTrajectory* doClone() const
  {
    return (new ClusterTrajectory(*this));
  }
public:
  /**
   * Print the contains of the Cluster Trajectory
   */
  virtual void printCT() const;

  /**
   * Write the cluster trajectory into the file
   *
   * The first line contains the cluster id, followed
   * by the FeatureSet trajectory
   *
   * @see appendToFileFTS
   */
  virtual void writeToFileCT(std::string file_name) const;
  virtual void appendToFileCT(std::string file_name) const;

  /**
   * Read a cluster trajectory from a file
   *
   * @see writeToFileCT
   */
  virtual void readFromFileCT(std::string file_name);
  virtual void readFromFileCT(std::ifstream& file_to_read);


};

}
#endif /* CLUSTERTRAJECTORY_H_ */
