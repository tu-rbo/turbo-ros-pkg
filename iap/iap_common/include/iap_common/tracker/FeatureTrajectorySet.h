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
 * FeatureTrajectorySet.h
 *
 *  Created on: Nov 28, 2011
 *      Author: shoefer
 */

#ifndef FEATURETRAJECTORYSET_H_
#define FEATURETRAJECTORYSET_H_

#include "FeatureSet.h"
#include "FeatureTrajectory.h"

namespace vision
{

class FeatureTrajectorySet;
typedef boost::shared_ptr<FeatureTrajectorySet> FeatureTrajectorySetPtr;

/**
 * @brief A set of features tracked over time
 *
 * A set of features with each own having a distinct
 * trajectory over time. This class allows to iterate
 * over the time axis (FeatureTrajectory) or over the
 * features (FeatureSets)
 */
class FeatureTrajectorySet
{
public:

  /**
   * Constructor
   */
  FeatureTrajectorySet();

  /**
   * Copy constructor
   * @param fts FeatureTrajectorySet to be copied
   */
  FeatureTrajectorySet(const FeatureTrajectorySet & fts);

  /**
   * Destructor
   */
  virtual ~FeatureTrajectorySet();

  /**
   * Add a FeatureSet to the Trajectory of FeatureSets. The new FeatureSet MUST have the same number of Features than the previously stored FS's
   * @param fs - FeatureSet to be added
   * @return - True if the adding operation was successful
   */
  virtual bool addFeatureSet(const FeatureSetPtr &fs);

  // By no means define this method:
  //virtual void add(FeatureTrajectoryPtr);
  // It will be a problem as it is not allowed in the subclass ClusterTrajectory

  /**
   * @brief Returns a pointer to a fresh copy of a FeatureSet
   * @param frame_nr - Number of the frame we want to get the FeatureSet from
   * @return COPY of the FeatureSet of in the desired frame
   */
  virtual FeatureSetPtr getFeatureSet(const int unsigned frame_nr) const;

  /**
   * @brief Returns a pointer to a fresh copy of a Feature Trajectory
   * @param feature_id - Id of the Feature we want to get the Trajectory from
   * @return COPY of the FeatureTrajectory of the desired Feature
   */
  virtual FeatureTrajectoryPtr getFeatureTrajectory(const int feature_id) const;

  /**
   * Returns a pointer to a fresh copy of the Feature
   * @param frame_nr - Number of the frame we want to get the Feature from
   * @param feature_id - Id of the Feature we want to get the Feature from
   * @return - COPY of the Feature
   */
  virtual FeaturePtr getFeature(const int unsigned frame_nr, const int feature_id) const;

  /**
   * @brief Returns the number for tracked features
   *
   * Warning: This method assumes consistency. If the internal state of the trajectory is changed
   *  be sure to run isConsistent
   */
  virtual int getNumberOfFeatures() const;

  /**
   * @brief Returns the trajectory length
   *
   * Warning: This method assumes consistency. If the internal state of the trajectory is changed
   *  be sure to run isConsistent
   */
  virtual unsigned int getTrajectoryLength() const;

  /**
   * @brief Checks for consistency
   *
   * Verifies that there is the some amount of features in every time step
   */
  virtual bool isConsistent() const;

  /**
   * @brief Estimate roughly the amount of motion of this set of Features along the trajectory
   *
   * It uses the center of all the Feature Set to estimate the motion
   * @param first_frame - First frame to estimate the amount of motion
   * @param last_frame - Last frame to estimate the amount of motion
   * @return Value of the amount of motion of the Feature Set center along the trajectory
   */
  virtual double estimateAmountOfMotion(int first_frame = 0, int last_frame = 0) const;

  /**
   * @brief Estimate the amount of motion of each (not lost) feature
   *
   * @param first_frame - First frame to estimate the amount of motion
   * @param last_frame - Last frame to estimate the amount of motion
   * @return Vector with the amount of motion of each (not lost) feature
   */
  virtual std::vector<double> estimateFeaturesMotion(int first_frame = 0, int last_frame = 0) const;

  /**
   * Estimates the amount of motion of each feature along the whole trajectory and finds the most moving one. The amount of motion
   * is estimated adding the distance between each pair of consecutive frames, NOT directly as the distance between the first and the last frames
   * @return - Id of the most moving Feature
   */
  virtual int findMostMovingFeature() const;

  /**
   * Estimate the motion of the FeatureSet between the two frames of interest
   * @param first_frame - Reference frame defining the motion
   * @param last_frame - Final frame defining the motion
   * @return - Translation and Rotation of the motion
   */
  virtual FeatureSet::MotionDefinition estimateMotion(int first_frame = 0, int last_frame = 0) const;

  /**
   * Clear the FeatureTrajectorySet deleting all contained elements
   */
  virtual void clear();

  /**
   * Clone function
   * @return - Clone of this FeatureTrajectorySet
   */
  FeatureTrajectorySetPtr clone() const;

  /**
   * Check if the Feature of interest gets lost along the trajectory. It assumes that once it gets lost it can't be recovered, so it
   * checks directly in the last frame
   * @param feature_id - Feature of interest
   * @return - True if the Feature gets lost along the trajectory
   */
  virtual bool isLost(const int feature_id) const;

  /**
   * Check if the Feature of interest is contained in this Set
   * @param feature_id - Id of the Feature of interest
   * @return - True if this Set contains a Feature with this Id
   */
  virtual bool isContained(const int feature_id) const;

  /**
   * Creates a vector< vector < FeaturePtr > > from this FeatureTrajectorySet
   * @return - vector of vector of Features with a COPY of the contents of this FeatureTrajectorySet
   */
  virtual std::vector<std::vector< FeaturePtr > > FeatureTrajectorySet2Vector() const;

  /**
   * Publish the content of this FeatureTrajectorySet in ROS
   * @param publisher - Pointer to the publisher object that publishes the content
   * @param msec_delay - Msec of delay to wait to publish consecutive FeatureSet's
   */
  virtual void PublishInROS(ros::Publisher* publisher, int msec_delay=0) const;

  friend class ClusterTrajectorySet;

protected:
  std::vector<FeatureSetPtr> _feature_sets;

  virtual FeatureTrajectorySet* doClone() const
  {
    return (new FeatureTrajectorySet(*this));
  }

public:
  // Iterators ////////////////////////////////////////////////////////////////////////
  //    typedef set_trajectory_iterator ...;
  //    typedef trajectory_set_iterator ...;
  typedef std::vector<FeatureSetPtr>::iterator trajectory_set_iterator;
  typedef std::vector<FeatureSetPtr>::const_iterator const_trajectory_set_iterator;
  typedef std::vector<FeatureSetPtr>::reverse_iterator trajectory_set_rev_iterator;
  typedef std::vector<FeatureSetPtr>::const_reverse_iterator const_trajectory_set_rev_iterator;

  trajectory_set_iterator begin()
  {
    return _feature_sets.begin();
  }

  trajectory_set_iterator end()
  {
    return _feature_sets.end();
  }

  const_trajectory_set_iterator begin() const
  {
    return _feature_sets.begin();
  }

  const_trajectory_set_iterator end() const
  {
    return _feature_sets.end();
  }

  trajectory_set_rev_iterator rbegin()
  {
    return _feature_sets.rbegin();
  }

  trajectory_set_rev_iterator rend()
  {
    return _feature_sets.rend();
  }

  const_trajectory_set_rev_iterator rbegin() const
  {
    return _feature_sets.rbegin();
  }

  const_trajectory_set_rev_iterator rend() const
  {
    return _feature_sets.rend();
  }

  // Printing Functions //////////////////////////////////////////////////////////////////
  virtual void printFTS() const;
  virtual void printFT(int feature_id) const;
  virtual void printFS(int frame_nr) const;
  virtual void printF(int feature_id, int frame_nr) const;

  // Write and Read Functions ////////////////////////////////////////////////////////////
  virtual void writeToFileFTS(std::string file_name) const;
  virtual void appendToFileFTS(std::string file_name) const;
  virtual void readFromFileFTS(std::string file_name);
  virtual void readFromFileFTS(std::ifstream &file_to_read);
};

}

#endif /* FEATURETRAJECTORYSET_H_ */
