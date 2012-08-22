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
 * FeatureTrajectory.h
 *
 *  Created on: Nov 28, 2011
 *      Author: shoefer
 */

#ifndef FEATURETRAJECTORY_H_
#define FEATURETRAJECTORY_H_

#include "feature.h"
#include <boost/shared_ptr.hpp>

namespace vision
{

class FeatureTrajectory;
typedef boost::shared_ptr<FeatureTrajectory> FeatureTrajectoryPtr;

/**
 * @brief A trajectory of a feature over time
 *
 * A feature generates a trajectory which is represented
 * by this class. It can be instantiated directly or is used
 * when iterating over a FeatureTrajectorySet
 */
class FeatureTrajectory
{
public:

  /**
   * Constructor
   */
  FeatureTrajectory();

  /**
   * Copy constructor
   * @param ft - FeatureTrajectory to be copied
   */
  FeatureTrajectory(const FeatureTrajectory& ft);

  /**
   * Destructor
   */
  virtual ~FeatureTrajectory();

  /**
   * Clone Function
   * @return Clone (copy) this FeatureTrajectory
   */
  FeatureTrajectoryPtr clone() const;

  /**
   * Add a Feature to the Trajectory
   * @param f - Feature to be added
   * @return - True if the Feature could be successfully added
   */
  virtual bool addFeature(const FeaturePtr & f);

  /**
   * Get a COPY of the Feature at the desired frame
   * @param frame_nr - Number of the Frame we want the Feature from
   * @return - COPY of the Feature at the desired frame, or empty pointer if it was not found
   */
  virtual FeaturePtr getFeature(const unsigned int frame_nr) const;

  /**
   * Get the number of frames in which this trajectory is defined
   * @return - Number of frames of the Trajectory
   */
  virtual unsigned int getTrajectoryLength() const;

  /**
   * Estimate the amount of motion during the whole trajectory. The amount of motion is estimated between each pair
   * of consecutive frames and then added. It is NOT the position difference between last and first frame!
   * @return - Amount of motion
   */
  virtual double estimateAmountOfMotion() const;

  /**
   * Finds the frame number where the Feature has moved maximally wrt. the given frame
   * @param comparing_frame - Frame to use as reference to find the most distanced one
   * @return - Frame number where the Feature distance is max wrt the given one
   */
  virtual int findFrameOfMaxDistance(int comparing_frame) const;

  /**
   * CLONE all the Features in the Trajectory and return a vector with them
   * @return - Vector with the trajectory
   */
  virtual std::vector<FeaturePtr>& featureTrajectory2vector() const;

protected:
  std::vector<FeaturePtr> _trajectory;

  virtual FeatureTrajectory* doClone() const
  {
    return (new FeatureTrajectory(*this));
  }

public:
  // Iterators ///////////////////////////////////////////////////
  typedef std::vector<FeaturePtr>::iterator iterator;
  typedef std::vector<FeaturePtr>::const_iterator const_iterator;

  iterator begin()
  {
    return _trajectory.begin();
  }

  iterator end()
  {
    return _trajectory.end();
  }

  const_iterator begin() const
  {
    return _trajectory.begin();
  }

  const_iterator end() const
  {
    return _trajectory.end();
  }

  // Printing Functions ////////////////////////////////////////////
  virtual void printFT() const;
  virtual void printF(const int frame_number) const;

  // Writing Functions /////////////////////////////////////////////
  virtual void writeToFileFT(std::string file_name) const;
  virtual void appendToFileFT(std::string file_name) const;
};

}

#endif /* FEATURETRAJECTORY_H_ */
