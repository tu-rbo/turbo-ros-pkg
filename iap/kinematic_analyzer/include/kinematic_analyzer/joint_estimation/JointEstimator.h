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
 * JointEstimator.h
 *
 *  Created on: Apr 19, 2012
 *      Author: roberto
 */

#ifndef JOINTESTIMATOR_H_
#define JOINTESTIMATOR_H_

#include "BodyTrajectory.h"
#include "BodyTrajectoryPair.h"
#include "JointEstimatorResult.h"
#include "ClusterTrajectory.h"
#include "joint/JointFactory.h"

class JointEstimator;
typedef boost::shared_ptr<JointEstimator> JointEstimatorPtr;

typedef enum JointEstimatorType
{
  TRANSFORMATION_BASED = 0, FEATURE_BASED =1, RECURSIVE = 2, PROBABILISTIC = 3
} JointEstimatorType;

class JointEstimator
{
public:
  JointEstimator(double min_motion);
  JointEstimator(const JointEstimator& je);
  virtual ~JointEstimator();

  virtual JointEstimatorResultPtr execute(vision::ClusterTrajectoryPtr ct1, vision::ClusterTrajectoryPtr ct2) = 0;

  /**
   * Get the last result
   */
  virtual JointEstimatorResultPtr getJointEstimatorResult() const;

  virtual void reset() = 0;

  /**
   * Get the type of JointEstimator
   * @return - Type of joint estimator
   */
  virtual JointEstimatorType getType() const = 0;

  /**
   * Get the type of JointEstimator as an std::string
   * @return - String with the name of the JointEstimator type
   */
  virtual std::string getTypeStr() const = 0;

  JointEstimatorPtr clone() const;

  /**
   * Set the minimum motion of the features, used to trigger the segmentation. It could be relevant to characterize the
   * uncertainty of the measurement
   * @param - min_motion - Minimum motion that triggers the segmentation
   */
  virtual void setMinimumMotion(double min_motion);

protected:
  /**
   * last joint estimator result
   */
  JointEstimatorResultPtr _jer;

  // TODO ???
  std::vector<JointPtr> _joints;

  double _min_motion;

  virtual JointEstimator* doClone() const = 0;
};

#endif /* JOINTESTIMATOR_H_ */
