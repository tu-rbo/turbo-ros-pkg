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
 * TransformationBasedJointEstimator.h
 *
 *  Created on: Apr 19, 2012
 *      Author: roberto
 */

#ifndef TRANSFORMATIONBASEDJOINTESTIMATOR_H_
#define TRANSFORMATIONBASEDJOINTESTIMATOR_H_

#include "JointEstimator.h"
#include "BodyTrajectoryPair.h"

class TransformationBasedJointEstimator;
typedef boost::shared_ptr<TransformationBasedJointEstimator> TransformationBasedJointEstimatorPtr;

class TransformationBasedJointEstimator : public JointEstimator
{
public:
  TransformationBasedJointEstimator(double min_motion);
  TransformationBasedJointEstimator(const TransformationBasedJointEstimator& tbje);
  virtual ~TransformationBasedJointEstimator();

  virtual JointEstimatorResultPtr execute(vision::ClusterTrajectoryPtr ct1, vision::ClusterTrajectoryPtr ct2);

  virtual void reset();

  TransformationBasedJointEstimatorPtr clone() const;

  /**
   * Get the type of JointEstimator
   * @return - Type of joint estimator
   */
  virtual JointEstimatorType getType() const;

  /**
   * Get the type of JointEstimator as an std::string
   * @return - String with the name of the JointEstimator type
   */
  virtual std::string getTypeStr() const;

protected:

  vision::ClusterTrajectoryPtr _ct1;
  vision::ClusterTrajectoryPtr _ct2;

  BodyTrajectoryPairPtr _btp;

  /**
   * Calculation of the axes for each joint type
   */
  virtual void estimateAxes();

  /**
   * Calculation of the probabilities for each joint type
   */
  virtual void estimateProbabilities();


  virtual TransformationBasedJointEstimator* doClone() const
  {
    return (new TransformationBasedJointEstimator(*this));
  }
};

#endif /* TRANSFORMATIONBASEDJOINTESTIMATOR_H_ */
