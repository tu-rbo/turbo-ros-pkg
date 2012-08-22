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
 * RecursiveJointEstimator.h
 *
 *  Created on: Apr 19, 2012
 *      Author: roberto
 */

#ifndef RECURSIVEJOINTESTIMATOR_H_
#define RECURSIVEJOINTESTIMATOR_H_

#include "JointEstimator.h"
#include "BodyTrajectoryPair.h"
#include "KinematicAnalyzer.h"

class RecursiveJointEstimator;
typedef boost::shared_ptr<RecursiveJointEstimator> RecursiveJointEstimatorPtr;

class RecursiveJointEstimator : public JointEstimator
{
public:
  RecursiveJointEstimator(double min_motion);
  RecursiveJointEstimator(const RecursiveJointEstimator& tbje);
  virtual ~RecursiveJointEstimator();

  virtual JointEstimatorResultPtr execute(vision::ClusterTrajectoryPtr ct1, vision::ClusterTrajectoryPtr ct2);

  /**
   * Set the Estimator object that will be used internally to analyze at each step
   * @param je - Pointer to the Estimator object
   */
  virtual void setEstimator(JointEstimatorPtr je);

  virtual void reset();

  RecursiveJointEstimatorPtr clone() const;

  /**
   * Get the type of JointEstimator
   * @return - Type of joint estimator
   */
  virtual JointEstimatorType getType() const;

  /**
   * Get the type of JointEstimator as an std::string
   * @return - String with the name of the JointEstimator type
   */
  virtual std::string getTypeStr() const ;

protected:

  /**
   * Recursive filtering: It takes the previous estimation and the current estimation/measurement and
   * merges both in a new estimation
   * @param jer_previous - Previous estimation
   * @param jer_current - Current estimation/measurement
   * @return - Resulting estimation after filtering
   */
  virtual JointEstimatorResultPtr _recursiveFiltering(JointEstimatorResultPtr jer_previous,
                                                      JointEstimatorResultPtr jer_current);

  KinematicStructure _internal_belief;
  JointEstimatorPtr _joint_estimator;

  vision::ClusterTrajectoryPtr _ct1;
  vision::ClusterTrajectoryPtr _ct2;

  BodyTrajectoryPairPtr _btp;

  /**
   * Calculation of the axes for each joint type
   */
  virtual void estimateAxes(JointEstimatorResultPtr measurement);

  /**
   * Calculation of the probabilities for each joint type
   */
  virtual void estimateProbabilities(JointEstimatorResultPtr measurement);

  virtual RecursiveJointEstimator* doClone() const
  {
    return (new RecursiveJointEstimator(*this));
  }
};

#endif /* RECURSIVEJOINTESTIMATOR_H_ */
