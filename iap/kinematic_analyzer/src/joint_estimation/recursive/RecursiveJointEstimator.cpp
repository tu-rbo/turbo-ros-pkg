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
 * RecursiveJointEstimator.cpp
 *
 *  Created on: Apr 19, 2012
 *      Author: roberto
 */

#include "recursive/RecursiveJointEstimator.h"
#include "recursive/RecursiveJointFactory.h"
#include "recursive/RecursiveJoint.h"

RecursiveJointEstimator::RecursiveJointEstimator(double min_motion):
JointEstimator(min_motion)
{
  this->_joints = RecursiveJointFactory::getInstance()->generateJoints(min_motion);
}

RecursiveJointEstimator::RecursiveJointEstimator(const RecursiveJointEstimator& tbje) :
  JointEstimator(tbje)
{
  this->_internal_belief = tbje._internal_belief;
  this->_joint_estimator = tbje._joint_estimator;
  this->_ct1 = tbje._ct1;
  this->_ct2 = tbje._ct2;
  this->_btp = tbje._btp;
}

RecursiveJointEstimator::~RecursiveJointEstimator()
{
}

JointEstimatorResultPtr RecursiveJointEstimator::execute(vision::ClusterTrajectoryPtr ct1,
                                                         vision::ClusterTrajectoryPtr ct2)
{
  this->_ct1 = ct1;
  this->_ct2 = ct2;

  // Estimate for both ClusterTrajectories the set of RigidTransformations
  BodyTrajectoryPtr body_trajectory1 = ct1->estimateBodyTrajectory();
  BodyTrajectoryPtr body_trajectory2 = ct2->estimateBodyTrajectory();
  this->_btp = BodyTrajectoryPairPtr(new BodyTrajectoryPair(body_trajectory1, body_trajectory2));

  // Obtain the current measurement using the internal JointEstimator object
  JointEstimatorResultPtr measurement_current = this->_joint_estimator->execute(this->_ct1, this->_ct2);

  // Merge previous belief and current measurement
  JointEstimatorResultPtr internal_belief = this->_internal_belief[std::pair<int, int>(ct1->getClusterId(),
                                                                                       ct2->getClusterId())];
  if (!internal_belief)
  {
    this->_joints = RecursiveJointFactory::getInstance()->generateJoints(this->_min_motion);
  }
  else
  {
    this->_joints = internal_belief->getJoints();
  }

  this->estimateAxes(measurement_current);

  // create joint result
  this->_jer = JointEstimatorResultPtr(new JointEstimatorResult());
  std::vector<JointPtr>::iterator jh_it = this->_joints.begin();
  std::vector<JointPtr>::iterator jh_it_end = this->_joints.end();
  for (; jh_it != jh_it_end; jh_it++)
  {
    this->_jer->setJointHypothesis((*jh_it)->getType(), (*jh_it));
  }
  this->estimateProbabilities(measurement_current);
  this->_internal_belief[std::pair<int, int>(ct1->getClusterId(), ct2->getClusterId())] = this->_jer->clone();
  return this->_jer;
}

void RecursiveJointEstimator::estimateAxes(JointEstimatorResultPtr measurement)
{
  std::vector<JointPtr>::iterator jh_it = _joints.begin();
  std::vector<JointPtr>::iterator jh_it_end = _joints.end();

  for (; jh_it != jh_it_end; jh_it++)
  {
    RecursiveJointPtr joint = boost::dynamic_pointer_cast<RecursiveJoint>(*jh_it);
    joint->setClusterTrajectories(_ct1, _ct2);
    joint->setBodyTrajectoryPair(_btp);

    // Obtain the measured joint of each type
    JointPtr measured_joint = measurement->getJointHypothesis(joint->getType());

    // Set the latest measurement to be merged into the RE loop
    joint->setMeasurement(measured_joint);
    // Merge internal belief and latest measurement
    joint->estimateAxis();
  }
}

void RecursiveJointEstimator::setEstimator(JointEstimatorPtr je)
{
  this->_joint_estimator = je->clone();
}

void RecursiveJointEstimator::estimateProbabilities(JointEstimatorResultPtr measurement)
{
  // Copy the probabilities from the last measurements (from the non recursive JointEstimator)
  std::vector<JointPtr>::iterator jh_it = this->_joints.begin();
  std::vector<JointPtr>::iterator jh_it_end = this->_joints.end();

  for (; jh_it != jh_it_end; jh_it++)
  {
    this->_jer->setJointProbability((*jh_it)->getType(), measurement->getJointProbability((*jh_it)->getType()));
  }
}

void RecursiveJointEstimator::reset()
{
  this->_internal_belief.clear();
}

RecursiveJointEstimatorPtr RecursiveJointEstimator::clone() const
{
  return RecursiveJointEstimatorPtr(doClone());
}

JointEstimatorResultPtr RecursiveJointEstimator::_recursiveFiltering(JointEstimatorResultPtr jer_previous,
                                                                     JointEstimatorResultPtr jer_current)
{
  // If there is no previous estimation
  if (!jer_previous)
  {

  }
  else
  {

  }
}

JointEstimatorType RecursiveJointEstimator::getType() const
{
  return RECURSIVE;
}

std::string RecursiveJointEstimator::getTypeStr() const
{
  return std::string("RECURSIVE");
}
