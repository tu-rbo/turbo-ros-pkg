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
 * TransformationBasedJointEstimator.cpp
 *
 *  Created on: Apr 19, 2012
 *      Author: roberto
 */

#include "transformation_based/TransformationBasedJointEstimator.h"
#include "transformation_based/TransformationBasedJointFactory.h"
#include "transformation_based/TransformationBasedJoint.h"

TransformationBasedJointEstimator::TransformationBasedJointEstimator(double min_motion):
JointEstimator(min_motion)
{
  this->_joints = TransformationBasedJointFactory::getInstance()->generateJoints(min_motion);
}

TransformationBasedJointEstimator::TransformationBasedJointEstimator(const TransformationBasedJointEstimator& tbje) :
  JointEstimator(tbje)
{
}

TransformationBasedJointEstimator::~TransformationBasedJointEstimator()
{
}

JointEstimatorResultPtr TransformationBasedJointEstimator::execute(vision::ClusterTrajectoryPtr ct1,
                                                                   vision::ClusterTrajectoryPtr ct2)
{
  this->_ct1 = ct1;
  this->_ct2 = ct2;

  // Estimate for both ClusterTrajectories the set of RigidTransformations
  BodyTrajectoryPtr body_trajectory1 = ct1->estimateBodyTrajectory();
  BodyTrajectoryPtr body_trajectory2 = ct2->estimateBodyTrajectory();

  this->_btp = BodyTrajectoryPairPtr(new BodyTrajectoryPair(body_trajectory1, body_trajectory2));

  // Estimate local trajectory between the pair of rigid bodies
  BodyTrajectoryPtr local_trajectory = this->_btp->getLocalTrajectory();

  // Generate joints
  this->_joints = TransformationBasedJointFactory::getInstance()->generateJoints(this->_min_motion);

  // run estimation
  this->estimateAxes();

  // create joint result
  this->_jer = JointEstimatorResultPtr(new JointEstimatorResult(ct1->getClusterId(), ct2->getClusterId()));
  std::vector<JointPtr>::iterator jh_it = this->_joints.begin();
  std::vector<JointPtr>::iterator jh_it_end = this->_joints.end();

  for (; jh_it != jh_it_end; jh_it++)
  {
    this->_jer->setJointHypothesis((*jh_it)->getType(), (*jh_it));
  }

  this->estimateProbabilities();

  return this->_jer;
}

void TransformationBasedJointEstimator::estimateAxes()
{
  // Estimate the type of joint based on the previous characterization of the local motion

  // estimate axes
  std::vector<JointPtr>::iterator jh_it = _joints.begin();
  std::vector<JointPtr>::iterator jh_it_end = _joints.end();

  for (; jh_it != jh_it_end; jh_it++)
  {
    TransformationBasedJointPtr joint = boost::dynamic_pointer_cast<TransformationBasedJoint>(*jh_it);
    joint->setClusterTrajectories(_ct1, _ct2);
    joint->setBodyTrajectoryPair(_btp);
    joint->estimateAxis();
  }
}

void TransformationBasedJointEstimator::estimateProbabilities()
{
  // transformation based joint estimator does not care about axes
  // we analyse the BodyTrajectory parameters directly
  std::vector<double> similarities_to_identity = this->_btp->getLocalTrajectory()->getSimilaritiesToI();
  double global_rotation_avg = (this->_btp->getBody1Trajectory()->getAccRotationAngle()
      + this->_btp->getBody2Trajectory()->getAccRotationAngle()) / 2.;

  double global_translation_avg = (this->_btp->getBody1Trajectory()->getAccTranslationDistance()
      + this->_btp->getBody2Trajectory()->getAccTranslationDistance()) / 2.;

  int num_frames = this->_btp->getTrajectoryLength();
  double local_acc_rotation = this->_btp->getLocalTrajectory()->getAccRotationAngle();
  double local_rot_paral = this->_btp->getLocalTrajectory()->getRotationAxisParallelism();
  double local_acc_translation = this->_btp->getLocalTrajectory()->getAccTranslationDistance();
  double local_trans_paral = this->_btp->getLocalTrajectory()->getTranslationAxisParallelism();

  ROS_INFO_STREAM_NAMED(
                        "TransformationBasedJointEstimator.estimateProbabilities",
                        "\nLocal acc rotation: " << local_acc_rotation << "\nLocal rotation parallelism: "
                            << local_rot_paral << "\nLocal acc translation: " << local_acc_translation
                            << "\nLocal translation parallelism: " << local_trans_paral);

  ROS_DEBUG_STREAM_NAMED(
                         "TransformationBasedJointEstimator.estimateProbabilities",
                         "Global Rotation AVG: " << global_rotation_avg << ";  Global Translation AVG: "
                             << global_translation_avg);

  // Set all probabilities to zero
  this->_jer->setJointProbability(PRISMATIC_JOINT, 0.);
  this->_jer->setJointProbability(REVOLUTE_JOINT, 0.);
  this->_jer->setJointProbability(RIGID_JOINT, 0.);
  this->_jer->setJointProbability(UNDEFINED_JOINT, 0.);
  this->_jer->setJointProbability(UNCERTAIN_JOINT, 0.);


  BodyTrajectoryPtr local_traj = this->_btp->getLocalTrajectory();

  // UNCERTAIN
  //if (global_rotation_avg < 0.01 && global_translation_avg < 0.05)
  if (similarities_to_identity.at(0) > 0.85)
  {
    if (global_rotation_avg < 0.01 && global_translation_avg < 0.05)
    {
      ROS_INFO_STREAM_NAMED("TransformationBasedJointEstimator.estimateProbabilities", "Uncertain joint");
      this->_jer->setJointProbability(UNCERTAIN_JOINT, 1.);
    }
    // RIGID
    //else if (local_acc_translation <= 0.3 && local_acc_rotation < 0.2)
    else
    {
      ROS_INFO("[TransformationBasedJointEstimator::estimateProbabilities] Rigid joint");
      this->_jer->setJointProbability(RIGID_JOINT, 1.);
    }
  }
  // PRISMATIC
  //else if (local_trans_paral > 0.6 && local_acc_translation > 0.2 )
  else if (((similarities_to_identity.at(1)) > 0.7))
  {
    ROS_INFO_STREAM_NAMED("TransformationBasedJointEstimator.estimateProbabilities", "Prismatic joint");
    this->_jer->setJointProbability(PRISMATIC_JOINT, 1.);
  }
  // REVOLUTE
  //else if (local_rot_paral > 0.9 && local_acc_rotation > 1.)
//  else if (local_rot_paral > 0.9 && local_acc_rotation/num_frames > 0.05)
  else
  {
    ROS_INFO_STREAM_NAMED("TransformationBasedJointEstimator.estimateProbabilities", "Revolute joint");
    this->_jer->setJointProbability(REVOLUTE_JOINT, 1.);
  }
  // DISCONNECTED
  // FIXME currently not supported
//  else
//  {
//    ROS_INFO_STREAM_NAMED("TransformationBasedJointEstimator.estimateProbabilities", "Undefined joint (disconnected)");
//    this->_jer->setJointProbability(UNDEFINED_JOINT, 1.);
//  }
}

void TransformationBasedJointEstimator::reset()
{

}

TransformationBasedJointEstimatorPtr TransformationBasedJointEstimator::clone() const
{
  return TransformationBasedJointEstimatorPtr(doClone());
}

JointEstimatorType TransformationBasedJointEstimator::getType() const
{
  return TRANSFORMATION_BASED;
}

std::string TransformationBasedJointEstimator::getTypeStr() const
{
  return std::string("TRANSFORMATION_BASED");
}
