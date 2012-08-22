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
 * FeatureBasedJointEstimator.cpp
 *
 *  Created on: Apr 19, 2012
 *      Author: roberto
 */

#include "feature_based/FeatureBasedJointEstimator.h"
#include "feature_based/FeatureBasedJointFactory.h"
#include "feature_based/FeatureBasedJoint.h"

#include "MathFunctions.h"

FeatureBasedJointEstimator::FeatureBasedJointEstimator(double min_motion) :
  JointEstimator(min_motion)
{
  this->_joints = FeatureBasedJointFactory::getInstance()->generateJoints(min_motion);
}

FeatureBasedJointEstimator::FeatureBasedJointEstimator(const FeatureBasedJointEstimator& tbje) :
  JointEstimator(tbje)
{
}

FeatureBasedJointEstimator::~FeatureBasedJointEstimator()
{
}

JointEstimatorResultPtr FeatureBasedJointEstimator::execute(vision::ClusterTrajectoryPtr ct1,
                                                            vision::ClusterTrajectoryPtr ct2)
{
  this->_ct1 = ct1;
  this->_ct2 = ct2;

  // Estimate for both ClusterTrajectories the set of RigidTransformations
  BodyTrajectoryPtr body_trajectory1 = ct1->estimateBodyTrajectory();
  body_trajectory1->getAccRotationAngle();
  body_trajectory1->getAccTranslationDistance();

  BodyTrajectoryPtr body_trajectory2 = ct2->estimateBodyTrajectory();
  body_trajectory2->getAccRotationAngle();
  body_trajectory2->getAccTranslationDistance();

  this->_btp = BodyTrajectoryPairPtr(new BodyTrajectoryPair(body_trajectory1, body_trajectory2));

  // Estimate local trajectory between the pair of rigid bodies
  BodyTrajectoryPtr local_trajectory = this->_btp->getLocalTrajectory();
  local_trajectory->getAccRotationAngle();
  local_trajectory->getAccTranslationDistance();

  // TODO: can the logging go somewhere else?

  Eigen::Quaterniond q1 =
      _btp->getBody1Trajectory()->getRigidTransformation(_btp->getTimeEnd())->getRotationQuaternion();
  Eigen::Vector3d t1 = _btp->getBody1Trajectory()->getRigidTransformation(_btp->getTimeEnd())->getTranslation();
  std::stringstream info1;
  info1 << std::string("Global motion body 1:\nTranslation: ") << t1.x() << " " << t1.y() << " " << t1.z() << std::endl
      << std::string("Rotation: ") << q1.w() << " " << q1.x() << " " << q1.y() << " " << q1.z();
  // TODO: why not use ROS_INFO_STREAM_NAMED?
  ROS_INFO_STREAM_NAMED("FeatureBasedJointEstimator.execute", info1.str());
  Eigen::Quaterniond
                     q2 =
                         _btp->getBody2Trajectory()->getRigidTransformation(_btp->getBody2Trajectory()->getTimeEnd())->getRotationQuaternion();
  Eigen::Vector3d t2 = _btp->getBody2Trajectory()->getRigidTransformation(_btp->getTimeEnd())->getTranslation();
  std::stringstream info2;
  info2 << std::string("Global motion body 2:\nTranslation: ") << t2.x() << " " << t2.y() << " " << t2.z() << std::endl
      << std::string("Rotation: ") << q2.w() << " " << q2.x() << " " << q2.y() << " " << q2.z();
  ROS_INFO_STREAM_NAMED("FeatureBasedJointEstimator.execute", info2.str());
  Eigen::Quaterniond q3 =
      local_trajectory->getRigidTransformation(local_trajectory->getTimeEnd())->getRotationQuaternion();
  Eigen::Vector3d t3 = local_trajectory->getRigidTransformation(local_trajectory->getTimeEnd())->getTranslation();
  std::stringstream info3;
  info3 << std::string("Local motion:\nTranslation: ") << t3.x() << " " << t3.y() << " " << t3.z() << std::endl
      << std::string("Rotation: ") << q3.w() << " " << q3.x() << " " << q3.y() << " " << q3.z();
  ROS_INFO_STREAM_NAMED("FeatureBasedJointEstimator.execute", info3.str());

  // Generate joints
  this->_joints = FeatureBasedJointFactory::getInstance()->generateJoints(this->_min_motion);

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

void FeatureBasedJointEstimator::estimateAxes()
{
  // Estimate the type of joint based on the previous characterization of the local motion

  // estimate axes
  std::vector<JointPtr>::iterator jh_it = _joints.begin();
  std::vector<JointPtr>::iterator jh_it_end = _joints.end();

  for (; jh_it != jh_it_end; jh_it++)
  {
    FeatureBasedJointPtr joint = boost::dynamic_pointer_cast<FeatureBasedJoint>(*jh_it);
    joint->setClusterTrajectories(_ct1, _ct2);
    joint->setBodyTrajectoryPair(_btp);
    joint->estimateAxis();
  }
}

void FeatureBasedJointEstimator::estimateProbabilities()
{
  // Set all probabilities to zero
  this->_jer->setJointProbability(PRISMATIC_JOINT, 0.);
  this->_jer->setJointProbability(REVOLUTE_JOINT, 0.);
  this->_jer->setJointProbability(RIGID_JOINT, 0.);
  this->_jer->setJointProbability(UNDEFINED_JOINT, 0.);
  this->_jer->setJointProbability(UNCERTAIN_JOINT, 0.);

  std::vector<JointPtr>::iterator jh_it = _joints.begin();
  std::vector<JointPtr>::iterator jh_it_end = _joints.end();

  int global_joint_index = -1;

  double prismatic_probability = 0;
  double revolute_probability = 0;
  double rigid_probability = 0;

  for (; jh_it != jh_it_end; jh_it++)
  {
    //FeatureBasedJointPtr joint = boost::dynamic_pointer_cast<FeatureBasedJoint>(*jh_it);
    switch ((*jh_it)->getType())
    {
      case PRISMATIC_JOINT:
      {
        prismatic_probability = (*jh_it)->getGoodnessOfFit();
        break;
      }
      case REVOLUTE_JOINT:
      {
        revolute_probability = (*jh_it)->getGoodnessOfFit();
        break;
      }
      case RIGID_JOINT:
      {
        rigid_probability = (*jh_it)->getGoodnessOfFit();
        break;
      }
    }
  }

  double undefined_probability = (1.0 - prismatic_probability) * (1.0 - revolute_probability) * (1.0
      - rigid_probability);
  std::vector<double> probabilities;
  probabilities.push_back(prismatic_probability);
  probabilities.push_back(revolute_probability);
  probabilities.push_back(rigid_probability);
  probabilities.push_back(undefined_probability);
  math::max(probabilities, global_joint_index);
  switch (global_joint_index)
  {
    case 0:
      ROS_INFO("[FeatureBasedJointEstimator::estimateProbabilities] Prismatic joint");
      this->_jer->setJointProbability(PRISMATIC_JOINT, 1.);
      break;
    case 1:
      ROS_INFO("[FeatureBasedJointEstimator::estimateProbabilities] Revolute joint");
      this->_jer->setJointProbability(REVOLUTE_JOINT, 1.);
      break;
    case 2:
      ROS_INFO("[FeatureBasedJointEstimator::estimateProbabilities] Rigid joint");
      this->_jer->setJointProbability(RIGID_JOINT, 1.);
      break;
    case 3:
      ROS_INFO("[FeatureBasedJointEstimator::estimateProbabilities] Undefined joint (disconnected)");
      this->_jer->setJointProbability(UNDEFINED_JOINT, 1.);
      break;
    default:
      ROS_ERROR("[FeatureBasedJointEstimator::estimateProbabilities] Any joint type has goodness > 0");
      break;
  }
}

void FeatureBasedJointEstimator::reset()
{

}

FeatureBasedJointEstimatorPtr FeatureBasedJointEstimator::clone() const
{
  return FeatureBasedJointEstimatorPtr(doClone());
}

JointEstimatorType FeatureBasedJointEstimator::getType() const
{
  return FEATURE_BASED;
}

std::string FeatureBasedJointEstimator::getTypeStr() const
{
  return std::string("FEATURE_BASED");
}
