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
 * JointEstimatorResult.cpp
 *
 *  Created on: Apr 19, 2012
 *      Author: roberto
 */

#include "JointEstimatorResult.h"

JointEstimatorResult::JointEstimatorResult(int link1, int link2)
: _link1(link1), _link2(link2)
{
}

JointEstimatorResult::JointEstimatorResult(const JointEstimatorResult& jer)
{
  this->_joint_hypothesis = jer._joint_hypothesis;
  this->_link1 = jer._link1;
  this->_link2 = jer._link2;
}

JointEstimatorResult::~JointEstimatorResult()
{
  // TODO Auto-generated destructor stub
}

void JointEstimatorResult::setJointHypothesis(JointType type, JointPtr joint)
{
  assert (type == joint->getType());
  this->_joint_hypothesis[joint->getType()] = joint;
}

JointPtr JointEstimatorResult::getJointHypothesis(JointType type) const
{
  return this->_joint_hypothesis.at(type);
}

void JointEstimatorResult::setJointProbability(JointType type, double p)
{
  this->_joint_probability[type] = p;
}

double JointEstimatorResult::getJointProbability(JointType type) const
{
  return this->_joint_probability.at(type);
}

JointPtr JointEstimatorResult::getMostProbableJoint() const
{
  double max_probability = -1.;
  JointPtr max_probable_joint;
  std::map<JointType, double>::const_iterator jp_it = this->_joint_probability.begin();
  std::map<JointType, double>::const_iterator jp_it_end = this->_joint_probability.end();
  for (; jp_it != jp_it_end; jp_it++)
  {
    if (jp_it->second > max_probability)
    {
      max_probability = jp_it->second;
      max_probable_joint = this->_joint_hypothesis.at(jp_it->first)->clone();
    }
  }
  if (max_probable_joint)
  {
    ROS_INFO_STREAM("[JointEstimatorResult::getMostProbableJoint] The most probable Joint is "
        << max_probable_joint->getTypeStr() << " with probability "<<max_probability);
  }else
  {
    ROS_ERROR_STREAM("[JointEstimatorResult::getMostProbableJoint] There is no more probable joint!");
  }
  return max_probable_joint;
}

std::vector<JointPtr> JointEstimatorResult::getJoints() const
{
  std::vector<JointPtr> ret_val;
  std::map<JointType, JointPtr>::const_iterator jp_it = this->_joint_hypothesis.begin();
  std::map<JointType, JointPtr>::const_iterator jp_it_end = this->_joint_hypothesis.end();
  for (; jp_it != jp_it_end; jp_it++)
  {
    ret_val.push_back(jp_it->second->clone());
  }
  return ret_val;
}

JointEstimatorResultPtr JointEstimatorResult::clone() const
{
  return JointEstimatorResultPtr(doClone());
}

iap_common::JointEstimatorResultMsg JointEstimatorResult::toROSMsg() {
  iap_common::JointEstimatorResultMsg msg;

  msg.link1 = _link1;
  msg.link2 = _link2;

  std::map<JointType, JointPtr>::const_iterator it;
  it = _joint_hypothesis.begin();

  JointType bestType = getMostProbableJoint()->getType();

  for (; it != _joint_hypothesis.end(); it++) {
    JointType type = it->first;
    JointPtr joint = it->second;

    msg.joint_hypothesis.push_back(joint->toROSMsg());
    msg.joint_probability.push_back(_joint_probability[type]);

    if (bestType == type) {
      msg.best = msg.joint_hypothesis.back().type;
    }
  }

  return msg;
}

void  JointEstimatorResult::set(iap_common::JointEstimatorResultMsg&) {
  // FIXME not implemented
}

/*void JointEstimatorResult::estimate(vision::ClusterTrajectoryPtr ct1, vision::ClusterTrajectoryPtr ct2, BodyTrajectoryPairPtr btp, BodyTrajectoryPtr local_traj)
 {
 std::map<JointType, JointHypothesisPtr>::iterator jh_it = this->_joint_hypothesis.begin();
 std::map<JointType, JointHypothesisPtr>::iterator jh_it_end = this->_joint_hypothesis.end();
 for(; jh_it != jh_it_end; jh_it++)
 {
 jh_it->second->estimateProbability(ct1, ct2, btp, local_traj);
 jh_it->second->estimateAxis(ct1, ct2, btp, local_traj);
 }
 }*/
