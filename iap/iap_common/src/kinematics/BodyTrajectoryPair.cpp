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
 * BodyTrajectoryPair.cpp
 *
 *  Created on: Apr 16, 2012
 *      Author: roberto
 */

#include "BodyTrajectoryPair.h"
#include "RigidTransformation.h"

BodyTrajectoryPair::BodyTrajectoryPair()
{
  this->_bt1 = BodyTrajectoryPtr();
  this->_bt2 = BodyTrajectoryPtr();
  this->_lt = BodyTrajectoryPtr();
}

BodyTrajectoryPair::BodyTrajectoryPair(const BodyTrajectoryPair& btp)
{
  this->_bt1 = btp.getBody1Trajectory();
  this->_bt1 = btp.getBody2Trajectory();
}

BodyTrajectoryPair::BodyTrajectoryPair(BodyTrajectoryPtr bt1, BodyTrajectoryPtr bt2)
{
  if (bt1->getTimeStart() != bt2->getTimeStart() || bt1->getTimeEnd() != bt2->getTimeEnd()
      || bt1->getTrajectoryLength() != bt2->getTrajectoryLength())
  {
    ROS_ERROR_NAMED("BodyTrajectoryPair.BodyTrajectoryPair","Different length of reference and non-reference trajectories!");
  }
  this->_bt1 = bt1;
  this->_bt2 = bt2;
}

BodyTrajectoryPair::~BodyTrajectoryPair()
{
}

BodyTrajectoryPtr BodyTrajectoryPair::getBody1Trajectory() const
{
  return this->_bt1;
}

BodyTrajectoryPtr BodyTrajectoryPair::getBody2Trajectory() const
{
  return this->_bt2;
}

int BodyTrajectoryPair::getTimeStart() const
{
  return this->_bt1->getTimeStart();
}

int BodyTrajectoryPair::getTimeEnd() const
{
  return this->_bt2->getTimeEnd();
}

int BodyTrajectoryPair::getTrajectoryLength() const
{
  return this->_bt1->getTrajectoryLength();
}

int BodyTrajectoryPair::getBodyId1() const
{
  return this->_bt1->getBodyId();
}

int BodyTrajectoryPair::getBodyId2() const
{
  return this->_bt2->getBodyId();
}

BodyTrajectoryPtr BodyTrajectoryPair::getLocalTrajectory(double scaling_factor)
{
  if (!this->_lt)
  {
    std::vector<RigidTransformationPtr> local_transformations;
    for (int i = this->_bt1->getTimeStart(); i <= this->_bt1->getTimeEnd(); i++)
    {
      Eigen::Transform<double, 3, Eigen::Affine> scaled_motion =
          this->_bt2->getRigidTransformation(i)->getTransformationMatrix();
      // NOTE: Scaling both RBs and estimating the RT between them is equivalent to scale the translation of the original RT
      scaled_motion(0, 3) = scaling_factor * scaled_motion(0, 3);
      scaled_motion(1, 3) = scaling_factor * scaled_motion(1, 3);
      scaled_motion(2, 3) = scaling_factor * scaled_motion(2, 3);
      RigidTransformationPtr scaled_rbt = RigidTransformationPtr(new RigidTransformation(scaled_motion));
      local_transformations.push_back(scaled_rbt - this->_bt1->getRigidTransformation(i));
    }
    this->_lt = BodyTrajectoryPtr(
                                  new BodyTrajectory(this->_bt2->getTimeStart(), local_transformations,
                                                     this->_bt2->getBodyId()));
  }
  return this->_lt;
}

BodyTrajectoryPairPtr BodyTrajectoryPair::clone() const
{
  return BodyTrajectoryPairPtr(doClone());
}


iap_common::BodyTrajectoryPairMsg BodyTrajectoryPair::toROSMsg() const {
  iap_common::BodyTrajectoryPairMsg msg;

  msg.body1_trajectory = _bt1->toROSMsg();
  msg.body2_trajectory = _bt2->toROSMsg();
  msg.local_trajectory = _lt->toROSMsg();

  return msg;
}

void  BodyTrajectoryPair::set(iap_common::BodyTrajectoryPairMsg&) {
  // FIXME not implemented
}
