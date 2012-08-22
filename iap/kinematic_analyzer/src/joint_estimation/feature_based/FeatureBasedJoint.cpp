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
#include "feature_based/FeatureBasedJoint.h"

FeatureBasedJoint::FeatureBasedJoint()
{
}

FeatureBasedJoint::~FeatureBasedJoint()
{
}

void FeatureBasedJoint::setClusterTrajectories(vision::ClusterTrajectoryPtr ct1,
                               vision::ClusterTrajectoryPtr ct2) {
  _ct1 = ct1;
  _ct2 = ct2;
}

void FeatureBasedJoint::setBodyTrajectoryPair(BodyTrajectoryPairPtr btp)
{
  _btp = btp;
}

FeatureBasedJointPtr FeatureBasedJoint::clone() const
{
  return FeatureBasedJointPtr(doClone());
}

::iap_common::JointMsg FeatureBasedJoint::toROSMsg(::iap_common::JointMsg& msg)
{
  msg.bodyTrajectoryPair = _btp->toROSMsg();
  msg.joint_estimator = msg.TRANSFORMATION_BASED;
  return msg;
}
