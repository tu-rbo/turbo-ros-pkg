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
 * FeatureBasedRigidJoint.cpp
 *
 *  Created on: Apr 23, 2012
 *      Author: roberto
 */

#include "feature_based/FeatureBasedRigidJoint.h"

#include "MathFunctions.h"

using namespace vision;

FeatureBasedRigidJoint::FeatureBasedRigidJoint() :
  Joint(), FeatureBasedJoint(), RigidJoint()
{
}

FeatureBasedRigidJoint::FeatureBasedRigidJoint(const FeatureBasedRigidJoint& tbpj) :
  Joint(tbpj), FeatureBasedJoint(tbpj), RigidJoint(tbpj)
{

}

FeatureBasedRigidJoint::~FeatureBasedRigidJoint()
{
}

void FeatureBasedRigidJoint::estimateAxis()
{
  // Motion axis supposing rigid motion (point of application = center of the body, orientation = (0,0,0)):
  FeaturePtr center_body2 = this->_ct2->getFeatureSet(this->_ct2->getTrajectoryLength() - 1)->findCenter();
  FeaturePtr center_body1 = this->_ct1->getFeatureSet(this->_ct1->getTrajectoryLength() - 1)->findCenter();
  FeaturePtr middle_offset = (center_body1 - center_body2) / 2;
  //FeaturePtr middle_point = center_body2 + middle_offset;
  //  FeaturePtr(
  //                                       new Feature((center_body1->getX() - center_body2->getX()) / 2,
  //                                                   (center_body1->getY() - center_body2->getY()) / 2,
  //                                                   (center_body1->getZ() - center_body2->getZ()) / 2));
  //  middle_point->setPos(center_body2->getX() + middle_point->getX(), center_body2->getY() + middle_point->getY(),
  //                       center_body2->getZ() + middle_point->getZ());
  this->_axis.position = center_body2 + middle_offset;
  this->_axis.orientation = FeaturePtr(new Feature(0., 0., 0.));
}

double FeatureBasedRigidJoint::getGoodnessOfFit()
{
  /* Calculate if the bodies have enough motion */
  ClusterTrajectoryPtr body_local_ct1 = ClusterTrajectoryPtr(new ClusterTrajectory());
  ClusterTrajectoryPtr body_local_ct2 = ClusterTrajectoryPtr(new ClusterTrajectory());

  unsigned int Nframes = this->_ct2->getTrajectoryLength();

  for (int i = 0; i < Nframes; i++)
  {
    body_local_ct1->addFeatureSet(this->_ct1->getCluster(i));
    body_local_ct2->addFeatureSet(this->_ct2->getCluster(i));
  }
  std::vector<std::vector<vision::FeaturePtr> > bodyOne = body_local_ct1->FeatureTrajectorySet2Vector();
  std::vector<std::vector<vision::FeaturePtr> > bodyTwo = body_local_ct2->FeatureTrajectorySet2Vector();
  double overallMotionOne = math::calculateOverallMotion(bodyOne);
  double overallMotionTwo = math::calculateOverallMotion(bodyTwo);

  if (overallMotionOne + overallMotionTwo < math::minimalMotionToCompute)
  {
    return 1.;
  }
  else
  {
    return 0.;
  }
}

::iap_common::JointMsg FeatureBasedRigidJoint::toROSMsg()
{
  ::iap_common::JointMsg msg = RigidJoint::toROSMsg();
  FeatureBasedJoint::toROSMsg(msg);
  return msg;
}
