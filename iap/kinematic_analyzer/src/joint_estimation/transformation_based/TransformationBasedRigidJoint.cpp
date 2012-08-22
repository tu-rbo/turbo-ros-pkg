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
 * TransformationBasedRigidJoint.cpp
 *
 *  Created on: Apr 23, 2012
 *      Author: roberto
 */

#include "transformation_based/TransformationBasedRigidJoint.h"

using namespace vision;

TransformationBasedRigidJoint::TransformationBasedRigidJoint() :
  Joint(), TransformationBasedJoint(), RigidJoint()
{
}

TransformationBasedRigidJoint::TransformationBasedRigidJoint(const TransformationBasedRigidJoint& tbpj) :
  Joint(tbpj), TransformationBasedJoint(tbpj), RigidJoint(tbpj)
{

}

TransformationBasedRigidJoint::~TransformationBasedRigidJoint()
{
}

void TransformationBasedRigidJoint::estimateAxis()
{
  // Motion axis supposing rigid motion (point of application = center of the body, orientation = (0,0,0)):
  FeaturePtr center_body2 = this->_ct2->getFeatureSet(this->_ct2->getTrajectoryLength() - 1)->findCenter();
  FeaturePtr center_body1 = this->_ct1->getFeatureSet(this->_ct1->getTrajectoryLength() - 1)->findCenter();
  FeaturePtr middle_offset(
                           new Feature((center_body1->getX() - center_body2->getX()) / 2.0,
                                       (center_body1->getY() - center_body2->getY()) / 2.0,
                                       (center_body1->getZ() - center_body2->getZ()) / 2.0));
  this->_axis.position = center_body2 + middle_offset;
  this->_axis.orientation = FeaturePtr(new Feature(0., 0., 0.));
}

double TransformationBasedRigidJoint::getGoodnessOfFit()
{
  // TODO
  return -1;
}

iap_common::JointMsg TransformationBasedRigidJoint::toROSMsg()
{
  iap_common::JointMsg msg = RigidJoint::toROSMsg();
  TransformationBasedJoint::toROSMsg(msg);
  return msg;
}
