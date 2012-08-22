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
 * TransformationBasedUndefinedJoint.cpp
 *
 *  Created on: Apr 23, 2012
 *      Author: roberto
 */

#include "transformation_based/TransformationBasedUndefinedJoint.h"

using namespace vision;

TransformationBasedUndefinedJoint::TransformationBasedUndefinedJoint()
: Joint(), TransformationBasedJoint(), UndefinedJoint()
{
}

TransformationBasedUndefinedJoint::TransformationBasedUndefinedJoint(const TransformationBasedUndefinedJoint& tbpj)
: Joint(tbpj), TransformationBasedJoint(tbpj), UndefinedJoint(tbpj)
{

}

TransformationBasedUndefinedJoint::~TransformationBasedUndefinedJoint()
{
}

void TransformationBasedUndefinedJoint::estimateAxis()
{
  // Motion axis supposing rigid motion (point of application = center of the body, orientation = (0,0,0)):
  FeaturePtr center_body2 = this->_ct2->getFeatureSet(this->_ct2->getTrajectoryLength()-1)->findCenter();
  FeaturePtr center_body1 = this->_ct1->getFeatureSet(this->_ct1->getTrajectoryLength()-1)->findCenter();
  FeaturePtr middle_offset = (center_body1-center_body2)/2;
  this->_axis.position = center_body2 + middle_offset;
  this->_axis.orientation = FeaturePtr(new Feature(0., 0., 0.));
}


double TransformationBasedUndefinedJoint::getGoodnessOfFit()
{
  // TODO
  return -1;
}

iap_common::JointMsg TransformationBasedUndefinedJoint::toROSMsg()
{
  iap_common::JointMsg  msg = UndefinedJoint::toROSMsg();
  TransformationBasedJoint::toROSMsg(msg);
  return msg;
}
