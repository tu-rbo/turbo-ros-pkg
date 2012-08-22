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
 * TransformationBasedPrismaticJoint.cpp
 *
 *  Created on: Apr 23, 2012
 *      Author: shoefer
 */

#include "transformation_based/TransformationBasedPrismaticJoint.h"

using namespace vision;

TransformationBasedPrismaticJoint::TransformationBasedPrismaticJoint() :
   Joint(), TransformationBasedJoint(), PrismaticJoint()
{
}

TransformationBasedPrismaticJoint::TransformationBasedPrismaticJoint(const TransformationBasedPrismaticJoint& tbpj) :
  Joint(tbpj), TransformationBasedJoint(tbpj), PrismaticJoint(tbpj)
{

}

TransformationBasedPrismaticJoint::~TransformationBasedPrismaticJoint()
{
}

void TransformationBasedPrismaticJoint::estimateAxis()
{
  // The axis of the prismatic joint is placed at the center of the body 2 in the last frame and oriented
  // according to the local translation between first and most moving frame
  FeaturePtr axis_position = this->_ct1->getFeatureSet(this->_ct2->getTrajectoryLength() - 1)->findCenter();
  BodyTrajectoryPtr local_trajectory = this->_btp->getLocalTrajectory();
  Eigen::Vector3d axis_ori_eigen =
      local_trajectory->getRTwithMaxTranslationDistance()->getTranslation();
  FeaturePtr axis_orientation = FeaturePtr(new Feature(axis_ori_eigen.x(), axis_ori_eigen.y(), axis_ori_eigen.z()));

  this->_axis.position = axis_position->clone();
  this->_axis.orientation = axis_orientation->clone();
}

double TransformationBasedPrismaticJoint::getGoodnessOfFit()
{
  // TODO
  return -1;
}

iap_common::JointMsg TransformationBasedPrismaticJoint::toROSMsg()
{
  iap_common::JointMsg msg = PrismaticJoint::toROSMsg();
  TransformationBasedJoint::toROSMsg(msg);
  return msg;
}

double TransformationBasedPrismaticJoint::getJointOrientationUncertainty() const
{
  double ori_meas_std_x = 1.;
  if (this->_btp->getLocalTrajectory()->getMaxTranslationDistance() < this->_min_motion / 2.)
  {
    ori_meas_std_x = 1.;
  }
  else if (this->_btp->getLocalTrajectory()->getMaxTranslationDistance() > 3.0 * this->_min_motion / 2.0)
  {
    ori_meas_std_x = 0.01;
  }
  else
  {
    ori_meas_std_x = 1.0 - (this->_btp->getLocalTrajectory()->getMaxTranslationDistance() - this->_min_motion / 2.0)
        * 0.9 / this->_min_motion;
  }
  return ori_meas_std_x;
}
