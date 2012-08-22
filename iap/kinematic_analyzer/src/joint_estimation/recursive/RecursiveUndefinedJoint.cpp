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
 * RecursiveUndefinedJoint.cpp
 *
 *  Created on: Apr 23, 2012
 *      Author: roberto
 */

#include "recursive/RecursiveUndefinedJoint.h"

using namespace vision;

RecursiveUndefinedJoint::RecursiveUndefinedJoint()
: Joint(), RecursiveJoint(), UndefinedJoint()
{
}

RecursiveUndefinedJoint::RecursiveUndefinedJoint(const RecursiveUndefinedJoint& tbpj)
: Joint(tbpj), RecursiveJoint(tbpj), UndefinedJoint(tbpj)
{

}

RecursiveUndefinedJoint::~RecursiveUndefinedJoint()
{
}

void RecursiveUndefinedJoint::estimateAxis()
{
  // Motion axis supposing rigid motion (point of application = center of the body, orientation = (0,0,0)):
  FeaturePtr center_body2 = this->_ct2->getFeatureSet(this->_ct2->getTrajectoryLength()-1)->findCenter();
  FeaturePtr center_body1 = this->_ct1->getFeatureSet(this->_ct1->getTrajectoryLength()-1)->findCenter();
  FeaturePtr middle_offset = (center_body1-center_body2)/2;
  this->_axis.position = center_body2 + middle_offset;
  this->_axis.orientation = FeaturePtr(new Feature(0., 0., 0.));
}


double RecursiveUndefinedJoint::getGoodnessOfFit()
{
  // TODO
  return -1;
}

void RecursiveUndefinedJoint::_initialize()
{

}

void RecursiveUndefinedJoint::_update()
{
  // z is the measurement in this iteration
  FM::Vec z(NX);
  // NOTE: I was resizing the variance here (code at the end of this file)
  z[0] = this->_measurement->getAxis().position->getX();
  z[1] = this->_measurement->getAxis().position->getY();
  z[2] = this->_measurement->getAxis().position->getZ();
  z[3] = this->_measurement->getAxis().orientation->getX();
  z[4] = this->_measurement->getAxis().orientation->getY();
  z[5] = this->_measurement->getAxis().orientation->getZ();

  ROS_INFO_STREAM("[RecursiveUndefinedJoint::_update] Measurement (z) for updating the EKF: "
      << z[0] << " " << z[1] << " "<< z[2] << " "<< z[3] << " "<< z[4] << " "<< z[5]);

  this->_axis.position->setX(z[0]);
  this->_axis.position->setY(z[1]);
  this->_axis.position->setZ(z[2]);
  this->_axis.orientation->setX(z[3]);
  this->_axis.orientation->setY(z[4]);
  this->_axis.orientation->setZ(z[5]);
}

