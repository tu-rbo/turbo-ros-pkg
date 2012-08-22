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
 * RecursiveRevoluteJoint.cpp
 *
 *  Created on: Apr 23, 2012
 *      Author: shoefer
 */

#include "recursive/RecursiveRevoluteJoint.h"

#include <algorithm>

using namespace vision;

RecursiveRevoluteJoint::RecursiveRevoluteJoint() :
  Joint(), RecursiveJoint(), RevoluteJoint()
{
}

RecursiveRevoluteJoint::RecursiveRevoluteJoint(const RecursiveRevoluteJoint& tbpj) :
  Joint(tbpj), RecursiveJoint(tbpj), RevoluteJoint(tbpj)
{
}

RecursiveRevoluteJoint::~RecursiveRevoluteJoint()
{
}

void RecursiveRevoluteJoint::setBodyTrajectoryPair(BodyTrajectoryPairPtr btp)
{
  RecursiveJoint::setBodyTrajectoryPair(btp);
}

double RecursiveRevoluteJoint::getGoodnessOfFit()
{
  // TODO
  return -1;
}

void RecursiveRevoluteJoint::_initialize()
{
  this->_ekf.X.clear();
  // NOTE: I was resizing the variance here (code at the end of this file)
  this->_ekf.x[0] = this->_measurement->getAxis().position->getX();
  this->_ekf.x[1] = this->_measurement->getAxis().position->getY();
  this->_ekf.x[2] = this->_measurement->getAxis().position->getZ();

  double ori_norm = this->_measurement->getAxis().orientation->norm();
  this->_ekf.x[3] = this->_measurement->getAxis().orientation->getX() / ori_norm;
  this->_ekf.x[4] = this->_measurement->getAxis().orientation->getY() / ori_norm;
  this->_ekf.x[5] = this->_measurement->getAxis().orientation->getZ() / ori_norm;

  for (int i = 0; i < 3; i++) // Position
  {
    this->_ekf.X(i, i) = sqr(this->_measurement->getJointPositionUncertainty()); // It can't be exactly 0 or we could have an exception "S not PD (Positive Definite) in observe"
  }
  for (int i = 3; i < NX; i++)
  {
    this->_ekf.X(i, i) = sqr(this->_measurement->getJointOrientationUncertainty());
  }

  this->_ekf.init();

  this->_axis.position->setX(this->_ekf.x[0]);
  this->_axis.position->setY(this->_ekf.x[1]);
  this->_axis.position->setZ(this->_ekf.x[2]);
  this->_axis.orientation->setX(this->_ekf.x[3]);
  this->_axis.orientation->setY(this->_ekf.x[4]);
  this->_axis.orientation->setZ(this->_ekf.x[5]);
}

void RecursiveRevoluteJoint::_update()
{
  // NOTE: I was resizing the variance here (code at the end of this file)
  this->_last_z[0] = this->_measurement->getAxis().position->getX();
  this->_last_z[1] = this->_measurement->getAxis().position->getY();
  this->_last_z[2] = this->_measurement->getAxis().position->getZ();
  double ori_norm = this->_measurement->getAxis().orientation->norm();
  this->_last_z[3] = this->_measurement->getAxis().orientation->getX() / ori_norm;
  this->_last_z[4] = this->_measurement->getAxis().orientation->getY() / ori_norm;
  this->_last_z[5] = this->_measurement->getAxis().orientation->getZ() / ori_norm;

  // Change the direction of the orientation vector if it is opposite to the stored one
  // This manages the problem of open-closing a prismatic joint or rotating in one direction and then in the other
  // which provides a similar vector but with oposite directions
  if ((this->_last_z[3] * this->_ekf.x[3] + this->_last_z[4] * this->_ekf.x[4] + this->_last_z[5] * this->_ekf.x[5])
      < -0.5)
  {
    this->_last_z[3] = -this->_last_z[3];
    this->_last_z[4] = -this->_last_z[4];
    this->_last_z[5] = -this->_last_z[5];
  }

  //double observation_noise = std::min(1/(10*ori_norm), 0.1);    // The noise is maximal 0.1
  for (int i = 0; i < 3; i++)
  {
    this->_observation_model.Zv[i] = sqr(this->_measurement->getJointPositionUncertainty());
  }
  for (int i = 3; i < NX; i++)
  {
    this->_observation_model.Zv[i] = sqr(this->_measurement->getJointOrientationUncertainty());
  }

  this->_ekf.observe(this->_observation_model, this->_last_z);
  this->_ekf.update();

  this->_axis.position->setX(this->_ekf.x[0]);
  this->_axis.position->setY(this->_ekf.x[1]);
  this->_axis.position->setZ(this->_ekf.x[2]);
  this->_axis.orientation->setX(this->_ekf.x[3]);
  this->_axis.orientation->setY(this->_ekf.x[4]);
  this->_axis.orientation->setZ(this->_ekf.x[5]);
}

double RecursiveRevoluteJoint::getJointOrientationUncertainty() const
{
  double ori_norm = this->_axis.orientation->norm();
  this->_axis.orientation->setX(this->_axis.orientation->getX() / ori_norm);
  this->_axis.orientation->setY(this->_axis.orientation->getY() / ori_norm);
  this->_axis.orientation->setZ(this->_axis.orientation->getZ() / ori_norm);

  vision::FeaturePtr deviated_vector(
                                     new vision::Feature(this->_axis.orientation->getX() + sqrt(this->_ekf.X(3, 3)),
                                                         this->_axis.orientation->getY() + sqrt(this->_ekf.X(4, 4)),
                                                         this->_axis.orientation->getZ() + sqrt(this->_ekf.X(5, 5))));
  double norm_dev = deviated_vector->norm();
  deviated_vector->setX(deviated_vector->getX() / norm_dev);
  deviated_vector->setY(deviated_vector->getY() / norm_dev);
  deviated_vector->setZ(deviated_vector->getZ() / norm_dev);

  double max_std_dev = acos(this->_axis.orientation->dot(deviated_vector));
  ROS_INFO_STREAM_NAMED("RecursiveJoint.getMaximumDeviation",
                        "Maximum standard deviation wrt. the believed axis of motion [radians]: " << max_std_dev);
  return max_std_dev;
}

double RecursiveRevoluteJoint::getJointPositionUncertainty() const
{
  return this->_ekf.X(0, 0);
}

/*
 *   double ori_meas_std_x = std::max(
 (1 - this->_ekf.x[3])
 * this->_btp->getLocalTrajectory()->getMaxTranslationDistance(), 0.5);
 double ori_meas_std_y = std::max(
 (1 - this->_ekf.x[4])
 * this->_btp->getLocalTrajectory()->getMaxTranslationDistance(), 0.5);
 double ori_meas_std_z = std::max(
 (1 - this->_ekf.x[5])
 * this->_btp->getLocalTrajectory()->getMaxTranslationDistance(), 0.5);

 this->_ekf.X(3, 3) = sqr(ori_meas_std_x);
 this->_ekf.X(4, 4) = sqr(ori_meas_std_y);
 this->_ekf.X(5, 5) = sqr(ori_meas_std_z);*/

