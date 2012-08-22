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
 * RecursivePrismaticJoint.cpp
 *
 *  Created on: Apr 23, 2012
 *      Author: shoefer
 */

#include "recursive/RecursivePrismaticJoint.h"

#include <algorithm>

using namespace vision;

RecursivePrismaticJoint::RecursivePrismaticJoint() :
  Joint(), RecursiveJoint(), PrismaticJoint()
{
}

RecursivePrismaticJoint::RecursivePrismaticJoint(const RecursivePrismaticJoint& tbpj) :
  Joint(tbpj), RecursiveJoint(tbpj), PrismaticJoint(tbpj)
{
}

RecursivePrismaticJoint::~RecursivePrismaticJoint()
{
}

void RecursivePrismaticJoint::setBodyTrajectoryPair(BodyTrajectoryPairPtr btp)
{
  RecursiveJoint::setBodyTrajectoryPair(btp);
}

double RecursivePrismaticJoint::getGoodnessOfFit()
{
  // TODO
  return -1;
}

void RecursivePrismaticJoint::_initialize()
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

void RecursivePrismaticJoint::_update()
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

  for (int i = 0; i < 3; i++) // Position
  {
    this->_observation_model.Zv[i] = sqr(this->_measurement->getJointPositionUncertainty()); // It can't be exactly 0 or we could have an exception "S not PD (Positive Definite) in observe"
  }
  for (int i = 3; i < NX; i++)
  {
    this->_observation_model.Zv[i] = sqr(this->_measurement->getJointOrientationUncertainty());
  }

  ROS_INFO_STREAM_NAMED("RecursivePrismaticJoint._update",
                        "Clusters " << this->_btp->getBodyId1() << " and " << this->_btp->getBodyId2());
  ROS_INFO_STREAM_NAMED("RecursivePrismaticJoint._update",
                        "MaxTranslationDistance: " << this->_btp->getLocalTrajectory()->getMaxTranslationDistance());
  ROS_INFO_STREAM_NAMED(
                        "RecursivePrismaticJoint._update",
                        "Standard deviation of the orientation in this measurement: "
                            << this->_measurement->getJointOrientationUncertainty());

  this->_ekf.observe(this->_observation_model, this->_last_z);
  this->_ekf.update();

  this->_axis.position->setX(this->_ekf.x[0]);
  this->_axis.position->setY(this->_ekf.x[1]);
  this->_axis.position->setZ(this->_ekf.x[2]);
  this->_axis.orientation->setX(this->_ekf.x[3]);
  this->_axis.orientation->setY(this->_ekf.x[4]);
  this->_axis.orientation->setZ(this->_ekf.x[5]);
}

double RecursivePrismaticJoint::getJointOrientationUncertainty() const
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
  ROS_INFO_STREAM_NAMED("RecursiveJoint.getJointOrientationUncertainty",
                        "Maximum standard deviation wrt. the believed axis of motion [radians]: " << max_std_dev);
  return max_std_dev;
}

double RecursivePrismaticJoint::getJointPositionUncertainty() const
{
  return this->_ekf.X(0, 0);
}

//  double ori_meas_std_x = std::max(
//                                   (1 - this->_ekf.x[3])
//                                       * this->_btp->getLocalTrajectory()->getMaxTranslationDistance(), 0.5);
//  double ori_meas_std_y = std::max(
//                                   (1 - this->_ekf.x[4])
//                                       * this->_btp->getLocalTrajectory()->getMaxTranslationDistance(), 0.5);
//  double ori_meas_std_z = std::max(
//                                   (1 - this->_ekf.x[5])
//                                       * this->_btp->getLocalTrajectory()->getMaxTranslationDistance(), 0.5);


// NOTE: If the axis is not revolute, we trust completely the new position, but not the orientation which will be filtered
// NOTE: We can't do it in this way (we should call init() again because we changed x), so we set the noise of the measurement to 0 for these values.
//  this->_ekf.x[0] = z[0];
//  this->_ekf.x[1] = z[1];
//  this->_ekf.x[2] = z[2];

//double observation_noise = this->_btp->getLocalTrajectory()->getRTwithMaxTranslationDistance() / 10;
//double observation_noise = std::min(1/(10*ori_norm), 0.1);    // The noise is maximal 0.1
//double observation_noise = this->_measurement->getGoodnessOfFit();
//  for (int i = 3; i < NX; i++)
//  {
//    this->_observation_model.Zv[i] = sqr(observation_noise);
//  }

/*
 //Working pretty good for static prismatic and 0.4 min motion
 std::cout << "Value: " << std::max(std::min(1./(exp(abs(50.*this->_btp->getLocalTrajectory()->getMaxTranslationDistance() -0.01))-1.),1.),0.01) << std::endl<< std::endl;
 double ori_meas_std_x = std::max(std::min(1./(exp(abs(50.*this->_btp->getLocalTrajectory()->getMaxTranslationDistance() -0.01))-1.),1.),0.01);
 double ori_meas_std_y = std::max(std::min(1./(exp(abs(50.*this->_btp->getLocalTrajectory()->getMaxTranslationDistance() -0.01))-1.),1.),0.01);
 double ori_meas_std_z = std::max(std::min(1./(exp(abs(50.*this->_btp->getLocalTrajectory()->getMaxTranslationDistance() -0.01))-1.),1.),0.01);
 */

//    double observation_noise =
//        0.01 / (1 - exp(-this->_btp->getLocalTrajectory()->getMaxTranslationDistance()));
//
//  double ori_meas_std_x = std::max(
//                                   (1 - this->_last_z[3])
//                                       * 1/this->_btp->getLocalTrajectory()->getMaxTranslationDistance(), 0.01);
//  double ori_meas_std_y = std::max(
//                                   (1 - this->_last_z[4])
//                                       * 1/this->_btp->getLocalTrajectory()->getMaxTranslationDistance(), 0.01);
//  double ori_meas_std_z = std::max(
//                                   (1 - this->_last_z[5])
//                                       * 1/this->_btp->getLocalTrajectory()->getMaxTranslationDistance(), 0.01);

/*  // Logging the predicted value and the noise of the prediction
 this->_prism_axis_prediction << this->_ekf.x[0] << " " << this->_ekf.x[1] << " " << this->_ekf.x[2] << " "
 << this->_ekf.x[3] << " " << this->_ekf.x[4] << " " << this->_ekf.x[5] << std::endl;

 this->_prism_covariance_prediction << this->_prediction_model.q[0] << " " << this->_prediction_model.q[1] << " "
 << this->_prediction_model.q[2] << " " << this->_prediction_model.q[3] << " " << this->_prediction_model.q[4]
 << " " << this->_prediction_model.q[5] << std::endl;*/

/*  std::stringstream file_clusters_name;
 file_clusters_name << std::string("axes/c") << _btp->getBodyId1() << std::string("c") << _btp->getBodyId2();
 std::string file_prism_root_name = file_clusters_name.str() + std::string("_prism_");

 std::string _prism_axis_internal_belief_name = file_prism_root_name + std::string("axis_ib.txt");
 this->_prism_axis_internal_belief.open(_prism_axis_internal_belief_name.c_str(), std::ios::out | std::ios::app);

 std::string _prism_covariance_internal_belief_name = file_prism_root_name + std::string("cov_ib.txt");
 this->_prism_covariance_internal_belief.open(_prism_covariance_internal_belief_name.c_str(),
 std::ios::out | std::ios::app);

 std::string _prism_axis_measurement_name = file_prism_root_name + std::string("axis_meas.txt");
 this->_prism_axis_measurement.open(_prism_axis_measurement_name.c_str(), std::ios::out | std::ios::app);

 std::string _prism_covariance_measurement_name = file_prism_root_name + std::string("cov_meas.txt");
 this->_prism_covariance_measurement.open(_prism_covariance_measurement_name.c_str(), std::ios::out | std::ios::app);

 std::string _prism_axis_prediction_name = file_prism_root_name + std::string("axis_pred.txt");
 this->_prism_axis_prediction.open(_prism_axis_prediction_name.c_str(), std::ios::out | std::ios::app);

 std::string _prism_covariance_prediction_name = file_prism_root_name + std::string("cov_pred.txt");
 this->_prism_covariance_prediction.open(_prism_covariance_prediction_name.c_str(), std::ios::out | std::ios::app);*/
