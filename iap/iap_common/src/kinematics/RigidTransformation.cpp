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
 * RigidTransformation.cpp
 *
 *  Created on: Apr 12, 2012
 *      Author: roberto
 */

#include "RigidTransformation.h"

#include <iostream>
#include <fstream>
#include <ros/ros.h>

RigidTransformation::RigidTransformation()
{
  // TODO Auto-generated constructor stub
}

RigidTransformation::RigidTransformation(const RigidTransformation& rt)
{
  this->_rigid_transformation = rt.getTransformationMatrix();
}

RigidTransformation::RigidTransformation(Eigen::Matrix4f transformation)
{
  this->_rigid_transformation = Eigen::Transform<double, 3, Eigen::Affine>(transformation.cast<double> ());
}

RigidTransformation::RigidTransformation(Eigen::Transform<double, 3, Eigen::Affine> transformation)
{
  this->_rigid_transformation = transformation;
}

RigidTransformation::~RigidTransformation()
{
  // TODO Auto-generated destructor stub
}

Eigen::Vector3d RigidTransformation::getRotationAxis() const
{
  return this->getRotationAngleAxis().axis();
}

double RigidTransformation::getRotationAngle() const
{
  return this->getRotationAngleAxis().angle();
}

Eigen::Vector3d RigidTransformation::getRotationAxisPosition() const
{
  Eigen::Transform<double, 3, Eigen::Affine> transf_mat = this->getTransformationMatrix();
  Eigen::Vector3d translation_eigen(transf_mat(0, 3), transf_mat(1, 3), transf_mat(2, 3));
  Eigen::Vector3d axis_rot_eigen(transf_mat(2, 1) - transf_mat(1, 2), transf_mat(0, 2) - transf_mat(2, 0),
                                 transf_mat(1, 0) - transf_mat(0, 1));
  double axis_rot_dot_translation = axis_rot_eigen.dot(translation_eigen);
  int axis_rot_dot_translation_signus = 0;
  if (axis_rot_dot_translation < 0)
  {
    axis_rot_dot_translation_signus = -1;
  }
  else if (axis_rot_dot_translation > 0)
  {
    axis_rot_dot_translation_signus = 1;
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("RigidTransformation.getRotationAxisPosition",
        "Dot product between line vector and translation vector is 0");
  }
  double rot_angle = (double)axis_rot_dot_translation_signus * (fabs(acos((transf_mat(0, 0) + transf_mat(1, 1)
      + transf_mat(2, 2) - 1) / 2)));
  double pitch = axis_rot_dot_translation / (2 * rot_angle * sin(rot_angle));
  Eigen::Matrix3d rotation_matrix_eigen;
  rotation_matrix_eigen << transf_mat(0, 0), transf_mat(0, 1), transf_mat(0, 2), transf_mat(1, 0), transf_mat(1, 1), transf_mat(
                                                                                                                                1,
                                                                                                                                2), transf_mat(
                                                                                                                                               2,
                                                                                                                                               0), transf_mat(
                                                                                                                                                              2,
                                                                                                                                                              1), transf_mat(
                                                                                                                                                                             2,
                                                                                                                                                                             2);
  Eigen::Vector3d point_on_rot_axis = (Eigen::Matrix3d::Identity() - rotation_matrix_eigen.transpose())
      * translation_eigen / (2 * (1 - cos(rot_angle)));
  return point_on_rot_axis;
}

Eigen::AngleAxisd RigidTransformation::getRotationAngleAxis() const
{
  Eigen::AngleAxisd angle_axis = Eigen::AngleAxisd(this->_rigid_transformation.linear());
  if (angle_axis.angle() >= M_PI)
  {
    int integer_part = floor(angle_axis.angle() / M_PI);
    double original_angle = angle_axis.angle();
    double rest = fmod(original_angle, M_PI);
    if (integer_part % 2 != 0)
    {
      ROS_DEBUG_STREAM_NAMED("RigidTransformation.getRotationAngleAxis", "Changing orientation of the axis of rotation (+).");
      angle_axis = Eigen::AngleAxisd(M_PI - rest, -angle_axis.axis());
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED("RigidTransformation.getRotationAngleAxis", "Angle must be between 0 and PI (+).");
      angle_axis = Eigen::AngleAxisd(rest, angle_axis.axis());
    }
  }
  else if (angle_axis.angle() < 0.)
  {
    int integer_part = ceil(angle_axis.angle() / M_PI);
    double original_angle = angle_axis.angle();
    double rest = fmod(original_angle, M_PI);
    if (integer_part % 2 != 0)
    {
      ROS_DEBUG_STREAM_NAMED("RigidTransformation.getRotationAngleAxis", "Changing orientation of the axis of rotation (-).");
      angle_axis = Eigen::AngleAxisd(M_PI + rest, -angle_axis.axis());
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED("RigidTransformation.getRotationAngleAxis", "Angle must be between 0 and PI (-).");
      angle_axis = Eigen::AngleAxisd(-rest, -angle_axis.axis());
    }
  }
  return angle_axis;
}

Eigen::Quaterniond RigidTransformation::getRotationQuaternion() const
{
  return Eigen::Quaterniond(this->_rigid_transformation.linear());
}

Eigen::Vector3d RigidTransformation::getTranslation() const
{
  return this->_rigid_transformation.translation();
}

Eigen::Transform<double, 3, Eigen::Affine> RigidTransformation::getTransformationMatrix() const
{
  return this->_rigid_transformation;
}

RigidTransformationPtr RigidTransformation::clone() const
{
  return RigidTransformationPtr(doClone());
}

void RigidTransformation::writeToFileRT(std::string file_name) const
{
  std::ofstream file_to_write;
  file_to_write.open(file_name.c_str(), std::ios_base::trunc);
  if (!file_to_write.is_open())
  {
    ROS_ERROR_NAMED("RigidTransformation.appendToFileRT", "The file couldn't be opened.");
    return;
  }
  file_to_write << this->_rigid_transformation(0, 0) << " " << this->_rigid_transformation(0, 1) << " "
      << this->_rigid_transformation(0, 2) << " " << this->_rigid_transformation(0, 3) << std::endl
      << this->_rigid_transformation(1, 0) << " " << this->_rigid_transformation(1, 1) << " "
      << this->_rigid_transformation(1, 2) << " " << this->_rigid_transformation(1, 3) << std::endl
      << this->_rigid_transformation(2, 0) << " " << this->_rigid_transformation(2, 1) << " "
      << this->_rigid_transformation(2, 2) << " " << this->_rigid_transformation(2, 3) << std::endl
      << this->_rigid_transformation(3, 0) << " " << this->_rigid_transformation(3, 1) << " "
      << this->_rigid_transformation(3, 2) << " " << this->_rigid_transformation(3, 3) << std::endl << std::endl;
  file_to_write.close();
}

void RigidTransformation::appendToFileRT(std::string file_name) const
{
  std::ofstream file_to_write;
  file_to_write.open(file_name.c_str(), std::ios::out | std::ios::app);
  if (!file_to_write.is_open())
  {
    ROS_ERROR_NAMED("RigidTransformation.appendToFileRT", "The file couldn't be opened.");
    return;
  }
  file_to_write << this->_rigid_transformation(0, 0) << " " << this->_rigid_transformation(0, 1) << " "
      << this->_rigid_transformation(0, 2) << " " << this->_rigid_transformation(0, 3) << std::endl
      << this->_rigid_transformation(1, 0) << " " << this->_rigid_transformation(1, 1) << " "
      << this->_rigid_transformation(1, 2) << " " << this->_rigid_transformation(1, 3) << std::endl
      << this->_rigid_transformation(2, 0) << " " << this->_rigid_transformation(2, 1) << " "
      << this->_rigid_transformation(2, 2) << " " << this->_rigid_transformation(2, 3) << std::endl
      << this->_rigid_transformation(3, 0) << " " << this->_rigid_transformation(3, 1) << " "
      << this->_rigid_transformation(3, 2) << " " << this->_rigid_transformation(3, 3) << std::endl << std::endl;
  file_to_write.close();
}
