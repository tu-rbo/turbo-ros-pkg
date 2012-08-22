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
 * TransformationBasedRevoluteJoint.cpp
 *
 *  Created on: Apr 23, 2012
 *      Author: shoefer
 */

#include "transformation_based/TransformationBasedRevoluteJoint.h"

using namespace vision;

TransformationBasedRevoluteJoint::TransformationBasedRevoluteJoint() :
  Joint(), TransformationBasedJoint(), RevoluteJoint()
{
}

TransformationBasedRevoluteJoint::TransformationBasedRevoluteJoint(const TransformationBasedRevoluteJoint& tbpj) :
  Joint(tbpj), TransformationBasedJoint(tbpj), RevoluteJoint(tbpj)
{

}

TransformationBasedRevoluteJoint::~TransformationBasedRevoluteJoint()
{
}

void TransformationBasedRevoluteJoint::estimateAxis()
{
  // Update the joint definition for the revolute case only if the newly estimated joint (with current_scale)  fits better in a revolute motion pattern
  // Geometric explanation: The transformation matrix (or the equivalent quaternion of rotation and vector of translation) provides all the required information
  // to define the direction and the position of the axis of rotation.
  // REMEMBER: Even pure rotational motions, if the axis of rotation doesn't cross the origin (0,0,0) we will have a translational part != 0
  // The rotational part provides the orientation of the axis and the angle of rotation around it. Use AngleAxis from Eigen.
  // The translational part provides the "offset" of the axis, where it is placed in the 3D space. We build a equations system with the form:
  // tx = (a(v^2+w^2)-u(bv+cw))(1-cos(theta))+(bw-cv)sin(theta)
  // ty = (b(u^2+w^2)-v(au+cw))(1-cos(theta))+(cu-aw)sin(theta)
  // tz = (c(u^2+v^2)-w(au+bv))(1-cos(theta))+(av-bu)sin(theta)
  // where (tx,ty,tz)=t is the translation in the motion, (u,v,w) is the orientation of the axis of rotation and (a,b,c)=x is a point in 3D space crossed by the
  // axis of rotation. Ax = b
  // There are infinite solutions for this system (det(A)=0)), because the solution is a line = axis of rotation, but we only need one (one point).
  // We will use Eigen to decompose the matrix and find one solution
  // NOTE: The first results using Eigen to solve the system were not very accurate. The accuracy is measured as norm(A*xest - b)/norm(b).
  // In an ideal case, the first norm should be 0 because A*xest = b.
  // One partial solution I found is to scale the matrix and b to avoid numerical errors. The results are better.
  // When the angle of rotation is very small (<5 degrees) and the translation too (b),
  // the results are not very reliable (use this information for the RE?)

  // USING screw theory -> Handbook of robotics p. 16

  BodyTrajectoryPtr local_trajectory = this->_btp->getLocalTrajectory();
  RigidTransformationPtr last_local_transform = local_trajectory->getRTwithMaxRotationAngle();
  Eigen::Transform<double, 3, Eigen::Affine> last_local_transform_eigen =
      last_local_transform->getTransformationMatrix();

  Eigen::Vector3d translation_eigen(last_local_transform_eigen(0, 3), last_local_transform_eigen(1, 3),
                                    last_local_transform_eigen(2, 3));
  Eigen::Vector3d axis_rot_eigen(last_local_transform_eigen(2, 1) - last_local_transform_eigen(1, 2),
                                 last_local_transform_eigen(0, 2) - last_local_transform_eigen(2, 0),
                                 last_local_transform_eigen(1, 0) - last_local_transform_eigen(0, 1));
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
    ROS_ERROR_STREAM_NAMED("TransformationBasedRevoluteJoint.estimateAxis",
                           "Dot product between line vector and translation vector is 0");
  }
  double rot_angle = (double)axis_rot_dot_translation_signus * (fabs(
                                                                     acos(
                                                                          (last_local_transform_eigen(0, 0)
                                                                              + last_local_transform_eigen(1, 1)
                                                                              + last_local_transform_eigen(2, 2) - 1)
                                                                              / 2)));
  double pitch = axis_rot_dot_translation / (2 * rot_angle * sin(rot_angle));
  Eigen::Matrix3d rotation_matrix_eigen;
  rotation_matrix_eigen << last_local_transform_eigen(0, 0), last_local_transform_eigen(0, 1), last_local_transform_eigen(
                                                                                                                          0,
                                                                                                                          2), last_local_transform_eigen(
                                                                                                                                                         1,
                                                                                                                                                         0), last_local_transform_eigen(
                                                                                                                                                                                        1,
                                                                                                                                                                                        1), last_local_transform_eigen(
                                                                                                                                                                                                                       1,
                                                                                                                                                                                                                       2), last_local_transform_eigen(
                                                                                                                                                                                                                                                      2,
                                                                                                                                                                                                                                                      0), last_local_transform_eigen(
                                                                                                                                                                                                                                                                                     2,
                                                                                                                                                                                                                                                                                     1), last_local_transform_eigen(
                                                                                                                                                                                                                                                                                                                    2,
                                                                                                                                                                                                                                                                                                                    2);
  Eigen::Vector3d point_on_rot_axis = (Eigen::Matrix3d::Identity() - rotation_matrix_eigen.transpose())
      * translation_eigen / (2 * (1 - cos(rot_angle)));
  Eigen::Vector3d angular_velocity = axis_rot_eigen / (2 * sin(rot_angle));

//  Eigen::AngleAxisd last_local_angle_axis = last_local_transform->getRotationAngleAxis();
//  double last_local_angle = last_local_angle_axis.angle(); // In radians
//  Eigen::Vector3d last_local_axis = last_local_angle_axis.axis(); // 3D vector

  axis_rot_eigen.normalize();

  this->_axis.orientation = FeaturePtr(new Feature(axis_rot_eigen(0), axis_rot_eigen(1), axis_rot_eigen(2)));
  this->_axis.position = FeaturePtr(new Feature(point_on_rot_axis(0), point_on_rot_axis(1), point_on_rot_axis(2)));
}

double TransformationBasedRevoluteJoint::getGoodnessOfFit()
{
  // TODO
  return -1;
}

iap_common::JointMsg TransformationBasedRevoluteJoint::toROSMsg()
{
  iap_common::JointMsg msg = RevoluteJoint::toROSMsg();
  TransformationBasedJoint::toROSMsg(msg);
  return msg;
}

double TransformationBasedRevoluteJoint::getJointOrientationUncertainty() const
{
  //double observation_noise = std::min(0.001/(1-exp(-10*this->_btp->getLocalTrajectory()->getMaxRotationAngle())), 0.2);
  double observation_noise = this->_btp->getLocalTrajectory()->getMaxRotationAngle() / 20.0;
  return observation_noise;
}

double TransformationBasedRevoluteJoint::getJointPositionUncertainty() const
{
  //double observation_noise = std::min(0.001/(1-exp(-10*this->_btp->getLocalTrajectory()->getMaxRotationAngle())), 0.2);
  double observation_noise = this->_btp->getLocalTrajectory()->getMaxRotationAngle() / 20.0;
  return observation_noise;
}
