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
 * RigidTransformation.h
 *
 *  Created on: Apr 12, 2012
 *      Author: roberto
 */

#ifndef RIGIDTRANSFORMATION_H_
#define RIGIDTRANSFORMATION_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <vector>

class RigidTransformation;
typedef boost::shared_ptr<RigidTransformation> RigidTransformationPtr;

class RigidTransformation
{
public:
  RigidTransformation();

  RigidTransformation(const RigidTransformation& rt);
  RigidTransformation(Eigen::Matrix4f transformation);
  RigidTransformation(Eigen::Transform<double, 3, Eigen::Affine> transformation);
  virtual ~RigidTransformation();

  virtual Eigen::Vector3d getRotationAxis() const;
  virtual double getRotationAngle() const;

  /**
   * Get the position (one point) of the axis of the rotation of this transformation
   * @return - Vector with one point on the axis of rotation
   */
  virtual Eigen::Vector3d getRotationAxisPosition() const;

  /**
   * Get the rotation as the axis of rotation (Eigen::Vector3d) and the angle of rotation (double)
   * We assure that the angle of rotation is within the interval [0,PI]
   * @return - AngleAxisd object containing the angle and axis of rotation
   */
  virtual Eigen::AngleAxisd getRotationAngleAxis() const;
  virtual Eigen::Quaterniond getRotationQuaternion() const;

  virtual Eigen::Vector3d getTranslation() const;

  virtual Eigen::Transform<double, 3, Eigen::Affine> getTransformationMatrix() const;

  /**
   * Composition of two RigidTransformations
   * @param rt_first - RigidTransformation to be added
   * @param rt_second - RigidTransformation to be added
   * @return - The composition of both RigidTransformations
   */
  friend RigidTransformationPtr operator+(RigidTransformationPtr rt_first, RigidTransformationPtr rt_second)
  {

    return RigidTransformationPtr(
                                  new RigidTransformation(
                                                          rt_first->getTransformationMatrix()
                                                              * (rt_second->getTransformationMatrix())));

  }

  /**
   * Subtract one RigidTranformation to other
   * @param rt_first - RigidTransformation to extract from
   * @param rt_extracted - RigidTransformation to be extracted to this
   * @return - The result of extracting rt_extracted to this
   */
  friend RigidTransformationPtr operator-(RigidTransformationPtr rt_first, RigidTransformationPtr rt_extracted)
  {
    return RigidTransformationPtr(
                                  new RigidTransformation(
                                                          rt_first->getTransformationMatrix()
                                                              * (rt_extracted->getTransformationMatrix().inverse(
                                                                                                                 Eigen::Isometry))));
  }

  /**
   * Clone the whole object and return a copy
   * @return - Clone of the RigidTransformation
   */
  RigidTransformationPtr clone() const;

  virtual void writeToFileRT(std::string file_name) const;

  virtual void appendToFileRT(std::string file_name) const;

protected:
  Eigen::Transform<double, 3, Eigen::Affine> _rigid_transformation;

  virtual RigidTransformation* doClone() const
  {
    return (new RigidTransformation(*this));
  }
};

#endif /* RIGIDTRANSFORMATION_H_ */
