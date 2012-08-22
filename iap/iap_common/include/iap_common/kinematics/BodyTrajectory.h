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
 * BodyTrajectory.h
 *
 *  Created on: Apr 12, 2012
 *      Author: roberto
 */

#ifndef BODYTRAJECTORY_H_
#define BODYTRAJECTORY_H_

#include "RigidTransformation.h"
#include "iap_common/BodyTrajectoryMsg.h"

#include <vector>

class BodyTrajectory;
typedef boost::shared_ptr<BodyTrajectory> BodyTrajectoryPtr;

class BodyTrajectory
{
public:
  BodyTrajectory();
  BodyTrajectory(const BodyTrajectory& bt);
  BodyTrajectory(int time_start, std::vector<RigidTransformationPtr> transformations, int body_id);
  virtual ~BodyTrajectory();

  virtual int getTimeStart() const;
  virtual int getTimeEnd() const;
  virtual int getTrajectoryLength() const;
  virtual int getBodyId() const;

  virtual double getAccRotationAngle();
  virtual double getMaxRotationAngle();

  /**
   * Get the RigidTransformation of this BodyTrajectory that has the largest rotation angle
   * @return - RigidTransformation with largest rotation angle
   */
  virtual RigidTransformationPtr getRTwithMaxRotationAngle();

  virtual double getRotationAxisParallelism();
  virtual double getAccTranslationDistance();
  virtual double getMaxTranslationDistance();

  /**
   * Get the RigidTransformation of this BodyTrajectory that has the largest translation distance
   * @return - RigidTransformation with largest translation distance
   */
  virtual RigidTransformationPtr getRTwithMaxTranslationDistance();
  virtual double getTranslationAxisParallelism();

  virtual RigidTransformationPtr getRigidTransformation(int frame) const;

  virtual std::vector<RigidTransformationPtr> getRigidTransformations() const;

  /**
   * Clone the whole object and return a copy
   * @return - Clone of the Joint
   */
  BodyTrajectoryPtr clone() const;

  virtual void writeToFileBT(std::string file_name) const;

  /**
   * Transform this object into a ROS message
   *
   * Note: Cannot be const because might invoke estimate*** methods
   */
  virtual iap_common::BodyTrajectoryMsg toROSMsg();

  /**
   * Set the class from a ROS message
   *
   * (currently not implemented)
   */
  virtual void set(iap_common::BodyTrajectoryMsg&);

  /**
   * Estimate the similarity of the transformation matrices to the Identity
   * The first value is the similarity of the whole T (4x4) to I (4x4) in all frames
   * The second value is the similarity of the rotation matrix R (3x3) to I (3x3) when T is not the identity (there is motion)
   * The third value is the similarity of the translation vector t (3x1) to the zero vector (3x1) when T is not the identity
   * 1 means that they are exactly the same for all frames, 0 means that they are different for almost all the frames.
   * @return
   */
  virtual std::vector<double> getSimilaritiesToI() const;

private:
  std::vector<RigidTransformationPtr> _transformations;
  int _time_start;
  int _body_id;

  int _max_rot_frame;
  int _max_trans_frame;

  bool _rot_params_estimated;
  double _acc_rot_angle;
  double _max_rot_angle;
  double _rot_axis_parall;
  double _rot_axis_distance;
  virtual void estimateRotationParameters();

  bool _trans_params_estimated;
  double _acc_trans_dist;
  double _max_trans_dist;
  double _trans_axis_parall;
  virtual void estimateTranslationParameters();

  virtual BodyTrajectory* doClone() const
  {
    return (new BodyTrajectory(*this));
  }
};

#endif /* BODYTRAJECTORY_H_ */
