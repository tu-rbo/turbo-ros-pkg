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
 * TransformationBasedPrismaticJoint.h
 *
 *  Created on: Apr 23, 2012
 *      Author: shoefer
 */

#ifndef TRANSFORMATIONBASEDREVOLUTEJOINT_H_
#define TRANSFORMATIONBASEDREVOLUTEJOINT_H_

#include "RevoluteJoint.h"

#include "transformation_based/TransformationBasedJoint.h"

class TransformationBasedRevoluteJoint;
typedef boost::shared_ptr<TransformationBasedRevoluteJoint> TransformationBasedRevoluteJointPtr;


class TransformationBasedRevoluteJoint : public TransformationBasedJoint,
  public RevoluteJoint
{
public:
  TransformationBasedRevoluteJoint();
  TransformationBasedRevoluteJoint(const TransformationBasedRevoluteJoint& tbpj);
  virtual ~TransformationBasedRevoluteJoint();

  virtual void estimateAxis();
  virtual double getGoodnessOfFit();

  TransformationBasedRevoluteJointPtr clone() const {
    return TransformationBasedRevoluteJointPtr(doClone());
  }

  /**
   * Transform this object into a ROS message
   */
  virtual iap_common::JointMsg toROSMsg();

  /**
   * Get the maximum deviation in radians from the orientation, based on the covariance of the measurement
   * To be redefined in some derived classes (prismatic and revolute)
   * @return - Standard deviation in radians
   */
  virtual double getJointPositionUncertainty() const;

  /**
   * Get the maximum deviation in radians from the orientation, based on the covariance of the measurement
   * @return - Standard deviation in radians
   */
  virtual double getJointOrientationUncertainty() const;

protected:
  virtual TransformationBasedRevoluteJoint* doClone() const
  {
    return (new TransformationBasedRevoluteJoint(*this));
  }

};

#endif /* TRANSFORMATIONBASEDREVOLUTEJOINT_H_ */
