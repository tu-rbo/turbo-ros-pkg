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
 * TransformationBasedRigidJoint.h
 *
 *  Created on: Apr 23, 2012
 *      Author: roberto
 */

#ifndef TRANSFORMATIONBASEDRIGIDJOINT_H_
#define TRANSFORMATIONBASEDRIGIDJOINT_H_

#include "RigidJoint.h"

#include "transformation_based/TransformationBasedJoint.h"

class TransformationBasedRigidJoint;
typedef boost::shared_ptr<TransformationBasedRigidJoint> TransformationBasedRigidJointPtr;


class TransformationBasedRigidJoint : public TransformationBasedJoint,
  public RigidJoint
{
public:
  TransformationBasedRigidJoint();
  TransformationBasedRigidJoint(const TransformationBasedRigidJoint& tbpj);
  virtual ~TransformationBasedRigidJoint();

  virtual void estimateAxis();
  virtual double getGoodnessOfFit();

  TransformationBasedRigidJointPtr clone() const {
    return TransformationBasedRigidJointPtr(doClone());
  }

  /**
   * Transform this object into a ROS message
   */
  virtual iap_common::JointMsg toROSMsg();

protected:
  virtual TransformationBasedRigidJoint* doClone() const
  {
    return (new TransformationBasedRigidJoint(*this));
  }

};

#endif /* TRANSFORMATIONBASEDRIGIDJOINT_H_ */
