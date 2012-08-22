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
 * TransformationBasedUndefinedJoint.h
 *
 *  Created on: Apr 23, 2012
 *      Author: roberto
 */

#ifndef TRANSFORMATIONBASEDUNDEFINEDJOINT_H_
#define TRANSFORMATIONBASEDUNDEFINEDJOINT_H_

#include "UndefinedJoint.h"

#include "transformation_based/TransformationBasedJoint.h"

class TransformationBasedUndefinedJoint;
typedef boost::shared_ptr<TransformationBasedUndefinedJoint> TransformationBasedUndefinedJointPtr;


class TransformationBasedUndefinedJoint : public TransformationBasedJoint,
  public UndefinedJoint
{
public:
  TransformationBasedUndefinedJoint();
  TransformationBasedUndefinedJoint(const TransformationBasedUndefinedJoint& tbpj);
  virtual ~TransformationBasedUndefinedJoint();

  virtual void estimateAxis();
  virtual double getGoodnessOfFit();

  TransformationBasedUndefinedJointPtr clone() const {
    return TransformationBasedUndefinedJointPtr(doClone());
  }

  /**
   * Transform this object into a ROS message
   */
  virtual iap_common::JointMsg toROSMsg();

protected:
  virtual TransformationBasedUndefinedJoint* doClone() const
  {
    return (new TransformationBasedUndefinedJoint(*this));
  }

};

#endif /* TRANSFORMATIONBASEDUNDEFINEDJOINT_H_ */
