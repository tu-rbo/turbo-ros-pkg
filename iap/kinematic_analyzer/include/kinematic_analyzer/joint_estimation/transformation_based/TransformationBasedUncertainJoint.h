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
 * TransformationBasedUncertainJoint.h
 *
 *  Created on: Apr 23, 2012
 *      Author: roberto
 */

#ifndef TRANSFORMATIONBASEDUNCERTAINJOINT_H_
#define TRANSFORMATIONBASEDUNCERTAINJOINT_H_

#include "UncertainJoint.h"

#include "transformation_based/TransformationBasedJoint.h"

class TransformationBasedUncertainJoint;
typedef boost::shared_ptr<TransformationBasedUncertainJoint> TransformationBasedUncertainJointPtr;


class TransformationBasedUncertainJoint : public TransformationBasedJoint,
  public UncertainJoint
{
public:
  TransformationBasedUncertainJoint();
  TransformationBasedUncertainJoint(const TransformationBasedUncertainJoint& tbpj);
  virtual ~TransformationBasedUncertainJoint();

  virtual void estimateAxis();
  virtual double getGoodnessOfFit();

  TransformationBasedUncertainJointPtr clone() const {
    return TransformationBasedUncertainJointPtr(doClone());
  }

  /**
   * Transform this object into a ROS message
   */
  virtual iap_common::JointMsg toROSMsg();

protected:
  virtual TransformationBasedUncertainJoint* doClone() const
  {
    return (new TransformationBasedUncertainJoint(*this));
  }

};

#endif /* TRANSFORMATIONBASEDUNCERTAINJOINT_H_ */
