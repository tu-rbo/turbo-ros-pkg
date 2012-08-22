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
 * RecursiveRigidJoint.h
 *
 *  Created on: Apr 23, 2012
 *      Author: roberto
 */

#ifndef RECURSIVERIGIDJOINT_H_
#define RECURSIVERIGIDJOINT_H_

#include "RigidJoint.h"

#include "recursive/RecursiveJoint.h"

class RecursiveRigidJoint;
typedef boost::shared_ptr<RecursiveRigidJoint> RecursiveRigidJointPtr;


class RecursiveRigidJoint : public RecursiveJoint,
  public RigidJoint
{
public:
  RecursiveRigidJoint();
  RecursiveRigidJoint(const RecursiveRigidJoint& tbpj);
  virtual ~RecursiveRigidJoint();

  virtual void estimateAxis();
  virtual double getGoodnessOfFit();

  RecursiveRigidJointPtr clone() const {
    return RecursiveRigidJointPtr(doClone());
  }

protected:

  virtual void _initialize();

  virtual void _update();

  virtual RecursiveRigidJoint* doClone() const
  {
    return (new RecursiveRigidJoint(*this));
  }

};

#endif /* RECURSIVERIGIDJOINT_H_ */
