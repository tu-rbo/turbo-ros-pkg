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
 * RecursiveUndefinedJoint.h
 *
 *  Created on: Apr 23, 2012
 *      Author: roberto
 */

#ifndef RECURSIVEUNDEFINEDJOINT_H_
#define RECURSIVEUNDEFINEDJOINT_H_

#include "UndefinedJoint.h"

#include "recursive/RecursiveJoint.h"

class RecursiveUndefinedJoint;
typedef boost::shared_ptr<RecursiveUndefinedJoint> RecursiveUndefinedJointPtr;


class RecursiveUndefinedJoint : public RecursiveJoint,
  public UndefinedJoint
{
public:
  RecursiveUndefinedJoint();
  RecursiveUndefinedJoint(const RecursiveUndefinedJoint& tbpj);
  virtual ~RecursiveUndefinedJoint();

  virtual void estimateAxis();
  virtual double getGoodnessOfFit();

  RecursiveUndefinedJointPtr clone() const {
    return RecursiveUndefinedJointPtr(doClone());
  }

protected:

  virtual void _initialize();

  virtual void _update();

  virtual RecursiveUndefinedJoint* doClone() const
  {
    return (new RecursiveUndefinedJoint(*this));
  }

};

#endif /* RECURSIVEUNDEFINEDJOINT_H_ */
