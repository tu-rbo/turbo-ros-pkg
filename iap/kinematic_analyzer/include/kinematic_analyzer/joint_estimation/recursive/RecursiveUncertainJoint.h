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
 * RecursiveUncertainJoint.h
 *
 *  Created on: Apr 23, 2012
 *      Author: roberto
 */

#ifndef RECURSIVEUNCERTAINJOINT_H_
#define RECURSIVEUNCERTAINJOINT_H_

#include "UncertainJoint.h"

#include "recursive/RecursiveJoint.h"

class RecursiveUncertainJoint;
typedef boost::shared_ptr<RecursiveUncertainJoint> RecursiveUncertainJointPtr;


class RecursiveUncertainJoint : public RecursiveJoint,
  public UncertainJoint
{
public:
  RecursiveUncertainJoint();
  RecursiveUncertainJoint(const RecursiveUncertainJoint& tbpj);
  virtual ~RecursiveUncertainJoint();

  virtual void estimateAxis();
  virtual double getGoodnessOfFit();

  RecursiveUncertainJointPtr clone() const {
    return RecursiveUncertainJointPtr(doClone());
  }

protected:

  virtual void _initialize();

  virtual void _update();

  virtual RecursiveUncertainJoint* doClone() const
  {
    return (new RecursiveUncertainJoint(*this));
  }

};

#endif /* RECURSIVEUNCERTAINJOINT_H_ */
