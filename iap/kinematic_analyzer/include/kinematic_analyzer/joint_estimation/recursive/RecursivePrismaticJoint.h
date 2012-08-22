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
 * RecursivePrismaticJoint.h
 *
 *  Created on: Apr 23, 2012
 *      Author: shoefer
 */

#ifndef RECURSIVEPRISMATICJOINT_H_
#define RECURSIVEPRISMATICJOINT_H_

#include "PrismaticJoint.h"

#include "recursive/RecursiveJoint.h"

class RecursivePrismaticJoint;
typedef boost::shared_ptr<RecursivePrismaticJoint> RecursivePrismaticJointPtr;

class RecursivePrismaticJoint : public RecursiveJoint, public PrismaticJoint
{
public:
  RecursivePrismaticJoint();
  RecursivePrismaticJoint(const RecursivePrismaticJoint& tbpj);
  virtual ~RecursivePrismaticJoint();

  //virtual void estimateAxis();
  virtual double getGoodnessOfFit();

  RecursivePrismaticJointPtr clone() const
  {
    return RecursivePrismaticJointPtr(doClone());
  }


  virtual void setBodyTrajectoryPair(BodyTrajectoryPairPtr btp);

  /**
   * Get the maximum deviation in radians from the orientation, based on the covariance of the measurement
   * @return - Standard deviation in radians
   */
  virtual double getJointOrientationUncertainty() const;

  /**
   * Get the maximum deviation in meters from the position, based on the covariance of the measurement
   * @return - Standard deviation in meters
   */
  virtual double getJointPositionUncertainty() const;

protected:

  virtual void _initialize();

  virtual void _update();

  virtual RecursivePrismaticJoint* doClone() const
  {
    return (new RecursivePrismaticJoint(*this));
  }
};

#endif /* RECURSIVEPRISMATICJOINT_H_ */


/*
  std::ofstream _prism_axis_internal_belief;
  std::ofstream _prism_covariance_internal_belief;
  std::ofstream _prism_axis_measurement;
  std::ofstream _prism_covariance_measurement;
  std::ofstream _prism_axis_prediction;
  std::ofstream _prism_covariance_prediction;*/
