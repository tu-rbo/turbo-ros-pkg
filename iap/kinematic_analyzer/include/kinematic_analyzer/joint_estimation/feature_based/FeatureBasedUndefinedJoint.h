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
 * FeatureBasedUndefinedJoint.h
 *
 *  Created on: Apr 23, 2012
 *      Author: roberto
 */

#ifndef FEATUREBASEDUNDEFINEDJOINT_H_
#define FEATUREBASEDUNDEFINEDJOINT_H_

#include "UndefinedJoint.h"

#include "feature_based/FeatureBasedJoint.h"

class FeatureBasedUndefinedJoint;
typedef boost::shared_ptr<FeatureBasedUndefinedJoint> FeatureBasedUndefinedJointPtr;


class FeatureBasedUndefinedJoint : public FeatureBasedJoint,
  public UndefinedJoint
{
public:
  FeatureBasedUndefinedJoint();
  FeatureBasedUndefinedJoint(const FeatureBasedUndefinedJoint& tbpj);
  virtual ~FeatureBasedUndefinedJoint();

  virtual void estimateAxis();
  virtual double getGoodnessOfFit();

  FeatureBasedUndefinedJointPtr clone() const {
    return FeatureBasedUndefinedJointPtr(doClone());
  }

  /**
   * Transform this object into a ROS message
   */
  virtual ::iap_common::JointMsg toROSMsg();

protected:
  virtual FeatureBasedUndefinedJoint* doClone() const
  {
    return (new FeatureBasedUndefinedJoint(*this));
  }

};

#endif /* FEATUREBASEDUNDEFINEDJOINT_H_ */
