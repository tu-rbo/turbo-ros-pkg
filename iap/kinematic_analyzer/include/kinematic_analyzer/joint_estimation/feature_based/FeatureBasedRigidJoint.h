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
 * FeatureBasedRigidJoint.h
 *
 *  Created on: Apr 23, 2012
 *      Author: roberto
 */

#ifndef FEATUREBASEDRIGIDJOINT_H_
#define FEATUREBASEDRIGIDJOINT_H_

#include "RigidJoint.h"

#include "feature_based/FeatureBasedJoint.h"

class FeatureBasedRigidJoint;
typedef boost::shared_ptr<FeatureBasedRigidJoint> FeatureBasedRigidJointPtr;


class FeatureBasedRigidJoint : public FeatureBasedJoint,
  public RigidJoint
{
public:
  FeatureBasedRigidJoint();
  FeatureBasedRigidJoint(const FeatureBasedRigidJoint& tbpj);
  virtual ~FeatureBasedRigidJoint();

  virtual void estimateAxis();
  virtual double getGoodnessOfFit();

  FeatureBasedRigidJointPtr clone() const {
    return FeatureBasedRigidJointPtr(doClone());
  }

  /**
   * Transform this object into a ROS message
   */
  virtual ::iap_common::JointMsg toROSMsg();

protected:
  virtual FeatureBasedRigidJoint* doClone() const
  {
    return (new FeatureBasedRigidJoint(*this));
  }

};

#endif /* FEATUREBASEDRIGIDJOINT_H_ */
