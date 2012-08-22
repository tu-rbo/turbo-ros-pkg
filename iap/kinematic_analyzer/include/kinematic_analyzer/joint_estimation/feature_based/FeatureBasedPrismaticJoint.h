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
 * FeatureBasedPrismaticJoint.h
 *
 *  Created on: Apr 23, 2012
 *      Author: shoefer
 */

#ifndef FEATUREBASEDPRISMATICJOINT_H_
#define FEATUREBASEDPRISMATICJOINT_H_

#include "PrismaticJoint.h"

#include "feature_based/FeatureBasedJoint.h"

class FeatureBasedPrismaticJoint;
typedef boost::shared_ptr<FeatureBasedPrismaticJoint> FeatureBasedPrismaticJointPtr;


class FeatureBasedPrismaticJoint : public FeatureBasedJoint,
  public PrismaticJoint
{
public:
  FeatureBasedPrismaticJoint();
  FeatureBasedPrismaticJoint(const FeatureBasedPrismaticJoint& tbpj);
  virtual ~FeatureBasedPrismaticJoint();

  virtual void estimateAxis();
  virtual double getGoodnessOfFit();

  FeatureBasedPrismaticJointPtr clone() const {
    return FeatureBasedPrismaticJointPtr(doClone());
  }

  /**
   * Transform this object into a ROS message
   */
  virtual ::iap_common::JointMsg toROSMsg();

  /**
   * Get the maximum deviation in radians from the orientation, based on the covariance of the measurement
   * @return - Standard deviation in radians
   */
  virtual double getJointOrientationUncertainty() const;

protected:

  double _line_goodness;
  double _max_translation;
  virtual FeatureBasedPrismaticJoint* doClone() const
  {
    return (new FeatureBasedPrismaticJoint(*this));
  }

};

#endif /* FEATUREBASEDPRISMATICJOINT_H_ */
