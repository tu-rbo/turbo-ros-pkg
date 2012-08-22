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
 * FeatureBasedJoint.h
 *
 *  Created on: Apr 23, 2012
 *      Author: shoefer
 */

#ifndef FEATUREBASEDJOINT_H_
#define FEATUREBASEDJOINT_H_

#include "Joint.h"

class FeatureBasedJoint;
typedef boost::shared_ptr<FeatureBasedJoint> FeatureBasedJointPtr;

class FeatureBasedJoint : virtual public Joint
{
public:
  FeatureBasedJoint();
  virtual ~FeatureBasedJoint();

  virtual void setClusterTrajectories(vision::ClusterTrajectoryPtr ct1,
                                 vision::ClusterTrajectoryPtr ct2);
  virtual void setBodyTrajectoryPair(BodyTrajectoryPairPtr btp);

  /**
   * Clone the whole object and return a copy
   * @return - Clone of the FeatureBasedJoint
   */
  FeatureBasedJointPtr clone() const;

  /**
   * Completes a joint ROS message with transformation based information
   */
  virtual ::iap_common::JointMsg toROSMsg(::iap_common::JointMsg&);

protected:
  vision::ClusterTrajectoryPtr _ct1;
  vision::ClusterTrajectoryPtr _ct2;
  BodyTrajectoryPairPtr _btp;

  virtual FeatureBasedJoint* doClone() const = 0;

};

#endif /* FEATUREBASEDJOINT_H_ */
