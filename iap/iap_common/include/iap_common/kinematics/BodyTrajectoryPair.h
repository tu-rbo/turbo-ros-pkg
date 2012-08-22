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
 * BodyTrajectoryPair.h
 *
 *  Created on: Apr 16, 2012
 *      Author: roberto
 */

#ifndef BODYTRAJECTORYPAIR_H_
#define BODYTRAJECTORYPAIR_H_

#include "BodyTrajectory.h"
#include "iap_common/BodyTrajectoryPairMsg.h"

class BodyTrajectoryPair;
typedef boost::shared_ptr<BodyTrajectoryPair> BodyTrajectoryPairPtr;

class BodyTrajectoryPair
{
public:
  BodyTrajectoryPair();
  BodyTrajectoryPair(const BodyTrajectoryPair& btp);
  BodyTrajectoryPair(BodyTrajectoryPtr bt1, BodyTrajectoryPtr bt2);
  virtual ~BodyTrajectoryPair();

  virtual BodyTrajectoryPtr getBody1Trajectory() const;
  virtual BodyTrajectoryPtr getBody2Trajectory() const;
  virtual BodyTrajectoryPtr getLocalTrajectory(double scaling_factor = 1);

  virtual int getTimeStart() const;
  virtual int getTimeEnd() const;
  virtual int getTrajectoryLength() const;
  virtual int getBodyId1() const;
  virtual int getBodyId2() const;

  virtual BodyTrajectoryPairPtr clone() const;

  /**
   * Transform this object into a ROS message
   */
  virtual iap_common::BodyTrajectoryPairMsg toROSMsg() const;

  /**
   * Set the class from a ROS message
   *
   * (currently not implemented)
   */
  virtual void set(iap_common::BodyTrajectoryPairMsg &);

protected:
  BodyTrajectoryPtr _bt1;
  BodyTrajectoryPtr _bt2;
  BodyTrajectoryPtr _lt;

  virtual BodyTrajectoryPair* doClone() const
  {
    return (new BodyTrajectoryPair(*this));
  }
};

#endif /* BODYTRAJECTORYPAIR_H_ */
