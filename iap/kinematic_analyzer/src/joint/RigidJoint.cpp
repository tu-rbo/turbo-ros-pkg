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
 * RigidJoint.cpp
 *
 *  Created on: Apr 19, 2012
 *      Author: roberto
 */

#include "RigidJoint.h"

RigidJoint::RigidJoint()
{
  // TODO Auto-generated constructor stub

}

RigidJoint::RigidJoint(const RigidJoint& pj):
    Joint(pj)
{
}

RigidJoint::~RigidJoint()
{
  // TODO Auto-generated destructor stub
}

JointType RigidJoint::getType() const
{
  return RIGID_JOINT;
}

std::string RigidJoint::getTypeStr() const
{
  return std::string("RIGID");
}

RigidJointPtr RigidJoint::clone() const
{
  return RigidJointPtr(doClone());
}



iap_common::JointMsg RigidJoint::toROSMsg()
{
  iap_common::JointMsg msg = Joint::toROSMsg();
  msg.type = msg.RIGID;
  return msg;
}
