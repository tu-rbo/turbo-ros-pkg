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
 * PrismaticJoint.cpp
 *
 *  Created on: Apr 19, 2012
 *      Author: roberto
 */

#include "PrismaticJoint.h"

PrismaticJoint::PrismaticJoint()
{
  // TODO Auto-generated constructor stub

}

PrismaticJoint::PrismaticJoint(const PrismaticJoint& pj) :
Joint(pj)
{
}

PrismaticJoint::~PrismaticJoint()
{
  // TODO Auto-generated destructor stub
}

JointType PrismaticJoint::getType() const
{
  return PRISMATIC_JOINT;
}

std::string PrismaticJoint::getTypeStr() const
{
  return std::string("PRISMATIC");
}

PrismaticJointPtr PrismaticJoint::clone() const
{
  return PrismaticJointPtr(doClone());
}

iap_common::JointMsg PrismaticJoint::toROSMsg()
{
  iap_common::JointMsg msg = Joint::toROSMsg();
  msg.type = msg.PRISMATIC;
  return msg;
}

double PrismaticJoint::getJointPositionUncertainty() const
{
  return MINIMUM_STD_DEV;
}
