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
 * RecursiveJointFactory.cpp
 *
 *  Created on: Apr 23, 2012
 *      Author: shoefer
 */

#include "recursive/RecursiveJointFactory.h"
#include "recursive/RecursivePrismaticJoint.h"
#include "recursive/RecursiveRevoluteJoint.h"
#include "recursive/RecursiveRigidJoint.h"
#include "recursive/RecursiveUncertainJoint.h"
#include "recursive/RecursiveUndefinedJoint.h"

RecursiveJointFactory::RecursiveJointFactory()
{

}

RecursiveJointFactory::~RecursiveJointFactory()
{
}

JointFactory* RecursiveJointFactory::getInstance()
{
  if (m_instance_r == NULL)
  {
    m_instance_r = new RecursiveJointFactory;
  }
  return m_instance_r;
}

std::vector<JointPtr> RecursiveJointFactory::generateJoints(double min_motion) const
{
  std::vector<JointPtr> ret_val;
  ret_val.push_back(JointPtr(new RecursivePrismaticJoint()));
  ret_val.push_back(JointPtr(new RecursiveRevoluteJoint()));
  ret_val.push_back(JointPtr(new RecursiveRigidJoint()));
  ret_val.push_back(JointPtr(new RecursiveUndefinedJoint()));
  ret_val.push_back(JointPtr(new RecursiveUncertainJoint()));
  for(int i=0; i<5; i++)
  {
    ret_val.at(i)->setMinimumMotion(min_motion);
  }
  return ret_val;
}
