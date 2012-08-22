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
 * TransformationBasedJointFactory.cpp
 *
 *  Created on: Apr 23, 2012
 *      Author: shoefer
 */

#include "transformation_based/TransformationBasedJointFactory.h"
#include "transformation_based/TransformationBasedPrismaticJoint.h"
#include "transformation_based/TransformationBasedRevoluteJoint.h"
#include "transformation_based/TransformationBasedRigidJoint.h"
#include "transformation_based/TransformationBasedUncertainJoint.h"
#include "transformation_based/TransformationBasedUndefinedJoint.h"


TransformationBasedJointFactory::TransformationBasedJointFactory()
{

}

TransformationBasedJointFactory::~TransformationBasedJointFactory()
{
}

JointFactory* TransformationBasedJointFactory::getInstance()
{
  if (m_instance_tb == NULL)
  {
    m_instance_tb = new TransformationBasedJointFactory;
  }
  return m_instance_tb;
}

std::vector<JointPtr> TransformationBasedJointFactory::generateJoints(double min_motion) const
{
  std::vector<JointPtr> ret_val;
  ret_val.push_back(JointPtr(new TransformationBasedPrismaticJoint()));
  ret_val.push_back(JointPtr(new TransformationBasedRevoluteJoint()));
  ret_val.push_back(JointPtr(new TransformationBasedRigidJoint()));
  ret_val.push_back(JointPtr(new TransformationBasedUndefinedJoint()));
  ret_val.push_back(JointPtr(new TransformationBasedUncertainJoint()));
  for(int i=0; i<5; i++)
  {
    ret_val.at(i)->setMinimumMotion(min_motion);
  }
  return ret_val;
}
