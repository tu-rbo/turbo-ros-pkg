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
 * FeatureBasedJointFactory.cpp
 *
 *  Created on: Apr 23, 2012
 *      Author: shoefer
 */

#include "feature_based/FeatureBasedJointFactory.h"
#include "feature_based/FeatureBasedPrismaticJoint.h"
#include "feature_based/FeatureBasedRevoluteJoint.h"
#include "feature_based/FeatureBasedRigidJoint.h"
#include "feature_based/FeatureBasedUncertainJoint.h"
#include "feature_based/FeatureBasedUndefinedJoint.h"


FeatureBasedJointFactory::FeatureBasedJointFactory()
{

}

FeatureBasedJointFactory::~FeatureBasedJointFactory()
{
}

JointFactory* FeatureBasedJointFactory::getInstance()
{
  if (m_instance_fb == NULL)
  {
    m_instance_fb = new FeatureBasedJointFactory;
  }
  return m_instance_fb;
}

std::vector<JointPtr> FeatureBasedJointFactory::generateJoints(double min_motion) const
{
  std::vector<JointPtr> ret_val;
  ret_val.push_back(JointPtr(new FeatureBasedPrismaticJoint()));
  ret_val.push_back(JointPtr(new FeatureBasedRevoluteJoint()));
  ret_val.push_back(JointPtr(new FeatureBasedRigidJoint()));
  ret_val.push_back(JointPtr(new FeatureBasedUndefinedJoint()));
  ret_val.push_back(JointPtr(new FeatureBasedUncertainJoint()));
  for(int i=0; i<5; i++)
  {
    ret_val.at(i)->setMinimumMotion(min_motion);
  }
  return ret_val;
}
