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
#include "kinematics.h"

namespace iap_common
{

int mergeJointTypes(int j1, int j2)
{
  // merge joints
  // same joint type
  iap_common::JointMsg jdMsg;

  // prev joint is rigid --> take new observation
  if (j1 == jdMsg.RIGID) {
    return j2;
  }

  // new joint is rigid --> take old observation
  // (probably no motion occurred in this joint this time)
  if (j2 == jdMsg.RIGID) {
    return j1;
  }

  // if one type is uncertain take the other one
  if (j1 == jdMsg.UNCERTAIN) {
    return j2;
  }
  if (j2 == jdMsg.UNCERTAIN) {
    return j1;
  }

  // neither of the two is rigid
  // --> UNDEFINED
  return jdMsg.UNDEFINED;

}

std::ostream& operator<<(std::ostream& stream, const iap_common::KinematicStructureMsg& ks)
{
  for (unsigned int i = 0; i < ks.joint_estimator_results.size(); i++)
  {
    if (i > 0)
      stream << " ";

    iap_common::JointEstimatorResultMsg jdArray = ks.joint_estimator_results[i];
    int best_joint_type = jdArray.best;

    iap_common::JointMsg jdMsg;

    std::string type_str;
    if (best_joint_type == jdMsg.PRISMATIC)
    {
      type_str = "PRISM";
    }
    else if (best_joint_type == jdMsg.REVOLUTE)
    {
      type_str = "REV";
    }
    else if (best_joint_type == jdMsg.RIGID)
    {
      type_str = "RIG";
    }
    else if (best_joint_type == jdMsg.UNCERTAIN)
    {
      type_str = "UNCERT";
    }
    else
    {
      type_str = "UNDEF";
    }

    // joints are commutative!
    stream << type_str << "(" << jdArray.link1 << "," << jdArray.link2 << ")"
        << type_str << "(" << jdArray.link2 << "," << jdArray.link1 << ")";

  }


  return stream;

}

}
