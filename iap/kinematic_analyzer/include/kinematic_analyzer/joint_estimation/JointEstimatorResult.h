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
 * JointEstimatorResult.h
 *
 *  Created on: Apr 19, 2012
 *      Author: roberto
 */

#ifndef JOINTESTIMATORRESULT_H_
#define JOINTESTIMATORRESULT_H_

#include "Joint.h"

#include "iap_common/JointEstimatorResultMsg.h"

class JointEstimatorResult;
typedef boost::shared_ptr<JointEstimatorResult> JointEstimatorResultPtr;

class JointEstimatorResult
{
public:
  /**
   * Constructor
   */
  JointEstimatorResult(int link1=-1, int link2=-1);

  /**
   * Copy constructor
   * @param jer - Reference to the JointEstimatorResult object to be copied
   */
  JointEstimatorResult(const JointEstimatorResult& jer);

  /**
   * Destructor
   */
  virtual ~JointEstimatorResult();

  /**
   * Clone this object and return a smart pointer to the copy
   * @return
   */
  virtual JointEstimatorResultPtr clone() const;


  virtual void setJointHypothesis(JointType type, JointPtr joint);
  virtual JointPtr getJointHypothesis(JointType type) const;

  virtual void setJointProbability(JointType type, double p);
  virtual double getJointProbability(JointType type) const;

  virtual JointPtr getMostProbableJoint() const;
  virtual std::vector<JointPtr> getJoints() const;

  /**
   * Transform this object into a ROS message
   */
  virtual iap_common::JointEstimatorResultMsg toROSMsg();

  /**
   * Set the class from a ROS message
   *
   * (currently not implemented)
   */
  virtual void set(iap_common::JointEstimatorResultMsg &);

//  virtual void estimate(vision::ClusterTrajectoryPtr ct1, vision::ClusterTrajectoryPtr ct2, BodyTrajectoryPairPtr btp, BodyTrajectoryPtr local_traj);

protected:
  std::map<JointType, JointPtr> _joint_hypothesis;
  std::map<JointType, double> _joint_probability;

  int _link1;
  int _link2;

  /**
   * Auxiliar function to handle cloning with inheritance and smart pointers
   * @return - Normal pointer to the clone of this object
   */
  virtual JointEstimatorResult* doClone() const
  {
    return (new JointEstimatorResult(*this));
  }
};

#endif /* JOINTESTIMATORRESULT_H_ */
