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

#ifndef FEATUREBASEDREVOLUTEJOINT_H_
#define FEATUREBASEDREVOLUTEJOINT_H_

#include "RevoluteJoint.h"

#include "feature_based/FeatureBasedJoint.h"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/io.hpp>

class FeatureBasedRevoluteJoint;
typedef boost::shared_ptr<FeatureBasedRevoluteJoint> FeatureBasedRevoluteJointPtr;

class FeatureBasedRevoluteJoint : public FeatureBasedJoint, public RevoluteJoint
{
public:
  FeatureBasedRevoluteJoint();
  FeatureBasedRevoluteJoint(const FeatureBasedRevoluteJoint& tbpj);
  virtual ~FeatureBasedRevoluteJoint();

  virtual void estimateAxis();
  virtual double getGoodnessOfFit();

  FeatureBasedRevoluteJointPtr clone() const
  {
    return FeatureBasedRevoluteJointPtr(doClone());
  }

  /**
   * Transform this object into a ROS message
   */
  virtual ::iap_common::JointMsg toROSMsg();

  /**
   * Get the maximum deviation in radians from the orientation, based on the covariance of the measurement
   * To be redefined in some derived classes (prismatic and revolute)
   * @return - Standard deviation in radians
   */
  virtual double getJointPositionUncertainty() const;

  /**
   * Get the maximum deviation in radians from the orientation, based on the covariance of the measurement
   * @return - Standard deviation in radians
   */
  virtual double getJointOrientationUncertainty() const;

protected:

  double _circle_goodness;

  virtual FeatureBasedRevoluteJoint* doClone() const
  {
    return (new FeatureBasedRevoluteJoint(*this));
  }
  std::vector<double> fitPlane(std::vector<vision::FeaturePtr> &featureVec);

  boost::numeric::ublas::matrix<double> magic_cosini(double A, double B, double C);

  std::vector<double> circleFitByTaubin(std::vector<vision::FeaturePtr> &f);

};

#endif /* FEATUREBASEDREVOLUTEJOINT_H_ */
