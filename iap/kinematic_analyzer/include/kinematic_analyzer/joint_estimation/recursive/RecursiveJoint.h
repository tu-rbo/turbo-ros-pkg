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
 * RecursiveJoint.h
 *
 *  Created on: Apr 23, 2012
 *      Author: roberto
 */

#ifndef RECURSIVEJOINT_H_
#define RECURSIVEJOINT_H_

#include "Joint.h"


#include "FeatureSet.h"
#include "Eigen/Core"

#include "BayesFilter/bayesFlt.hpp"
#include "BayesFilter/covFlt.hpp"
#include "BayesFilter/infFlt.hpp"

#include "Test/random.hpp"
#include <cmath>
#include <iostream>
#include <boost/numeric/ublas/io.hpp>
#include <boost/random.hpp>


namespace
{

namespace FM = Bayesian_filter_matrix;
using namespace FM;

// Choose Filtering Scheme to use
typedef Bayesian_filter::Covariance_scheme FilterScheme;

// Square
template<class scalar>
  inline scalar sqr(scalar x)
  {
    return x * x;
  }

// Constant Dimensions
const unsigned NX = 6; // Filter State Dimension ( Position point -> 3, Orientation vector -> 3)

// Filter Parameters
// Noise on observing system state
//const Float OBS_NOISE = 1.;
// Initial noise on observing system state
const Float i_OBS_NOISE = 1.;
// Prediction Noise: no Prediction noise as perturbation is known
//const Float PREDICTION_NOISE = 1.;
// Filter's Initial state uncertainty: System state is unknown
const Float i_PREDICTION_NOISE = 1.;

const Float i_BELIEF_NOISE = 10.;

}//namespace

/*
 * Prediction Model
 * Linearized state predict model with additive Control input
 */
class Predictor : public Bayesian_filter::Linrz_predict_model
{
  mutable FM::Vec fx;
  mutable FM::Vec motion;
public:
  Predictor();
  void predict(const FM::Vec& u);
  const FM::Vec& f(const FM::Vec& x) const;
  void JacobianFx(const FM::Vec& x);
};

/*
 * Observation Model
 * Linear state predict model
 */
class Observer : public Bayesian_filter::Linear_uncorrelated_observe_model
{
  mutable FM::Vec z_pred;
public:
  Observer();
  const FM::Vec& h(const FM::Vec& x) const;
};

/*
 * State = (w x y z) -> Motion axis (quaternion)
 * X(t+1) = F X(t) + B U(t+1) + noise(Q)
 * Y(t) = H X(t) + noise(R)
 */

class RecursiveJoint;
typedef boost::shared_ptr<RecursiveJoint> RecursiveJointPtr;

class RecursiveJoint : virtual public Joint
{
public:
  RecursiveJoint();
  virtual ~RecursiveJoint();

  virtual void setClusterTrajectories(vision::ClusterTrajectoryPtr ct1,
                                 vision::ClusterTrajectoryPtr ct2);
  virtual void setBodyTrajectoryPair(BodyTrajectoryPairPtr btp);

  virtual void setMeasurement(JointPtr measurement);

  /**
   * Clone the whole object and return a copy
   * @return - Clone of the TransformationBasedJoint
   */
  RecursiveJointPtr clone() const;

  virtual void estimateAxis();

  /**
   * Get the covariance matrix of the internal belief
   * @return - Covariance matrix
   */
  virtual Eigen::Matrix<double, NX, NX> getCovarianceMatrixInternalBelief() const;

  /**
   * Get the maximum deviation in radians from the orientation, based on the covariance of the internal belief
   * @return - Standard deviation in radians
   */
  virtual double getMaximumDeviation() const;

  /**
   * Get the last predicted value
   * @return - Last prediction
   */
  virtual FM::Vec getLastPrediction() const;

  /**
   * Get the covariance of the last prediction
   * @return - Covariance of the last prediction
   */
  virtual FM::Vec getCovarianceLastPrediction() const;

  /**
   * Get the last integrated measurement
   * @return - Last measurement
   */
  virtual FM::Vec getLastMeasurement() const;

  /**
   * Get the covariance of the last integrated measurement
   * @return - Covariance of the last integrated measurement
   */
  virtual FM::Vec getCovarianceLastMeasurement() const;

  virtual visualization_msgs::Marker getAxisUncertaintyMarkerPosition() const;

protected:
  vision::ClusterTrajectoryPtr _ct1;
  vision::ClusterTrajectoryPtr _ct2;
  BodyTrajectoryPairPtr _btp;

  JointPtr _measurement;

  virtual void _initialize() = 0;

  virtual void _predict();

  virtual void _update() = 0;

  virtual void _renormalize();

  Predictor _prediction_model;
  Observer _observation_model;
  FilterScheme _ekf;

  virtual RecursiveJoint* doClone() const = 0;

  double _amount_of_motion;

  FM::Vec _last_z;
  FM::Vec _last_prediction;

  bool _initialized;


};

#endif /* RECURSIVEJOINT_H_ */
