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
#include "recursive/RecursiveJoint.h"

#include <algorithm>

Predictor::Predictor() :
  Bayesian_filter::Linrz_predict_model(NX, NX), fx(NX), motion(7)
{
  FM::identity(Fx);

  // Setup constant noise model: G is identity
  q[0] = sqr(i_PREDICTION_NOISE);
  q[1] = sqr(i_PREDICTION_NOISE);
  q[2] = sqr(i_PREDICTION_NOISE);
  q[3] = sqr(i_PREDICTION_NOISE);
  q[4] = sqr(i_PREDICTION_NOISE);
  q[5] = sqr(i_PREDICTION_NOISE);

  FM::identity(G);
}

void Predictor::predict(const FM::Vec& u)
{
  motion = u;
}

const FM::Vec& Predictor::f(const FM::Vec& x) const
{
  /*
   (Q * P).w = (w*0 - x*Px - y*Py - z*Pz)
   (Q * P).x = (w*Px + x*0 + y*Pz - z*Py)
   (Q * P).y = (w*Py - x*Pz + y*0 + z*Px)
   (Q * P).z = (w*Pz + x*Py - y*Px + z*0)

   ((Q * P) * Q').w = ((- x*Px - y*Py - z*Pz)*(w) - (w*Px + y*Pz - z*Py)*(-x) - (w*Py - x*Pz + z*Px)*(-y) - (w*Pz + x*Py - y*Px)*(-z))
   ((Q * P) * Q').x = ((- x*Px - y*Py - z*Pz)*(-x) + (w*Px + y*Pz - z*Py)*(w) + (w*Py - x*Pz + z*Px)*(-z) - (w*Pz + x*Py - y*Px)*(-y))
   ((Q * P) * Q').y = ((- x*Px - y*Py - z*Pz)*(-y) - (w*Px + y*Pz - z*Py)*(-z) + (w*Py - x*Pz + z*Px)*(w) + (w*Pz + x*Py - y*Px)*(-x))
   ((Q * P) * Q').z = ((- x*Px - y*Py - z*Pz)*(-z) + (w*Px + y*Pz - z*Py)*(-y) - (w*Py - x*Pz + z*Px)*(-x) + (w*Pz + x*Py - y*Px)*(w))

   (Q * V).w = (w*0 - x*Vx - y*Vy - z*Vz)
   (Q * V).x = (w*Vx + x*0 + y*Vz - z*Vy)
   (Q * V).y = (w*Vy - x*Vz + y*0 + z*Vx)
   (Q * V).z = (w*Vz + x*Vy - y*Vx + z*0)

   ((Q * V) * Q').w = ((- x*Vx - y*Vy - z*Vz)*(w) - (w*Vx + y*Vz - z*Vy)*(-x) - (w*Vy - x*Vz + z*Vx)*(-y) - (w*Vz + x*Vy - y*Vx)*(-z))
   ((Q * V) * Q').x = ((- x*Vx - y*Vy - z*Vz)*(-x) + (w*Vx + y*Vz - z*Vy)*(w) + (w*Vy - x*Vz + z*Vx)*(-z) - (w*Vz + x*Vy - y*Vx)*(-y))
   ((Q * V) * Q').y = ((- x*Vx - y*Vy - z*Vz)*(-y) - (w*Vx + y*Vz - z*Vy)*(-z) + (w*Vy - x*Vz + z*Vx)*(w) + (w*Vz + x*Vy - y*Vx)*(-x))
   ((Q * V) * Q').z = ((- x*Vx - y*Vy - z*Vz)*(-z) + (w*Vx + y*Vz - z*Vy)*(-y) - (w*Vy - x*Vz + z*Vx)*(-x) + (w*Vz + x*Vy - y*Vx)*(w))
   */
  fx[0] = ((-motion[4] * x[0] - motion[5] * x[1] - motion[6] * x[2]) * (-motion[4]) + (motion[3] * x[0] + motion[5]
      * x[2] - motion[6] * x[1]) * (motion[3]) + (motion[3] * x[1] - motion[4] * x[2] + motion[6] * x[0])
      * (-motion[6]) - (motion[3] * x[2] + motion[4] * x[1] - motion[5] * x[0]) * (-motion[5])) + motion[0];
  fx[1] = ((-motion[4] * x[0] - motion[5] * x[1] - motion[6] * x[2]) * (-motion[5]) - (motion[3] * x[0] + motion[5]
      * x[2] - motion[6] * x[1]) * (-motion[6]) + (motion[3] * x[1] - motion[4] * x[2] + motion[6] * x[0])
      * (motion[3]) + (motion[3] * x[2] + motion[4] * x[1] - motion[5] * x[0]) * (-motion[4])) + motion[1];
  fx[2] = ((-motion[4] * x[0] - motion[5] * x[1] - motion[6] * x[2]) * (-motion[6]) + (motion[3] * x[0] + motion[5]
      * x[2] - motion[6] * x[1]) * (-motion[5]) - (motion[3] * x[1] - motion[4] * x[2] + motion[6] * x[0])
      * (-motion[4]) + (motion[3] * x[2] + motion[4] * x[1] - motion[5] * x[0]) * (motion[3])) + motion[2];
  fx[3] = ((-motion[4] * x[3] - motion[5] * x[4] - motion[6] * x[5]) * (-motion[4]) + (motion[3] * x[3] + motion[5]
      * x[5] - motion[6] * x[4]) * (motion[3]) + (motion[3] * x[4] - motion[4] * x[5] + motion[6] * x[3])
      * (-motion[6]) - (motion[3] * x[5] + motion[4] * x[4] - motion[5] * x[3]) * (-motion[5]));
  fx[4] = ((-motion[4] * x[3] - motion[5] * x[4] - motion[6] * x[5]) * (-motion[5]) - (motion[3] * x[3] + motion[5]
      * x[5] - motion[6] * x[4]) * (-motion[6]) + (motion[3] * x[4] - motion[4] * x[5] + motion[6] * x[3])
      * (motion[3]) + (motion[3] * x[5] + motion[4] * x[4] - motion[5] * x[3]) * (-motion[4]));
  fx[5] = ((-motion[4] * x[3] - motion[5] * x[4] - motion[6] * x[5]) * (-motion[6]) + (motion[3] * x[3] + motion[5]
      * x[5] - motion[6] * x[4]) * (-motion[5]) - (motion[3] * x[4] - motion[4] * x[5] + motion[6] * x[3])
      * (-motion[4]) + (motion[3] * x[5] + motion[4] * x[4] - motion[5] * x[3]) * (motion[3]));

  return fx;
}

void Predictor::JacobianFx(const FM::Vec& x)
{ // Linearized model, Jacobian of F at x
  // Fx(i,j) = diff(fx[i]/x[j])

  /*
   * fx[0] = ((- x*Px - y*Py - z*Pz)*(-x) + (w*Px + y*Pz - z*Py)*(w) + (w*Py - x*Pz + z*Px)*(-z) - (w*Pz + x*Py - y*Px)*(-y))
   * fx[1] = ((- x*Px - y*Py - z*Pz)*(-y) - (w*Px + y*Pz - z*Py)*(-z) + (w*Py - x*Pz + z*Px)*(w) + (w*Pz + x*Py - y*Px)*(-x))
   * fx[2] = ((- x*Px - y*Py - z*Pz)*(-z) + (w*Px + y*Pz - z*Py)*(-y) - (w*Py - x*Pz + z*Px)*(-x) + (w*Pz + x*Py - y*Px)*(w))
   * fx[3] = ((- x*Vx - y*Vy - z*Vz)*(-x) + (w*Vx + y*Vz - z*Vy)*(w) + (w*Vy - x*Vz + z*Vx)*(-z) - (w*Vz + x*Vy - y*Vx)*(-y))
   * fx[4] = ((- x*Vx - y*Vy - z*Vz)*(-y) - (w*Vx + y*Vz - z*Vy)*(-z) + (w*Vy - x*Vz + z*Vx)*(w) + (w*Vz + x*Vy - y*Vx)*(-x))
   * fx[5] = ((- x*Vx - y*Vy - z*Vz)*(-z) + (w*Vx + y*Vz - z*Vy)*(-y) - (w*Vy - x*Vz + z*Vx)*(-x) + (w*Vz + x*Vy - y*Vx)*(w))
   *
   * Results with xmaxima: diff(fx[i],j):
   * diff(fx[0]/Px) = w*w + x*x - y*y - z*z
   * diff(fx[0]/Py) = 2*x*y - 2*w*z
   * diff(fx[0]/Pz) = 2*x*z + 2*w*y
   * diff(fx[0]/Vx) = 0
   * diff(fx[0]/Vy) = 0
   * diff(fx[0]/Vz) = 0
   *
   * diff(fx[1]/Px) = 2* w* z + 2* x* y
   * diff(fx[1]/Py) = w*w -x*x + y*y - z*z
   * diff(fx[1]/Pz) = 2 *y* z - 2 *w *x
   * diff(fx[1]/Vx) = 0
   * diff(fx[1]/Vy) = 0
   * diff(fx[1]/Vz) = 0
   *
   * diff(fx[2]/Px) = 2* x* z - 2* w *y
   * diff(fx[2]/Py) = 2 *y *z + 2* w* x
   * diff(fx[2]/Pz) = w*w -x*x - y*y + z*z
   * diff(fx[2]/Vx) = 0
   * diff(fx[2]/Vy) = 0
   * diff(fx[2]/Vz) = 0
   *
   * diff(fx[3]/Px) = 0
   * diff(fx[3]/Py) = 0
   * diff(fx[3]/Pz) = 0
   * diff(fx[3]/Vx) = w*w + x*x - y*y - z*z
   * diff(fx[3]/Vy) = 2*x*y - 2*w*z
   * diff(fx[3]/Vz) = 2*x*z + 2*w*y
   *
   * diff(fx[4]/Px) = 0
   * diff(fx[4]/Py) = 0
   * diff(fx[4]/Pz) = 0
   * diff(fx[4]/Vx) = 2* w* z + 2* x* y
   * diff(fx[4]/Vy) = w*w -x*x + y*y - z*z
   * diff(fx[4]/Vz) = 2 *y* z - 2 *w *x
   *
   * diff(fx[4]/Px) = 0
   * diff(fx[4]/Py) = 0
   * diff(fx[4]/Pz) = 0
   * diff(fx[4]/Vx) = 2* x* z - 2* w *y
   * diff(fx[4]/Vy) = 2 *y *z + 2* w* x
   * diff(fx[4]/Vz) = w*w -x*x - y*y + z*z
   */
  Fx(0, 0) = motion[3] * motion[3] + motion[4] * motion[4] - motion[5] * motion[5] - motion[6] * motion[6];
  Fx(0, 1) = 2 * motion[4] * motion[5] - 2 * motion[3] * motion[6];
  Fx(0, 2) = 2 * motion[4] * motion[6] + 2 * motion[3] * motion[5];
  Fx(0, 3) = 0;
  Fx(0, 4) = 0;
  Fx(0, 5) = 0;

  Fx(1, 0) = 2 * motion[3] * motion[6] + 2 * motion[4] * motion[5];
  Fx(1, 1) = motion[3] * motion[3] - motion[4] * motion[4] + motion[5] * motion[5] - motion[6] * motion[6];
  Fx(1, 2) = 2 * motion[5] * motion[6] - 2 * motion[3] * motion[4];
  Fx(1, 3) = 0;
  Fx(1, 4) = 0;
  Fx(1, 5) = 0;

  Fx(2, 0) = 2 * motion[4] * motion[6] - 2 * motion[3] * motion[5];
  Fx(2, 1) = 2 * motion[5] * motion[6] + 2 * motion[3] * motion[4];
  Fx(2, 2) = motion[3] * motion[3] - motion[4] * motion[4] - motion[5] * motion[5] + motion[6] * motion[6];
  Fx(2, 3) = 0;
  Fx(2, 4) = 0;
  Fx(2, 5) = 0;

  Fx(3, 0) = 0;
  Fx(3, 1) = 0;
  Fx(3, 2) = 0;
  Fx(3, 3) = motion[3] * motion[3] + motion[4] * motion[4] - motion[5] * motion[5] - motion[6] * motion[6];
  Fx(3, 4) = 2 * motion[4] * motion[5] - 2 * motion[3] * motion[6];
  Fx(3, 5) = 2 * motion[4] * motion[6] + 2 * motion[3] * motion[5];

  Fx(4, 0) = 0;
  Fx(4, 1) = 0;
  Fx(4, 2) = 0;
  Fx(4, 3) = 2 * motion[3] * motion[6] + 2 * motion[4] * motion[5];
  Fx(4, 4) = motion[3] * motion[3] - motion[4] * motion[4] + motion[5] * motion[5] - motion[6] * motion[6];
  Fx(4, 5) = 2 * motion[5] * motion[6] - 2 * motion[3] * motion[4];

  Fx(5, 0) = 0;
  Fx(5, 1) = 0;
  Fx(5, 2) = 0;
  Fx(5, 3) = 2 * motion[4] * motion[6] - 2 * motion[3] * motion[5];
  Fx(5, 4) = 2 * motion[5] * motion[6] + 2 * motion[3] * motion[4];
  Fx(5, 5) = motion[3] * motion[3] - motion[4] * motion[4] - motion[5] * motion[5] + motion[6] * motion[6];
}

Observer::Observer() :
  Bayesian_filter::Linear_uncorrelated_observe_model(NX, NX), z_pred(NX)
{
  // Observation noise variance
  // Uncorrelated model -> Z(k) = I * Zv(k) observe noise variance vector Zv
  Zv[0] = sqr(i_OBS_NOISE);
  Zv[1] = sqr(i_OBS_NOISE);
  Zv[2] = sqr(i_OBS_NOISE);
  Zv[3] = sqr(i_OBS_NOISE);
  Zv[4] = sqr(i_OBS_NOISE);
  Zv[5] = sqr(i_OBS_NOISE);

  // Initialize the observation matrix as the identity
  // zp(k) = Hx(k) * x(k|k-1) ,, Hx = I -> zp(k) = x(k|k-1)
  FM::identity(Hx);
}

const FM::Vec& Observer::h(const FM::Vec& x) const
{
  // Linear observation model -> Direct observation -> H is the identity
  z_pred = x;
  return z_pred;
}

RecursiveJoint::RecursiveJoint() :
  _ekf(NX), _prediction_model(), _observation_model(), _amount_of_motion(0), _last_z(NX), _last_prediction(NX),
      _initialized(false)
{
  this->_ekf.X.clear();
  for (int i = 0; i < NX; i++)
  {
    this->_ekf.x[i] = 0.;
    this->_ekf.X(i, i) = sqr(i_BELIEF_NOISE);
  }
  this->_ekf.init();
}

RecursiveJoint::~RecursiveJoint()
{
}

void RecursiveJoint::setClusterTrajectories(vision::ClusterTrajectoryPtr ct1, vision::ClusterTrajectoryPtr ct2)
{
  _ct1 = ct1;
  _ct2 = ct2;
}

void RecursiveJoint::setBodyTrajectoryPair(BodyTrajectoryPairPtr btp)
{
  _btp = btp;
}

void RecursiveJoint::setMeasurement(JointPtr measurement)
{
  this->_measurement = measurement->clone();
}

RecursiveJointPtr RecursiveJoint::clone() const
{
  return RecursiveJointPtr(doClone());
}

void RecursiveJoint::estimateAxis()
{
  if (this->_initialized)
  {
    this->_predict();

    this->_update();

    this->_renormalize();
  }
  else
  {
    this->_initialize();
    this->_initialized = true;
  }
}

visualization_msgs::Marker RecursiveJoint::getAxisUncertaintyMarkerPosition() const
{
  visualization_msgs::Marker rviz_marker;
  rviz_marker.header.frame_id = "/world";
  rviz_marker.header.stamp = ros::Time();
  rviz_marker.ns = "my_namespace";
  rviz_marker.id = 0;
  rviz_marker.type = visualization_msgs::Marker::SPHERE;
  rviz_marker.action = visualization_msgs::Marker::ADD;
  rviz_marker.pose.position.x = this->_axis.position->getX();
  rviz_marker.pose.position.y = this->_axis.position->getY();
  rviz_marker.pose.position.z = this->_axis.position->getZ();
  rviz_marker.pose.orientation.x = 0.0;
  rviz_marker.pose.orientation.y = 0.0;
  rviz_marker.pose.orientation.z = 0.0;
  rviz_marker.pose.orientation.w = 1.0;
  rviz_marker.scale.x = this->_ekf.X(0, 0);
  rviz_marker.scale.y = this->_ekf.X(1, 1);
  rviz_marker.scale.z = this->_ekf.X(2, 2);
  //marker.scale.z = 0.1;
  rviz_marker.color.a = 0.4;
  rviz_marker.color.r = 0.0;
  rviz_marker.color.g = 1.0;
  rviz_marker.color.b = 0.0;
  return rviz_marker;
}

void RecursiveJoint::_predict()
{
  RigidTransformationPtr motion_command =
      this->_btp->getBody1Trajectory()->getRigidTransformation(this->_btp->getTimeEnd());
  FM::Vec u(7);
  u[0] = motion_command->getTranslation().x();
  u[1] = motion_command->getTranslation().y();
  u[2] = motion_command->getTranslation().z();
  u[3] = motion_command->getRotationQuaternion().w();
  u[4] = motion_command->getRotationQuaternion().x();
  u[5] = motion_command->getRotationQuaternion().y();
  u[6] = motion_command->getRotationQuaternion().z();

  double amount_of_translation = motion_command->getTranslation().norm();
  double amount_of_rotation = motion_command->getRotationAngle();
  Eigen::Vector3d normal_to_rotation = motion_command->getRotationAxis();
  normal_to_rotation.normalize();

  //double trans_prediction_noise_std_dev = std::min(1 - 1 / exp(amount_of_translation), 0.01);
  //double rot_prediction_noise_std_dev = std::min(1 - 1 / exp(amount_of_rotation), 0.01);

  //double trans_prediction_noise_std_dev = amount_of_translation/20;
  //double rot_prediction_noise_std_dev = amount_of_rotation/20;

  double trans_pred_std_x = std::max(u[0] / 5., 0.005);
  double trans_pred_std_y = std::max(u[1] / 5., 0.005);
  double trans_pred_std_z = std::max(u[2] / 5., 0.005);

  //  double rot_pred_std_x = std::max((1 - normal_to_rotation.x()) * amount_of_rotation, 0.005);
  //  double rot_pred_std_y = std::max((1 - normal_to_rotation.y()) * amount_of_rotation, 0.005);
  //  double rot_pred_std_z = std::max((1 - normal_to_rotation.z()) * amount_of_rotation, 0.005);
  //
  //  std::cout << "Value (prediction) : " << rot_pred_std_x << " " << rot_pred_std_y << " " << rot_pred_std_z << std::endl;
  double rot_pred_std_x = 0.001;
  double rot_pred_std_y = 0.001;
  double rot_pred_std_z = 0.001;

  // q contains the variance (variance = standardDeviation^2 = sqr(stdDev) )
  this->_prediction_model.q[0] = sqr(trans_pred_std_x);
  this->_prediction_model.q[1] = sqr(trans_pred_std_y);
  this->_prediction_model.q[2] = sqr(trans_pred_std_z);

  this->_prediction_model.q[3] = sqr(rot_pred_std_x);
  this->_prediction_model.q[4] = sqr(rot_pred_std_y);
  this->_prediction_model.q[5] = sqr(rot_pred_std_z);

  this->_prediction_model.predict(u);
  this->_prediction_model.JacobianFx(this->_ekf.x);
  this->_ekf.predict(this->_prediction_model);
  this->_ekf.update();

  // Updates the copy of the prediction
  this->_last_prediction = this->_ekf.x;
}

void RecursiveJoint::_renormalize()
{
  double norm = sqrt(
                     this->_ekf.x[3] * this->_ekf.x[3] + this->_ekf.x[4] * this->_ekf.x[4] + this->_ekf.x[5]
                         * this->_ekf.x[5]);
  this->_ekf.x[3] /= norm;
  this->_ekf.x[4] /= norm;
  this->_ekf.x[5] /= norm;
}

Eigen::Matrix<double, NX, NX> RecursiveJoint::getCovarianceMatrixInternalBelief() const
{
  Eigen::Matrix<double, NX, NX> ret_cov_mat;
  for (int i = 0; i < NX; i++)
  {
    for (int j = 0; j < NX; j++)
    {
      ret_cov_mat(i, j) = this->_ekf.X(i, j);
    }
  }
  return ret_cov_mat;
}

double RecursiveJoint::getMaximumDeviation() const
{
  vision::FeaturePtr deviated_vector(
                                     new vision::Feature(this->_axis.orientation->getX() + sqrt(this->_ekf.X(3, 3)),
                                                         this->_axis.orientation->getY() + sqrt(this->_ekf.X(4, 4)),
                                                         this->_axis.orientation->getZ() + sqrt(this->_ekf.X(5, 5))));
  double norm_dev = deviated_vector->norm();
  deviated_vector->setX(deviated_vector->getX() / norm_dev);
  deviated_vector->setY(deviated_vector->getY() / norm_dev);
  deviated_vector->setZ(deviated_vector->getZ() / norm_dev);
  double max_std_dev = acos(this->_axis.orientation->dot(deviated_vector));
  ROS_INFO_STREAM_NAMED("RecursiveJoint.getMaximumDeviation",
                        "Maximum standard deviation wrt. the believed axis of motion [radians]: " << max_std_dev);
  return max_std_dev;
}

FM::Vec RecursiveJoint::getLastPrediction() const
{
  FM::Vec ret_lp(NX);
  for (int i = 0; i < NX; i++)
  {
    ret_lp[i] = this->_last_prediction[i];
  }
  return ret_lp;
}

FM::Vec RecursiveJoint::getCovarianceLastPrediction() const
{
  FM::Vec ret_cov_lp(NX);
  for (int i = 0; i < NX; i++)
  {
    ret_cov_lp[i] = this->_prediction_model.q[i];
  }
  return ret_cov_lp;
}

FM::Vec RecursiveJoint::getLastMeasurement() const
{
  FM::Vec ret_lm(NX);
  for (int i = 0; i < NX; i++)
  {
    ret_lm[i] = this->_last_z[i];
  }
  return ret_lm;
}

FM::Vec RecursiveJoint::getCovarianceLastMeasurement() const
{
  FM::Vec ret_cov_lm(NX);
  for (int i = 0; i < NX; i++)
  {
    ret_cov_lm[i] = this->_observation_model.Zv[i];
  }
  return ret_cov_lm;
}
