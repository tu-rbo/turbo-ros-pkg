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
 * ekf.cpp
 *
 *  Created on: 12.07.2010
 *      Author: dermax
 */

#include "ekf.h"
// matrix library from bundler is 130 times faster then boost ublas
#include "../libs/matrix/matrix.h"

#include <time.h>
#include "ros/ros.h"

namespace SFM
{

EKF::EKF(vision::CameraModel& camera) :
  Estimator(camera)
{
  _focal_length = camera.getFocalLength();
  _min_innovation_ekf = 0.5 / (double)cam.getXresolution();
  _max_num_loops_ekf = 50;
  _dt = 1.0 / 15.0;
  _initial_structure_src = BUNDLER_ADJUSTMENT;
}

EKF::~EKF()
{
}

void EKF::_initializeState(std::vector<vision::FeaturePtr> &initial_structure)
{
  _focal_length = (double)cam.getFocalLength() / (double)cam.getXresolution();
  _num_feats_init_structure = initial_structure.size();
  _updated_state = new double[_num_feats_init_structure * 3 + _num_motion_states];
  matrix_zeroes(1, _num_feats_init_structure * 3 + _num_motion_states, _updated_state);
  int i = 0;
  for (std::vector<vision::FeaturePtr>::iterator it = initial_structure.begin(); it != initial_structure.end(); it++)
  {
    _updated_state[i * 3 + 0] = (*it)->getX();
    _updated_state[i * 3 + 1] = (*it)->getY();
    _updated_state[i * 3 + 2] = (*it)->getZ();
    i++;
  }
  //translation
  _updated_state[_num_feats_init_structure * 3 + 0] = 0;
  _updated_state[_num_feats_init_structure * 3 + 1] = 0;
  _updated_state[_num_feats_init_structure * 3 + 2] = 0;
  // rotation
  _updated_state[_num_feats_init_structure * 3 + 3] = 0;
  _updated_state[_num_feats_init_structure * 3 + 4] = 0;
  _updated_state[_num_feats_init_structure * 3 + 5] = 0;
}

std::vector<std::vector<vision::FeaturePtr> > EKF::run(std::vector<vision::FeaturePtr> &initial_structure,
                                                       std::vector<std::vector<vision::FeaturePtr> > &features)
{
  std::vector<std::vector<vision::FeaturePtr> > result;

  // 1 - Estimate scale and set the initial structure
  _initializeState(initial_structure);

  // 2 - Predict the observation given the initial structure
  double* initial_observation = new double[2 * _num_feats_init_structure];
  _predictObservation(_updated_state, initial_observation); // Estimate the 2D points as projection of the 3D points

  // Number of 2D Features (usually it is the same as the number of 3D points)
  int number_feats_2d = features[0].size(); // == initial_structure.size() = numFeatures
  // Storage for indexes of the match between 3D points in the initial structure and 2D Features observed in first frame
  int *min_index = new int[number_feats_2d];
  // Storage of flags that express if a feature was already assigned as a projection of a 3D point
  bool *feature_assigned = new bool[number_feats_2d];
  // Storage of the minimum distance between the prediction of the observation (projected points) and the observation
  double *min_dist = new double[number_feats_2d];
  // Number of frames
  size_t steps = features.size();

  // Scale feature positions to +-1
  std::vector<std::vector<vision::FeaturePtr> > scaled_features;// = features;
  for (size_t frame = 0; frame < steps; frame++)
  {
    std::vector<vision::FeaturePtr> one_frame_scaled;
    for (size_t feat = 0; feat < number_feats_2d; feat++)
    {
      // TODO: Be careful! In all other points (Initializer::getStaticStructure and BundlerInitializer::run) we multiply by default_depth/FocalLength
      // but here we divide by x_resolution
      // (0,0) is at the center of the image
      one_frame_scaled.push_back(features[frame][feat]->cloneAndUpdate(-(float)((features[frame][feat]->getX() - cam.getXresolution() / 2.0)
          / (double)cam.getXresolution()), (float)((features[frame][feat]->getY() - cam.getYresolution() / 2.0)
              / (double)cam.getXresolution())));
      feature_assigned[feat] = false;
    }
    scaled_features.push_back(one_frame_scaled);
  }

  // 3 - Find matching features between the projected initial structure and the Features of the first frame
  double x_observed = 0, y_observed = 0, x_predicted = 0, y_predicted = 0;
  for (size_t k = 0; k < number_feats_2d; k++) // All 2D Features in first frame
  {
    min_index[k] = -1; // Mark as not assigned
    min_dist[k] = 5.0 / (double)cam.getXresolution(); // Set to a big error as initial min distance

    for (int k2 = 0; k2 < _num_feats_init_structure; k2++) // All projected Features of the initial structure
    {
      // Estimate the distance between observation an the projection of the initial structure
      x_observed = scaled_features[0][k]->getX();
      x_predicted = initial_observation[k2 * 2 + 0];
      y_observed = scaled_features[0][k]->getY();
      y_predicted = initial_observation[k2 * 2 + 1];
      double dist = (x_observed - x_predicted) * (x_observed - x_predicted) + (y_observed - y_predicted) * (y_observed
          - y_predicted);

      /* Check if the distance is smaller than 1 pixel and if this projected point was not already assigned.
       * If it was already assigned, check if this assignement had smaller/bigger distance than the distance between
       * the points in this iteration.
       */
      if ((dist < min_dist[k]) && (dist < 1.0 / (double)cam.getXresolution()))
      {
        if (min_index[k] != -1)
        {
          feature_assigned[min_index[k]] = false;
        }
        min_index[k] = k2; // Store the matching
        min_dist[k] = dist; // Store the new minimum distance
        feature_assigned[k2] = true; // Store that this 3D point is matched
      }
    }
  }
  // remove the observations that have no match to the initial structure
  std::vector<std::vector<vision::FeaturePtr> > reordered_features;
  // for all frames
  for (size_t frame = 0; frame < steps; frame++)
  {
    std::vector<vision::FeaturePtr> frame_features;
    // for all features
    for (size_t k = 0; k < features[0].size(); k++)
    {
      if (min_index[k] != -1)
      {
        frame_features.push_back(scaled_features[frame][k]);
      }
    }
    reordered_features.push_back(frame_features);
  }
  // remove the 3D points from the initial structure that have no match to the observations
  std::vector<vision::FeaturePtr> reordered_init_structure;
  // for all points
  for (size_t k = 0; k < features[0].size(); k++)
  {
    if (min_index[k] != -1)
    {
      reordered_init_structure.push_back(initial_structure[min_index[k]]);
    }
  }
  // new number of features
  _num_feats_init_structure = reordered_init_structure.size();
  ROS_INFO("[EKF::run] Found %d matches for EKF.", _num_feats_init_structure);
  if (_num_feats_init_structure == 0)
  {
    //    ROS_ERROR("[EKF::run] EKF could NOT find matching features.");
    //    for(int nf = 0; nf < features.size(); nf++)
    //    {
    //      std::vector<vision::FeaturePtr> one_frame;
    //      for(int ff = 0; ff<initial_structure.size(); ff++)
    //      {
    //        vision::FeaturePtr one_feat = initial_structure.at(ff)->clone();
    //        one_frame.push_back(one_feat);
    //      }
    //      result.push_back(one_frame);
    //    }
    return result;
  }
  // delete the old state vector
  delete _updated_state;
  // initalize the state vector with the initial structure that matches the observed features
  _initializeState(reordered_init_structure);
  // do the ekf ...
  _initVariables();
  int runs;
  double error = 0;
  // Stepping the EKF
  for (size_t frame = 0; (frame < steps); frame++)
  {
    runs = 0;
    // run the ekf with the same observation several times to reduce the prediction error
    do
    {
      clock_t start = clock();
      error = _step(reordered_features[frame]);
      runs++;
    } while ((error > _min_innovation_ekf) && (runs < _max_num_loops_ekf)); // terminate after max_runs or when the mean innovation is less than max_error

    result.push_back(_get3DPoints());
    //TODO: New! Is it working fine?
    for (int idx = 0; idx < this->_num_feats_init_structure; idx++)
    {
      result.rbegin()->at(idx)->setId(reordered_init_structure.at(idx)->getId());
    }
  }

  _freeVariables();
  return result;
}

std::vector<vision::FeaturePtr> EKF::_get3DPoints() const
{
  std::vector<vision::FeaturePtr> result;
  for (int i = 0; i < _num_feats_init_structure; i++)
  {
    vision::FeaturePtr feature(
                               new vision::Feature((float)_updated_state[i * 3 + 0], (float)_updated_state[i * 3 + 1],
                                                   (float)_updated_state[i * 3 + 2]));
    result.push_back(feature);
  }
  return result;
}

void EKF::_initVariables()
{
  _updated_covar = new double[(_num_feats_init_structure * 3 + _num_motion_states) * (_num_feats_init_structure * 3
      + _num_motion_states)];
  matrix_ident(_num_feats_init_structure * 3 + _num_motion_states, _updated_covar);
  matrix_scale(_num_feats_init_structure * 3 + _num_motion_states, _num_feats_init_structure * 3 + _num_motion_states,
               _updated_covar, 0.00001, _updated_covar);

  double Qrot = 1;
  double Qtrans = 1;
  _updated_covar[(_num_feats_init_structure * 3 + _num_motion_states) * (3 * _num_feats_init_structure + 0) + 3
      * _num_feats_init_structure + 0] = Qtrans;
  _updated_covar[(_num_feats_init_structure * 3 + _num_motion_states) * (3 * _num_feats_init_structure + 1) + 3
      * _num_feats_init_structure + 1] = Qtrans;
  _updated_covar[(_num_feats_init_structure * 3 + _num_motion_states) * (3 * _num_feats_init_structure + 2) + 3
      * _num_feats_init_structure + 2] = Qtrans;

  _updated_covar[(_num_feats_init_structure * 3 + _num_motion_states) * (3 * _num_feats_init_structure + 3) + 3
      * _num_feats_init_structure + 3] = Qrot;
  _updated_covar[(_num_feats_init_structure * 3 + _num_motion_states) * (3 * _num_feats_init_structure + 4) + 3
      * _num_feats_init_structure + 4] = Qrot;
  _updated_covar[(_num_feats_init_structure * 3 + _num_motion_states) * (3 * _num_feats_init_structure + 5) + 3
      * _num_feats_init_structure + 5] = Qrot;

  _R_ekf = new double[(_num_feats_init_structure * 2) * (_num_feats_init_structure * 2)];
  matrix_ident(_num_feats_init_structure * 2, _R_ekf);
  matrix_scale(_num_feats_init_structure * 2, _num_feats_init_structure * 2, _R_ekf, 0.01, _R_ekf);

  _Q_ekf = new double[_num_motion_states * _num_motion_states];
  matrix_ident(_num_motion_states, _Q_ekf);
  _Q_ekf[_num_motion_states * 0 + 0] = 1;
  _Q_ekf[_num_motion_states * 1 + 1] = 1;
  _Q_ekf[_num_motion_states * 2 + 2] = 1; // linear
  _Q_ekf[_num_motion_states * 3 + 3] = 3;
  _Q_ekf[_num_motion_states * 4 + 4] = 3;
  _Q_ekf[_num_motion_states * 5 + 5] = 3; // rotation
  matrix_scale(_num_motion_states, _num_motion_states, _Q_ekf, 0.01, _Q_ekf);

  _A_ekf = new double[(3 * _num_feats_init_structure + _num_motion_states) * (3 * _num_feats_init_structure
      + _num_motion_states)];
  matrix_zeroes(3 * _num_feats_init_structure + _num_motion_states, 3 * _num_feats_init_structure + _num_motion_states,
                _A_ekf);

  // _W_ekf doesn't depend on values of the state only the size matters
  _W_ekf = new double[(3 * _num_feats_init_structure + _num_motion_states) * _num_motion_states];
  _getW(_updated_state, _W_ekf);
  _W_ekf_trans = new double[(3 * _num_feats_init_structure + _num_motion_states) * _num_motion_states];
  matrix_transpose(3 * _num_feats_init_structure + _num_motion_states, _num_motion_states, _W_ekf, _W_ekf_trans);

  _W_Q_ekf = new double[(3 * _num_feats_init_structure + _num_motion_states) * _num_motion_states];
  matrix_product(3 * _num_feats_init_structure + _num_motion_states, _num_motion_states, _num_motion_states,
                 _num_motion_states, _W_ekf, _Q_ekf, _W_Q_ekf);
  //_W_Q_W_ekf_trans = ublas::prod(W_Q, W_trans);
  _W_Q_W_ekf_trans = new double[(3 * _num_feats_init_structure + _num_motion_states) * (3 * _num_feats_init_structure
      + _num_motion_states)];
  matrix_product((3 * _num_feats_init_structure + _num_motion_states), _num_motion_states, _num_motion_states,
                 (3 * _num_feats_init_structure + _num_motion_states), _W_Q_ekf, _W_ekf_trans, _W_Q_W_ekf_trans);

  _predicted_state = new double[_num_feats_init_structure * 3 + _num_motion_states];
  matrix_zeroes(1, _num_feats_init_structure * 3 + _num_motion_states, _predicted_state);

  _A_ekf_covar = new double[(3 * _num_feats_init_structure + _num_motion_states) * (3 * _num_feats_init_structure
      + _num_motion_states)];

  _predicted_covar = new double[(3 * _num_feats_init_structure + _num_motion_states) * (3 * _num_feats_init_structure
      + _num_motion_states)];
  // z is the observation vector
  _z = new double[_num_feats_init_structure * 2];
  _H_ekf = new double[(_num_feats_init_structure * 2) * (3 * _num_feats_init_structure + _num_motion_states)];
  matrix_zeroes((_num_feats_init_structure * 2), (3 * _num_feats_init_structure + _num_motion_states), _H_ekf);
  _H_ekf_trans = new double[(3 * _num_feats_init_structure + _num_motion_states) * (_num_feats_init_structure * 2)];

  //H_covar = ublas::prod(H,predCovar);
  _H_ekf_covar = new double[(_num_feats_init_structure * 2) * (3 * _num_feats_init_structure + _num_motion_states)];

  //covarH = ublas::prod(predCovar, H_trans);
  _covar_H_ekf = new double[(3 * _num_feats_init_structure + _num_motion_states) * (_num_feats_init_structure * 2)];
  //HcovarH = ublas::prod(H_covar, H_trans) + R;
  _H_covar_H_ekf = new double[(_num_feats_init_structure * 2) * (_num_feats_init_structure * 2)];

  _K_ekf = new double[(3 * _num_feats_init_structure + _num_motion_states) * (_num_feats_init_structure * 2)];

  _estimated_z = new double[_num_feats_init_structure * 2];
  _innovation = new double[_num_feats_init_structure * 2];

  _I_ekf = new double[(_num_feats_init_structure * 3 + _num_motion_states) * (_num_feats_init_structure * 3
      + _num_motion_states)];
  matrix_ident(_num_feats_init_structure * 3 + _num_motion_states, _I_ekf);
  //KH = ublas::prod(K,H);
  _K_H_ekf = new double[(_num_feats_init_structure * 3 + _num_motion_states) * (_num_feats_init_structure * 3
      + _num_motion_states)];
  //I_KH = (I - KH);
  _I_K_H_ekf = new double[(_num_feats_init_structure * 3 + _num_motion_states) * (_num_feats_init_structure * 3
      + _num_motion_states)];
  //HQH = ublas::matrix<double>(2*numFeatures, 2*numFeatures);
  _H_Q_H_ekf = new double[(2 * _num_feats_init_structure) * (2 * _num_feats_init_structure)];
}

void EKF::_freeVariables()
{
  delete _updated_state;
  delete _updated_covar;
  delete _R_ekf;
  delete _Q_ekf;
  delete _A_ekf;
  delete _W_ekf;
  delete _W_ekf_trans;
  delete _W_Q_ekf;
  delete _W_Q_W_ekf_trans;
  delete _predicted_state;
  delete _A_ekf_covar;
  delete _predicted_covar;
  delete _z;
  delete _H_ekf;
  delete _H_ekf_trans;
  delete _H_ekf_covar;
  delete _covar_H_ekf;
  delete _H_covar_H_ekf;
  delete _K_ekf;
  delete _estimated_z;
  delete _innovation;
  delete _I_ekf;
  delete _K_H_ekf;
  delete _I_K_H_ekf;
  delete _H_Q_H_ekf;
}

double EKF::_step(const std::vector<vision::FeaturePtr> &y)
{
  double mean_innovation;
  if (y.size() != _num_feats_init_structure)
  {
    ROS_ERROR(
        "[EKF::_step] The size of the measurement y differs from the expected number of tracked and matched Features.\nSize of y: %d Expected size (number of matched Features): %d",
        y.size(), _num_feats_init_structure);
    throw std::string(
                      "FATAL ERROR [EKF::_step]: The size of the measurement y differs from the expected number of tracked and matched Features.");
  }

  // Prediction step
  _getA(_updated_state, _A_ekf);
  _predictState(_updated_state, _predicted_state);
  //        predCovar=A*upCovar*A'+WQW;
  matrix_product(3 * _num_feats_init_structure + _num_motion_states,
                 3 * _num_feats_init_structure + _num_motion_states,
                 3 * _num_feats_init_structure + _num_motion_states,
                 3 * _num_feats_init_structure + _num_motion_states, _A_ekf, _updated_covar, _A_ekf_covar);
  matrix_transpose_product2(3 * _num_feats_init_structure + _num_motion_states,
                            3 * _num_feats_init_structure + _num_motion_states,
                            3 * _num_feats_init_structure + _num_motion_states,
                            3 * _num_feats_init_structure + _num_motion_states, _A_ekf_covar, _A_ekf, _predicted_covar);
  matrix_sum(3 * _num_feats_init_structure + _num_motion_states, 3 * _num_feats_init_structure + _num_motion_states,
             3 * _num_feats_init_structure + _num_motion_states, 3 * _num_feats_init_structure + _num_motion_states,
             _predicted_covar, _W_Q_W_ekf_trans, _predicted_covar);
  //update step
  _computeH(_predicted_state, _H_ekf);
  // K=predCovar*H'/(H*predCovar*H'+R);
  // H'
  matrix_transpose(_num_feats_init_structure * 2, 3 * _num_feats_init_structure + _num_motion_states, _H_ekf,
                   _H_ekf_trans);
  // predCovar*H'
  matrix_product(3 * _num_feats_init_structure + _num_motion_states,
                 3 * _num_feats_init_structure + _num_motion_states,
                 3 * _num_feats_init_structure + _num_motion_states, _num_feats_init_structure * 2, _predicted_covar,
                 _H_ekf_trans, _covar_H_ekf);
  // H*predCovar
  matrix_product(_num_feats_init_structure * 2, 3 * _num_feats_init_structure + _num_motion_states,
                 3 * _num_feats_init_structure + _num_motion_states,
                 3 * _num_feats_init_structure + _num_motion_states, _H_ekf, _predicted_covar, _H_ekf_covar);
  // H*predCovar*H'
  matrix_product(_num_feats_init_structure * 2, 3 * _num_feats_init_structure + _num_motion_states,
                 3 * _num_feats_init_structure + _num_motion_states, _num_feats_init_structure * 2, _H_ekf_covar,
                 _H_ekf_trans, _H_covar_H_ekf);
  // (H*predCovar*H'+R)
  matrix_sum(_num_feats_init_structure * 2, _num_feats_init_structure * 2, _num_feats_init_structure * 2,
             _num_feats_init_structure * 2, _H_covar_H_ekf, _R_ekf, _H_covar_H_ekf);
  // inv(H*predCovar*H'+R)
  matrix_invert(_num_feats_init_structure * 2, _H_covar_H_ekf, _H_Q_H_ekf);
  //K=predCovar*H'*inv(H*predCovar*H'+R);
  matrix_product(3 * _num_feats_init_structure + _num_motion_states, _num_feats_init_structure * 2,
                 _num_feats_init_structure * 2, _num_feats_init_structure * 2, _covar_H_ekf, _H_Q_H_ekf, _K_ekf);
  // predict
  _predictObservation(_predicted_state, _estimated_z);
  // measurement
  for (int i = 0; i < _num_feats_init_structure; i++)
  {
    _z[i * 2 + 0] = y[i]->getX();
    _z[i * 2 + 1] = y[i]->getY();
  }
  //innovation = z-z_estimate;
  matrix_diff(1, _num_feats_init_structure * 2, 1, _num_feats_init_structure * 2, _z, _estimated_z, _innovation);

  //		upState=predState+K*(innovation);
  matrix_product(3 * _num_feats_init_structure + _num_motion_states, _num_feats_init_structure * 2,
                 _num_feats_init_structure * 2, 1, _K_ekf, _innovation, _updated_state);
  matrix_sum(1, 3 * _num_feats_init_structure + _num_motion_states, 1,
             3 * _num_feats_init_structure + _num_motion_states, _updated_state, _predicted_state, _updated_state);

  //        upCovar=(eye(3*numFeatures+6)-K*H)*predCovar;
  matrix_product(3 * _num_feats_init_structure + _num_motion_states, _num_feats_init_structure * 2,
                 _num_feats_init_structure * 2, 3 * _num_feats_init_structure + _num_motion_states, _K_ekf, _H_ekf,
                 _K_H_ekf);
  //I_KH = (I - KH);
  matrix_diff(_num_feats_init_structure * 3 + _num_motion_states, _num_feats_init_structure * 3 + _num_motion_states,
              _num_feats_init_structure * 3 + _num_motion_states, _num_feats_init_structure * 3 + _num_motion_states,
              _I_ekf, _K_H_ekf, _I_K_H_ekf);
  matrix_product(_num_feats_init_structure * 3 + _num_motion_states,
                 _num_feats_init_structure * 3 + _num_motion_states,
                 _num_feats_init_structure * 3 + _num_motion_states,
                 _num_feats_init_structure * 3 + _num_motion_states, _I_K_H_ekf, _predicted_covar, _updated_covar);

  // calculate error after correction
  _predictObservation(_updated_state, _estimated_z);
  //innovation = z-z_estimate;
  matrix_diff(1, _num_feats_init_structure * 2, 1, _num_feats_init_structure * 2, _z, _estimated_z, _innovation);
  double dist = 0;
  for (int i = 0; i < _num_feats_init_structure; i++)
  {
    dist += sqrt(pow(_innovation[i * 2 + 0], 2) + pow(_innovation[i * 2 + 1], 2));
  }
  mean_innovation = dist / ((double)_num_feats_init_structure);
  return mean_innovation;
}

void EKF::_predictState(double *state, double *state_new) const
{
  // copy old to new
  matrix_scale(1, 3 * _num_feats_init_structure + _num_motion_states, state, 1.0, state_new);
  double LinVel[3];
  double AngVel[3];

  LinVel[0] = state[3 * _num_feats_init_structure + 0];
  LinVel[1] = state[3 * _num_feats_init_structure + 1];
  LinVel[2] = state[3 * _num_feats_init_structure + 2];

  AngVel[0] = state[3 * _num_feats_init_structure + 3];
  AngVel[1] = state[3 * _num_feats_init_structure + 4];
  AngVel[2] = state[3 * _num_feats_init_structure + 5];

  double curr[3], new_pos[3];
  double R[3 * 3];
  matrix_scale(1, 3, LinVel, _dt, LinVel);
  matrix_scale(1, 3, AngVel, _dt, AngVel);
  RODR(AngVel, R);
  for (int i = 0; i < _num_feats_init_structure; i++)
  {
    curr[0] = state[3 * i + 0];
    curr[1] = state[3 * i + 1];
    curr[2] = state[3 * i + 2];
    matrix_product(3, 3, 3, 1, R, curr, new_pos);
    matrix_sum(3, 1, 3, 1, new_pos, LinVel, new_pos);
    //new_pos = LinVel*dt + ublas::prod(RODR(AngVel*dt),curr);
    state_new[3 * i + 0] = new_pos[0];
    state_new[3 * i + 1] = new_pos[1];
    state_new[3 * i + 2] = new_pos[2];
  }
}

void EKF::_getA(double *state, double *A) const
{
  double LinVel[3];
  double AngVel[3];
  matrix_zeroes(3 * _num_feats_init_structure + _num_motion_states, 3 * _num_feats_init_structure + _num_motion_states,
                A);

  LinVel[0] = state[3 * _num_feats_init_structure + 0];
  LinVel[1] = state[3 * _num_feats_init_structure + 1];
  LinVel[2] = state[3 * _num_feats_init_structure + 2];

  AngVel[0] = state[3 * _num_feats_init_structure + 3];
  AngVel[1] = state[3 * _num_feats_init_structure + 4];
  AngVel[2] = state[3 * _num_feats_init_structure + 5];

  matrix_scale(1, 3, LinVel, _dt, LinVel);
  matrix_scale(1, 3, AngVel, _dt, AngVel);

  double DR[9 * 3];
  double DRx[3 * 3], DRy[3 * 3], DRz[3 * 3];
  double I3[3 * 3];
  matrix_ident(3, I3);
  RODR_derivative(AngVel, DR);

  DRx[0 * 3 + 0] = DR[0 * 3 + 0];
  DRx[0 * 3 + 1] = DR[0 * 3 + 1];
  DRx[0 * 3 + 2] = DR[0 * 3 + 2];
  DRx[1 * 3 + 0] = DR[1 * 3 + 0];
  DRx[1 * 3 + 1] = DR[1 * 3 + 1];
  DRx[1 * 3 + 2] = DR[1 * 3 + 2];
  DRx[2 * 3 + 0] = DR[2 * 3 + 0];
  DRx[2 * 3 + 1] = DR[2 * 3 + 1];
  DRx[2 * 3 + 2] = DR[2 * 3 + 2];

  DRy[0 * 3 + 0] = DR[3 * 3 + 0];
  DRy[0 * 3 + 1] = DR[3 * 3 + 1];
  DRy[0 * 3 + 2] = DR[3 * 3 + 2];
  DRy[1 * 3 + 0] = DR[4 * 3 + 0];
  DRy[1 * 3 + 1] = DR[4 * 3 + 1];
  DRy[1 * 3 + 2] = DR[4 * 3 + 2];
  DRy[2 * 3 + 0] = DR[5 * 3 + 0];
  DRy[2 * 3 + 1] = DR[5 * 3 + 1];
  DRy[2 * 3 + 2] = DR[5 * 3 + 2];

  DRz[0 * 3 + 0] = DR[6 * 3 + 0];
  DRz[0 * 3 + 1] = DR[6 * 3 + 1];
  DRz[0 * 3 + 2] = DR[6 * 3 + 2];
  DRz[1 * 3 + 0] = DR[7 * 3 + 0];
  DRz[1 * 3 + 1] = DR[7 * 3 + 1];
  DRz[1 * 3 + 2] = DR[7 * 3 + 2];
  DRz[2 * 3 + 0] = DR[8 * 3 + 0];
  DRz[2 * 3 + 1] = DR[8 * 3 + 1];
  DRz[2 * 3 + 2] = DR[8 * 3 + 2];

  double R[3 * 3];
  RODR(AngVel, R);

  double P[3];
  double dfdxyz[3 * 3];
  double dfdAVx[3];
  double dfdAVy[3];
  double dfdAVz[3];
  //feature with respect to itself
  //ublas::matrix<double> dfdxyz = ublas::prod(RODR(AngVel*dt),ublas::identity_matrix<double>(3));
  matrix_product(3, 3, 3, 3, R, I3, dfdxyz);
  //diff features wrt. features
  for (int i = 0; i < _num_feats_init_structure; i++)
  {
    P[0] = state[3 * i + 0];
    P[1] = state[3 * i + 1];
    P[2] = state[3 * i + 2];

    A[(3 * i + 0) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * i + 0] = dfdxyz[0 * 3 + 0];
    A[(3 * i + 0) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * i + 1] = dfdxyz[0 * 3 + 1];
    A[(3 * i + 0) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * i + 2] = dfdxyz[0 * 3 + 2];
    A[(3 * i + 1) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * i + 0] = dfdxyz[1 * 3 + 0];
    A[(3 * i + 1) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * i + 1] = dfdxyz[1 * 3 + 1];
    A[(3 * i + 1) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * i + 2] = dfdxyz[1 * 3 + 2];
    A[(3 * i + 2) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * i + 0] = dfdxyz[2 * 3 + 0];
    A[(3 * i + 2) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * i + 1] = dfdxyz[2 * 3 + 1];
    A[(3 * i + 2) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * i + 2] = dfdxyz[2 * 3 + 2];

    //feature with respect to the 9 last variables (linear/angular velocities and translation of center of rotation)
    //feature with respect to linear velocities
    A[(3 * i + 0) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * _num_feats_init_structure] = _dt;
    A[(3 * i + 0) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * _num_feats_init_structure + 1] = 0;
    A[(3 * i + 0) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * _num_feats_init_structure + 2] = 0;
    A[(3 * i + 1) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * _num_feats_init_structure] = 0;
    A[(3 * i + 1) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * _num_feats_init_structure + 1] = _dt;
    A[(3 * i + 1) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * _num_feats_init_structure + 2] = 0;
    A[(3 * i + 2) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * _num_feats_init_structure] = 0;
    A[(3 * i + 2) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * _num_feats_init_structure + 1] = 0;
    A[(3 * i + 2) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * _num_feats_init_structure + 2] = _dt;
    //feature with respect to angular velocities

    //ublas::vector<double> dfdAVx = ublas::prod(ublas::prod(DRx, ublas::identity_matrix<double>(3)),ublas::trans(P));
    matrix_product(3, 3, 3, 1, DRx, P, dfdAVx);
    //ublas::vector<double> dfdAVy = ublas::prod(ublas::prod(DRy, ublas::identity_matrix<double>(3)),ublas::trans(P));
    matrix_product(3, 3, 3, 1, DRy, P, dfdAVy);
    //ublas::vector<double> dfdAVz = ublas::prod(ublas::prod(DRz, ublas::identity_matrix<double>(3)),ublas::trans(P));
    matrix_product(3, 3, 3, 1, DRz, P, dfdAVz);

    A[(3 * i + 0) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * _num_feats_init_structure + 3]
        = dfdAVx[0];
    A[(3 * i + 0) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * _num_feats_init_structure + 4]
        = dfdAVy[0];
    A[(3 * i + 0) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * _num_feats_init_structure + 5]
        = dfdAVz[0];
    A[(3 * i + 1) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * _num_feats_init_structure + 3]
        = dfdAVx[1];
    A[(3 * i + 1) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * _num_feats_init_structure + 4]
        = dfdAVy[1];
    A[(3 * i + 1) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * _num_feats_init_structure + 5]
        = dfdAVz[1];
    A[(3 * i + 2) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * _num_feats_init_structure + 3]
        = dfdAVx[2];
    A[(3 * i + 2) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * _num_feats_init_structure + 4]
        = dfdAVy[2];
    A[(3 * i + 2) * (3 * _num_feats_init_structure + _num_motion_states) + 3 * _num_feats_init_structure + 5]
        = dfdAVz[2];
  }
  //the last 9 variables with respect to themselves
  //dLinVel/dLinVel
  A[(3 * _num_feats_init_structure + 0) * (3 * _num_feats_init_structure + _num_motion_states) + 3
      * _num_feats_init_structure + 0] = 1;
  A[(3 * _num_feats_init_structure + 0) * (3 * _num_feats_init_structure + _num_motion_states) + 3
      * _num_feats_init_structure + 1] = 0;
  A[(3 * _num_feats_init_structure + 0) * (3 * _num_feats_init_structure + _num_motion_states) + 3
      * _num_feats_init_structure + 2] = 0;
  A[(3 * _num_feats_init_structure + 1) * (3 * _num_feats_init_structure + _num_motion_states) + 3
      * _num_feats_init_structure + 0] = 0;
  A[(3 * _num_feats_init_structure + 1) * (3 * _num_feats_init_structure + _num_motion_states) + 3
      * _num_feats_init_structure + 1] = 1;
  A[(3 * _num_feats_init_structure + 1) * (3 * _num_feats_init_structure + _num_motion_states) + 3
      * _num_feats_init_structure + 2] = 0;
  A[(3 * _num_feats_init_structure + 2) * (3 * _num_feats_init_structure + _num_motion_states) + 3
      * _num_feats_init_structure + 0] = 0;
  A[(3 * _num_feats_init_structure + 2) * (3 * _num_feats_init_structure + _num_motion_states) + 3
      * _num_feats_init_structure + 1] = 0;
  A[(3 * _num_feats_init_structure + 2) * (3 * _num_feats_init_structure + _num_motion_states) + 3
      * _num_feats_init_structure + 2] = 1;
  //dAngVel/dAngVel
  A[(3 * _num_feats_init_structure + 3) * (3 * _num_feats_init_structure + _num_motion_states) + 3
      * _num_feats_init_structure + 3] = 1;
  A[(3 * _num_feats_init_structure + 3) * (3 * _num_feats_init_structure + _num_motion_states) + 3
      * _num_feats_init_structure + 4] = 0;
  A[(3 * _num_feats_init_structure + 3) * (3 * _num_feats_init_structure + _num_motion_states) + 3
      * _num_feats_init_structure + 5] = 0;
  A[(3 * _num_feats_init_structure + 4) * (3 * _num_feats_init_structure + _num_motion_states) + 3
      * _num_feats_init_structure + 3] = 0;
  A[(3 * _num_feats_init_structure + 4) * (3 * _num_feats_init_structure + _num_motion_states) + 3
      * _num_feats_init_structure + 4] = 1;
  A[(3 * _num_feats_init_structure + 4) * (3 * _num_feats_init_structure + _num_motion_states) + 3
      * _num_feats_init_structure + 5] = 0;
  A[(3 * _num_feats_init_structure + 5) * (3 * _num_feats_init_structure + _num_motion_states) + 3
      * _num_feats_init_structure + 3] = 0;
  A[(3 * _num_feats_init_structure + 5) * (3 * _num_feats_init_structure + _num_motion_states) + 3
      * _num_feats_init_structure + 4] = 0;
  A[(3 * _num_feats_init_structure + 5) * (3 * _num_feats_init_structure + _num_motion_states) + 3
      * _num_feats_init_structure + 5] = 1;
}

void EKF::_getW(double *state, double *W) const
{
  matrix_zeroes(3 * _num_feats_init_structure + _num_motion_states, _num_motion_states, W);
  W[(3 * _num_feats_init_structure + 0) * 6 + 0] = 1;
  W[(3 * _num_feats_init_structure + 1) * 6 + 1] = 1;
  W[(3 * _num_feats_init_structure + 2) * 6 + 2] = 1;
  W[(3 * _num_feats_init_structure + 3) * 6 + 3] = 1;
  W[(3 * _num_feats_init_structure + 4) * 6 + 4] = 1;
  W[(3 * _num_feats_init_structure + 5) * 6 + 5] = 1;
}

void EKF::_computeH(double *state, double *H) const
{
  for (int i = 0; i < _num_feats_init_structure; i++)
  {
    H[(i * 2 + 0) * (3 * _num_feats_init_structure + _num_motion_states) + i * 3 + 0] = _focal_length
        / state[i * 3 + 2];
    H[(i * 2 + 0) * (3 * _num_feats_init_structure + _num_motion_states) + i * 3 + 2] = _focal_length * (-state[i * 3
        + 0]) / (state[i * 3 + 2] * state[i * 3 + 2]);

    H[(i * 2 + 1) * (3 * _num_feats_init_structure + _num_motion_states) + i * 3 + 1] = _focal_length
        / state[i * 3 + 2];
    H[(i * 2 + 1) * (3 * _num_feats_init_structure + _num_motion_states) + i * 3 + 2] = _focal_length * (-state[i * 3
        + 1]) / (state[i * 3 + 2] * state[i * 3 + 2]);
  }
}

void EKF::_predictObservation(double *state, double *h) const
{
  for (int i = 0; i < _num_feats_init_structure; i++)
  {
    h[i * 2 + 0] = _focal_length * state[i * 3 + 0] / state[i * 3 + 2];
    h[i * 2 + 1] = _focal_length * state[i * 3 + 1] / state[i * 3 + 2];
  }
}

void EKF::RODR(double *OM, double *R) const
{

  double th = sqrt(OM[0] * OM[0] + OM[1] * OM[1] + OM[2] * OM[2]);
  double small = 1e-6;

  if (th < small)
  {
    R[3 * 0 + 0] = 1;
    R[3 * 0 + 1] = 0;
    R[3 * 0 + 2] = 0;
    R[3 * 1 + 0] = 0;
    R[3 * 1 + 1] = 1;
    R[3 * 1 + 2] = 0;
    R[3 * 2 + 0] = 0;
    R[3 * 2 + 1] = 0;
    R[3 * 2 + 2] = 1;
  }
  else
  {
    double x = OM[0] / th;
    double y = OM[1] / th;
    double z = OM[2] / th;

    double xx = x * x, xy = x * y, xz = x * z;
    double yy = y * y, yz = y * z, zz = z * z;
    double yx = xy, zx = xz, zy = yz;

    double sth = sin(th);
    double cth = cos(th);
    double mcth = 1.0 - cth;

    R[3 * 0 + 0] = 1 - mcth * (yy + zz);
    R[3 * 0 + 1] = -sth * z + mcth * yx;
    R[3 * 0 + 2] = sth * y + mcth * xz;
    R[3 * 1 + 0] = sth * z + mcth * xy;
    R[3 * 1 + 1] = 1 - mcth * (zz + xx);
    R[3 * 1 + 2] = -sth * x + mcth * yz;
    R[3 * 2 + 0] = -sth * y + mcth * xz;
    R[3 * 2 + 1] = sth * x + mcth * yz;
    R[3 * 2 + 2] = 1 - mcth * (xx + yy);

  }
}

void EKF::RODR_derivative(double *OM, double *DR) const
{
  double th = sqrt(OM[0] * OM[0] + OM[1] * OM[1] + OM[2] * OM[2]);
  double small = 1e-6;

  if (th < small)
  {
    DR[3 * 0 + 0] = 0;
    DR[3 * 3 + 0] = 0;
    DR[3 * 6 + 0] = 0;
    DR[3 * 1 + 0] = 0;
    DR[3 * 4 + 0] = 0;
    DR[3 * 7 + 0] = 1;
    DR[3 * 2 + 0] = 0;
    DR[3 * 5 + 0] = -1;
    DR[3 * 8 + 0] = 0;

    DR[3 * 0 + 1] = 0;
    DR[3 * 3 + 1] = 0;
    DR[3 * 6 + 1] = -1;
    DR[3 * 1 + 1] = 0;
    DR[3 * 4 + 1] = 0;
    DR[3 * 7 + 1] = 0;
    DR[3 * 2 + 1] = 1;
    DR[3 * 5 + 1] = 0;
    DR[3 * 8 + 1] = 0;

    DR[3 * 0 + 2] = 0;
    DR[3 * 3 + 2] = 1;
    DR[3 * 6 + 2] = 0;
    DR[3 * 1 + 2] = -1;
    DR[3 * 4 + 2] = 0;
    DR[3 * 7 + 2] = 0;
    DR[3 * 2 + 2] = 0;
    DR[3 * 5 + 2] = 0;
    DR[3 * 8 + 2] = 0;
  }
  else
  {
    double x = OM[0] / th;
    double y = OM[1] / th;
    double z = OM[2] / th;

    double xx = x * x, xy = x * y, xz = x * z;
    double yy = y * y, yz = y * z, zz = z * z;
    double yx = xy, zx = xz, zy = yz;

    double sth = sin(th);
    double cth = cos(th);
    double mcth = 1.0 - cth;

    double a = sth / th;
    double b = mcth / th;
    double c = cth - a;
    double d = sth - 2 * b;

    DR[0 * 3 + 0] = -d * (yy + zz) * x;
    DR[3 * 3 + 0] = -2 * b * y - d * (yy + zz) * y;
    DR[6 * 3 + 0] = -2 * b * z - d * (yy + zz) * z;
    DR[1 * 3 + 0] = b * y + c * zx + d * xy * x;
    DR[4 * 3 + 0] = b * x + c * zy + d * xy * y;
    DR[7 * 3 + 0] = a + c * zz + d * xy * z;
    DR[2 * 3 + 0] = b * z - c * yx + d * xz * x;
    DR[5 * 3 + 0] = -a - c * yy + d * xz * y;
    DR[8 * 3 + 0] = b * x - c * yz + d * xz * z;

    DR[0 * 3 + 1] = b * y - c * zx + d * xy * x;
    DR[3 * 3 + 1] = b * x - c * zy + d * xy * y;
    DR[6 * 3 + 1] = -a - c * zz + d * xy * z;
    DR[1 * 3 + 1] = -2 * b * x - d * (zz + xx) * x;
    DR[4 * 3 + 1] = -d * (zz + xx) * y;
    DR[7 * 3 + 1] = -2 * b * z - d * (zz + xx) * z;
    DR[2 * 3 + 1] = a + c * xx + d * yz * x;
    DR[5 * 3 + 1] = b * z + c * xy + d * yz * y;
    DR[8 * 3 + 1] = b * y + c * xz + d * yz * z;

    DR[0 * 3 + 2] = b * z + c * yx + d * zx * x;
    DR[3 * 3 + 2] = a + c * yy + d * zx * y;
    DR[6 * 3 + 2] = b * x + c * yz + d * zx * z;
    DR[1 * 3 + 2] = -a - c * xx + d * zy * x;
    DR[4 * 3 + 2] = b * z - c * xy + d * zy * y;
    DR[7 * 3 + 2] = b * y - c * xz + d * zy * z;
    DR[2 * 3 + 2] = -2 * b * x - d * (yy + xx) * x;
    DR[5 * 3 + 2] = -2 * b * y - d * (yy + xx) * y;
    DR[8 * 3 + 2] = -d * (yy + xx) * z;
  }
}

void EKF::setInitialStructureSource(EKF::InitialStructureSource init_struct_src)
{
  _initial_structure_src = init_struct_src;
}

EKF::InitialStructureSource EKF::getInitialStructureSource()
{
  return _initial_structure_src;
}

}
