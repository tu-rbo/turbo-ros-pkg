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
 * ekf.h
 *
 *  Created on: 12.07.2010
 *      Author: dermax
 */

#ifndef EKF_H_
#define EKF_H_
#include <vector>
#include "CameraModel.h"
#include "feature.h"
#include "Estimator.h"

namespace SFM
{

class EKF : public Estimator
{
public:

  typedef enum InitialStructureSource
  {
    BUNDLER_ADJUSTMENT = 0,
    PREVIOUS_EKF_ITERATION = 1
  }InitialStructureSource;

  /**
   * Constructor
   * @param camera - Provides the focal length and the resolution of the used camera
   */
  EKF(vision::CameraModel& camera);

  /**
   * Destructor
   */
  virtual ~EKF();

  /**
   * Creates 3D features for every feature in every frame using an initial structure estimate
   * and the motion of the features between the frames.
   * the structure will be scaled with some value
   * the features must be in pixel values
   *
   * @param initial_structure - First approximation of the 3D coordinates of the 2D tracked Features
   * @param features - 2D coordinates of the tracked 2D Features at each frame
   * @return Estimation of the 3D coordinates of the tracked 2D points at each frame
   */
  std::vector<std::vector<vision::FeaturePtr> > run(std::vector<vision::FeaturePtr> &initial_structure,
                                                    std::vector<std::vector<vision::FeaturePtr> > &features);
  /**
   * Assigns the source of the initial structure (different sources have different coordinates formats - sign and scale)
   * @param init_struct_src - Source of the initial structure
   */
  void setInitialStructureSource(InitialStructureSource init_struct_src);

  /**
   * Retrieves the source of the initial structure (different sources have different coordinates formats - sign and scale)
   * @return - Source of the initial structure
   */
  InitialStructureSource getInitialStructureSource();

private:

  /** Steps the EKF
   * @param y - Observation as a vector of Features
   * @return - Mean error between prediction an observation
   */
  double _step(const std::vector<vision::FeaturePtr> &y);

  /**
   * Predict the state from the current state
   * @param state - Current (k) state
   * @param state_new - Will contain the state at k+1 (output)
   */
  void _predictState(double *state, double *new_state) const;

  void _getA(double *state, double *A) const;

  void _getW(double *state, double *W) const;

  void _computeH(double *state, double *H) const;

  /**
   * Predict the observation from the belief
   * @param state - The current belief to predict the observation from
   * @param h - Will contain the observation in uv order (output)
   */
  void _predictObservation(double *state, double *h) const;

  /**
   * RODR  Rodrigues' formula
   * R = RODR(OM) computes the Rodrigues' formula of OM, returning the rotation matrix R =
   * expm(hat(OM)) (converted from RODR.m)
   *
   * @param OM - 3-Dimensional column vector
   * @param R - Resulting 3x3 rotation matrix
   */
  void RODR(double *OM, double* R) const;

  /**
   * Computes the derivative of the Rodrigues formula (converted from RODR.m).
   * In matrix notation this is the expression
   *
   *           d(vec expm(hat(OM)) )
   *     dR = ----------------------.
   *                  d OM^T
   *
   * @param OM - 3-Dimensional column vector
   * @param RD - Resulting 9x3 matrix
   */
  void RODR_derivative(double *OM, double *RD) const;

  /**
   * Creates a vector with the estimated 3D coordinates of the 2D tracked Features from the current internal state
   * @return - Vector of the 3D coordinates from the current internal state
   */
  std::vector<vision::FeaturePtr> _get3DPoints() const;

  void _reset(std::vector<vision::FeaturePtr>& initial_structure);

  /**
   * Allocates all variables needed for _step()
   */
  void _initVariables();

  /**
   * Deletes all variables created in _initVariables() for _step()
   */
  void _freeVariables();

  /**
   * Initializes the internal state of the EKF using an initial structure as first belief
   * @param initial_structure - First belief of the 3D coordinates of the tracked Features, that will be refined by the EKF
   */
  void _initializeState(std::vector<vision::FeaturePtr> &initial_structure);

  static const int _num_motion_states = 6;
  int _num_feats_init_structure;
  double _focal_length;
  double _dt;
  InitialStructureSource _initial_structure_src;

  int _max_num_loops_ekf;
  double _min_innovation_ekf;
  double *_A_ekf;
  double *_A_ekf_covar;
  double *_W_ekf;
  double *_W_ekf_trans;
  double *_W_Q_ekf;
  double *_predicted_covar;
  double *_H_ekf;
  double *_H_ekf_trans;
  double *_H_ekf_covar;
  double *_covar_H_ekf;
  double *_H_covar_H_ekf;
  double *_K_ekf;
  double *_I_ekf;
  double *_K_H_ekf;
  double *_I_K_H_ekf;
  double *_H_Q_H_ekf;
  double *_W_Q_W_ekf_trans;
  double *_updated_covar;
  double *_R_ekf;
  double *_Q_ekf;

  double *_predicted_state;
  double *_z;
  double *_estimated_z;
  double *_innovation;
  double *_updated_state;
};

}

#endif /* EKF_H_ */
