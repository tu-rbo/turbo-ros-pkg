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
 *  Copyright (c) 2008-2010  Noah Snavely (snavely (at) cs.cornell.edu)
 *    and the University of Washington
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

/* Epipolar.h */
/* Routines for computing epipolar geometry */

#ifndef __epipolar_h__
#define __epipolar_h__

#include "../libs/matrix/vector.h"

#include "keys.h"

/* Estimate an E-matrix from a given set of point matches */
std::vector<int> EstimateEMatrix(const std::vector<Keypoint> &k1, 
				 const std::vector<Keypoint> &k2, 
				 std::vector<KeypointMatch> matches, 
				 int num_trials, double threshold, 
				 double f1, double f2, 
                                 double *E, double *F);

/* Estimate relative pose from a given set of point matches */
int EstimatePose5Point(const std::vector<Keypoint> &k1, 
                       const std::vector<Keypoint> &k2, 
                       std::vector<KeypointMatch> matches, 
                       int num_trials, double threshold, 
                       double *K1, double *K2, 
                       double *R, double *t);

/* Estimate an F-matrix from a given set of point matches */
std::vector<int> EstimateFMatrix(const std::vector<Keypoint> &k1, 
				 const std::vector<Keypoint> &k2, 
				 std::vector<KeypointMatch> matches, 
				 int num_trials, double threshold, 
				 double *F, bool essential = false);

std::vector<int> EstimateFMatrix(const std::vector<KeypointWithDesc> &k1, 
				 const std::vector<KeypointWithDesc> &k2, 
				 std::vector<KeypointMatch> matches, 
				 int num_trials, double threshold, 
				 double *F, bool essential = false);

#endif /* __epipolar_h__ */
