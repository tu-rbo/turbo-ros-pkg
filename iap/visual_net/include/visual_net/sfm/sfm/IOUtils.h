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
 * IOUtils.h
 *
 *  Created on: Aug 31, 2009
 *      Author: jkclevel
 */

#ifndef IOUTILS_H_
#define IOUTILS_H_

#include "feature.h"
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <vector>

boost::numeric::ublas::vector<double>** readControls(int numFrames);

std::vector<vision::FeaturePtr> *readObservations(int &numFeatures, int &numFrames, std::ifstream &input, int img_w = 640, int img_h = 480);
std::vector<vision::FeaturePtr> *readObservations(int &numFeatures, int &numFrames, int &initFrame, std::ifstream &input, int img_w = 640, int img_h = 480);
std::vector<vision::FeaturePtr> *readCluster(int &numFeatures, int &numFrames, int &initFrame, std::ifstream &input, int img_w = 640, int img_h = 480);
std::vector< std::vector<vision::FeaturePtr> > readCluster(int &numFeatures, int &numFrames, int &initFrame, std::string filename);
void writePositions(std::vector< std::vector<vision::FeaturePtr> > features, std::string filename);

std::vector<vision::FeaturePtr> *readPositions(int numFeatures, int numFrames, std::ifstream &input);
std::vector< std::vector<vision::FeaturePtr> > read3dData(int numFeatures, int numFrames, std::ifstream &input);
std::vector< std::vector<vision::FeaturePtr> > readPositions(std::string filename);
void plotMatrix(boost::numeric::ublas::matrix<double> &m);

#endif /* IOUTILS_H_ */
