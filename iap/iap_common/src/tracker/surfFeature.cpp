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
#include "surfFeature.h"
using namespace vision;
using namespace std;

FeaturePtr SURFFeature::clone(){
	FeaturePtr copy(new SURFFeature(_x, _y, laplacian, size, dir, hessian, descriptor, _lost));
	return copy;
}

SURFFeature::SURFFeature(float x, float y, int laplacian, int size, float dir, float hessian, vector<float> descriptor, bool lost){
	this->_x=x;
	this->_y=y;
	this->laplacian=laplacian;
	this->size=size;
	this->dir=dir;
	this->hessian=hessian;
	this->descriptor=descriptor;
	this->_lost=lost;
}
