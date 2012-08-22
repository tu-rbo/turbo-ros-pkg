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
 * Robot.cpp
 *
 *  Created on: Aug 7, 2009
 *      Author: jkclevel
 */

#include "Robot.h"

using namespace vision;

Robot::Robot() {
	cameras = std::vector<CameraModel*>();
	velocity = boost::numeric::ublas::zero_vector<double>(3);
	position = boost::numeric::ublas::zero_vector<double>(3);
}

Robot::Robot(boost::numeric::ublas::vector<double> &position){
	cameras = std::vector<CameraModel*>();
	velocity = boost::numeric::ublas::zero_vector<double>(3);
	this->position = position;
}

Robot::~Robot() {
}

void Robot::addCamera(CameraModel *cam){
	cam->setCameraPos(boost::numeric::ublas::zero_vector<double>(3), this->position);
	cameras.push_back(cam);
}

std::vector<CameraModel*> Robot::getCameras(){
	return cameras;
}

void Robot::step(boost::numeric::ublas::vector<double> &linVel ){
	velocity = linVel;
	position += velocity;

	boost::numeric::ublas::vector<double> rot( boost::numeric::ublas::zero_vector<double>(3) );

	for(size_t i=0; i < cameras.size(); ++i){
		cameras[i]->moveCamera(rot, linVel);
	}
}

boost::numeric::ublas::vector<double> Robot::getPosition(){
	return position;

}
