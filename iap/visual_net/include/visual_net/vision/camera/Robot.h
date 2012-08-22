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
 * Robot.h
 *
 *  Created on: Aug 7, 2009
 *      Author: jkclevel
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <vector>
#include "feature.h"
#include "CameraModel.h"

class Robot {
public:
	Robot();
	Robot(boost::numeric::ublas::vector<double> &position);
	virtual ~Robot();

	//TODO implement
	//bool checkCollision(std::vector<Feature> &points);

	//void setVel(boost::numeric::ublas::vector<double> &velocity);
	//boost::numeric::ublas::vector<double> getVel();

	void step(boost::numeric::ublas::vector<double> &linVel);

	void addCamera(vision::CameraModel *cam);
	std::vector<vision::CameraModel*> getCameras();

	boost::numeric::ublas::vector<double> getPosition();

private:
	std::vector<vision::CameraModel*> cameras;
	boost::numeric::ublas::vector<double> velocity;
	boost::numeric::ublas::vector<double> position;

};

#endif /* ROBOT_H_ */
