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
#ifndef __CAMERAMODEL_H__
#define __CAMERAMODEL_H__

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <vector>
#include "feature.h"
//#include "MatrixInversion.h"

namespace vision{

class CameraModel{

public:
	//c'tor
	CameraModel(double f=1.1978, int xResolution=640, int yResolution=480);
	
	//c'tor: translation = camera's offset, rotation = camera's orientation, f = camera's focal length
	CameraModel(boost::numeric::ublas::vector<double> &translation, boost::numeric::ublas::vector<double> &rotation, double f);

	//d'tor
	virtual ~CameraModel();

	//returns a vector containing camera's view of the supplied points
	std::vector<vision::FeaturePtr> getObs(boost::numeric::ublas::matrix<double> &true_pts);
	//returns a 3D feature form observation and depth
	vision::FeaturePtr getInvObs(vision::FeaturePtr f, double depth);

	//returns a vector containing camera's view of the supplied points
	void getRowObs(boost::numeric::ublas::matrix<double> &true_pts, boost::numeric::ublas::vector<double> &obs);	

	void moveCamera(boost::numeric::ublas::vector<double> rotation, boost::numeric::ublas::vector<double> translation); 
	void setCameraPos(boost::numeric::ublas::vector<double> rotation, boost::numeric::ublas::vector<double> linTrans);
	double getFocalLength();

	int getXresolution(){return xResolution;}
	int getYresolution(){return yResolution;}

private:
	
	boost::numeric::ublas::matrix<double> rotR(boost::numeric::ublas::vector<double> &w);	//returns a rotation matrix constructed from a given vector of rotations
	boost::numeric::ublas::vector<double> translation; 										//3 vector containing the linear translation of the camera
	boost::numeric::ublas::matrix<double> rotation_matrix; 									//3x3 matrix describing the orientation of the camera
	double	focalLength;																	//cameras focal length
	int xResolution;
	int yResolution;


};
};

#endif /*CameraModel_H_*/

