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
#include "CameraModel.h"
using namespace boost::numeric::ublas;
using namespace vision;

//Constructs a new camera object with focalLength 
//No translation or rotation
CameraModel::CameraModel(double focalLength, int xResolution, int yResolution)
{
	this->focalLength = focalLength;
	this->translation = zero_vector<double>(3);
	boost::numeric::ublas::vector<double> tmp = zero_vector<double>(3);
	this->rotation_matrix = rotR( tmp );
	this->xResolution=xResolution;
	this->yResolution=yResolution;
}

//constructs a camera object
//translation is a vector [ x y z ]
//rotation is a vector [ theta_x theta_y theta_z ]
//focalLength is the camera's focal length
CameraModel::CameraModel(boost::numeric::ublas::vector<double> &translation, boost::numeric::ublas::vector<double> &rotation, double focalLength)
{
	this->focalLength = focalLength;
	this->translation = boost::numeric::ublas::vector<double>(translation);
	this->rotation_matrix = matrix<double>(rotR(rotation));
}
CameraModel::~CameraModel(){
}


//given a 3 x n matrix which contains the x y z of n features, returns the observation
//as the camera would see them
std::vector<FeaturePtr> CameraModel::getObs(matrix<double> &true_pts){

     std::vector<vision::FeaturePtr> featureVector;
     matrix<double> pts(true_pts);

 	//for each feature 	apply translation
 	for(unsigned int j=0; j<true_pts.size2(); ++j){
 	  pts(0,j) += (translation(0)  ) ;
 	  pts(1,j) += (translation(1)  );
 	  pts(2,j) += (translation(2)  );
 	}

     //apply rotation after translation
 	pts = prod(rotation_matrix, pts) ;

	//for each feature put through camera model
	for(unsigned int j=0; j<true_pts.size2(); ++j){
	  FeaturePtr f (new Feature((float) (pts(0,j)*(focalLength/(pts(2,j)))), (float) (pts(1,j)*(focalLength/(pts(2,j))))));
	  //add 2d feature to vector
	  featureVector.push_back(f);
		  
	}

	return featureVector;
}

//given a matrix of points, returns a vector with the observations of format
// (u1 v1 u2 v2 ... un vn)
void CameraModel::getRowObs(matrix<double> &true_pts, boost::numeric::ublas::vector<double> &obs){
  //apply rotation
    matrix<double> temp_pts( prod(rotation_matrix, true_pts) );
    matrix<double> pts( true_pts );

	
  //for each point
  for(unsigned int j=0; j<true_pts.size2(); ++j){
    //apply translation
	  pts(0,j) += (translation(0) ) ;
	  pts(1,j) += (translation(1) );
	  pts(2,j) += (translation(2)  );
  }
  pts = prod(rotation_matrix, pts);

  //for each point  perform 2d projection
  for(unsigned int j=0; j<true_pts.size2(); ++j){
    obs(2*j)=pts(0,j)*(focalLength/(pts(2,j)));
    obs(2*j + 1)=pts(1,j)*(focalLength/(pts(2,j)));
 }      
}

//set the absolute position of the cameras rotation and translation
void CameraModel::setCameraPos(boost::numeric::ublas::vector<double> rotation, boost::numeric::ublas::vector<double> linTrans){
	this->rotation_matrix = rotR(rotation);
	translation = linTrans;
}

//set the cameras
void CameraModel::moveCamera(boost::numeric::ublas::vector<double> rotation, boost::numeric::ublas::vector<double> linTrans){

	matrix<double> deltaRot( rotR(rotation) );

	rotation_matrix = prec_prod(rotation_matrix, rotR(rotation) );

	translation += linTrans;
}

double CameraModel::getFocalLength(){
	return focalLength;
}


//returns vector [theta_x theta_y theta_z] as a rotation matrix
matrix<double> CameraModel::rotR(boost::numeric::ublas::vector<double> &w){
	double x=w[0];
	double y=w[1];
	double z=w[2];
	

	matrix<double> rotx(3,3);
	matrix<double> roty(3,3);
	matrix<double> rotz(3,3);
	matrix<double> rotAll(3,3);

	rotx(0,0)= 1.0;			rotx(0,1)= 0;			rotx(0,2)=0;	
	rotx(1,0)= 0;			rotx(1,1)= cos(x);		rotx(1,2)= sin(x)*-1;	
	rotx(2,0)= 0;			rotx(2,1)= sin(x);		rotx(2,2)= cos(x);	

	roty(0,0)= cos(y);		roty(0,1)= 0;			roty(0,2)=sin(y);	
	roty(1,0)= 0.0;			roty(1,1)= 1.0;			roty(1,2)=0;	
	roty(2,0)= sin(y)*-1;	roty(2,1)= 0;			roty(2,2)=cos(y);	

	rotz(0,0)= cos(z);		rotz(0,1)= sin(z)*-1;	rotz(0,2)=0;	
	rotz(1,0)= sin(z);		rotz(1,1)= cos(z);		rotz(1,2)=0;	
	rotz(2,0)= 0;			rotz(2,1)= 0;			rotz(2,2)=1.0;	
	
	matrix<double> tmp(prod(rotz,roty));	
	rotAll = prod(tmp,rotx);
	return rotAll;
}

vision::FeaturePtr CameraModel::getInvObs(vision::FeaturePtr f, double depth)
{
	double x,y,z;
	x = (f->getX() - xResolution/2)*depth/focalLength;
	y = -(f->getY() - yResolution/2)*depth/focalLength;
	z = depth;
	return FeaturePtr(new Feature((float)x,(float)y,(float)z));
}
