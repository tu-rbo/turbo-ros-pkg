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
/** ***************************************************************************
* VideoSensor Model (implementation)
******************************************************************************
* $Author: dubik $
* $Date: 2009/03/10 15:01:15 $
* $Revision: 1.1 $
*****************************************************************************/

#include "videoSensor.h"
#include "sensorListener.h"
#include "imageInStream.h"

#include <iostream>
using namespace std;
using namespace vision;

VideoSensor::VideoSensor(std::string name){
	_iin = new ImageInStream(name);
	_currentImage = 0;
	cameraMatrix=NULL;
	distCoeffs=NULL;
	isCalibrated=false;
}
// ============================================================================

VideoSensor::VideoSensor(std::vector< std::string > fileList){
	_iin = new ImageInStream(fileList);
	_currentImage = 0;
	cameraMatrix=NULL;
	distCoeffs=NULL;
	isCalibrated=false;
}
// ============================================================================

VideoSensor::VideoSensor(){
	_iin = new ImageInStream();
	_currentImage = 0;
	cameraMatrix=NULL;
	distCoeffs=NULL;
	isCalibrated=false;
}
// ============================================================================

VideoSensor::~VideoSensor()
{
	if(_iin){
		delete _iin;
	}
	if(_currentImage){
		delete _currentImage;
	}
}
// ============================================================================

int VideoSensor::step(int count)
{
	int i = 0;

	try{
		while( (i < count) ){
			step();
			i++;
		}
	}catch(std::string &s){
		std::cout << s << std::endl;
	}

	return i;
}
// ============================================================================

void VideoSensor::step(){
	try{
		if(_currentImage) delete _currentImage;
		_currentImage = _iin->getNextImage();
		if(isCalibrated){
			Image* img = _currentImage->clone();
			cvUndistort2(img->getIplImage(), _currentImage->getIplImage(), cameraMatrix, distCoeffs);
			delete img;
		}

	}catch(std::string &s){
		_currentImage = 0;
		throw s;
	}

	notifyAll();
}
// ============================================================================

void VideoSensor::notifyAll()
{
	std::vector<SensorListener*>::iterator listenersIt;
	for(listenersIt = _listenerVector.begin();
		listenersIt != _listenerVector.end();
		listenersIt++)
	{
		SensorListener* listener = *(listenersIt);
		listener->notify(this);
	}
}
// ============================================================================
const Image* VideoSensor::getCurrentImage() const
{
	return _currentImage;
}

double VideoSensor::getFps() const
{
	return _iin->getFps();
}


double VideoSensor::getFrameCount() const
{
	double frameNum = _iin->getFrameCount();
	printf("Frame Count in Video sensor = %f", frameNum);
	return frameNum;
}

CvSize VideoSensor::getSize() const
{
	CvSize retSize = _iin->getSize();
	printf("Size of the video images = %d, %d\n", retSize.height, retSize.width);
	return retSize;
}

void VideoSensor::setCameraCalibration(double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2, double k3){
	if(cameraMatrix==NULL)	{
		cameraMatrix =  cvCreateMat(3,3,CV_32FC1);
		cameraMatrix->data.fl[0] = (float)fx;
		cameraMatrix->data.fl[1] = 0.0;
		cameraMatrix->data.fl[2] = (float)cx;
		cameraMatrix->data.fl[3] = 0.0;
		cameraMatrix->data.fl[4] = (float)fy;
		cameraMatrix->data.fl[5] = (float)cy;
		cameraMatrix->data.fl[6] = 0.0;
		cameraMatrix->data.fl[7] = 0.0;
		cameraMatrix->data.fl[8] = 1.0;

		if(k3==0.0)		{
			distCoeffs =  cvCreateMat(1,4,CV_32FC1);
			distCoeffs->data.fl[0] = (float)k1;
			distCoeffs->data.fl[1] = (float)k2;
			distCoeffs->data.fl[2] = (float)p1;
			distCoeffs->data.fl[3] = (float)p2;
		}else{
			distCoeffs =  cvCreateMat(1,5,CV_32FC1);
			distCoeffs->data.fl[0] = (float)k1;
			distCoeffs->data.fl[1] = (float)k2;
			distCoeffs->data.fl[2] = (float)p1;
			distCoeffs->data.fl[3] = (float)p2;
			distCoeffs->data.fl[4] = (float)k3;
		}
	}
	isCalibrated = true;
}

bool VideoSensor::calibrateCamera(int columns, int rows){
	bool retValue = _iin->calibrateCamera(columns, rows);
	isCalibrated = true;
	return retValue;
};

/** ***************************************************************************
End of File
******************************************************************************/
