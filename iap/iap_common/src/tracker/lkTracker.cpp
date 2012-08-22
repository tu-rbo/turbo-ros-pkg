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
 * LKTracker Model (implementation)
 ******************************************************************************
 * $Author: dubik $
 * $Date: 2009/03/10 15:01:04 $
 * $Revision: 1.1 $
 *****************************************************************************/

#include "lkTracker.h"
#include "feature.h"
#include "videoSensor.h"
#include "featureDetector.h"
#include "image.h"

#include "highgui.h"
#include "cv.h"
#include <iostream>
#include <string>

using namespace vision;

LKTracker::LKTracker(int winSize, int pyrLevels) : Tracker()
{
    _pyrLevels = pyrLevels;
    _winSize = winSize;
    _frame = 0;
    _flags = 0;
    _status = 0;
    _error = 0;
    _points[0] = 0;
    _points[1] = 0;
    _isInit=false;
    _isFreshImage = false;

}
// ============================================================================

LKTracker::~LKTracker()
{

    std::vector<FeaturePtr>::const_iterator featuresIt =
                            Tracker::features.begin();

    features.clear();

    if(Tracker::curImage){
        delete Tracker::curImage;
        Tracker::curImage = 0;
    }

    if(_status){
        delete[] _status;
    }
    if(_error){
        delete[] _error;
    }
    if(_points[0]){
        cvFree(&_points[0]);
    }
    if(_points[1]){
        cvFree(&_points[1]);
    }

    Tracker::notifyResetListeners();
}
// ============================================================================

int LKTracker::getCount()
{
    return _count;
}
// ============================================================================

void LKTracker::notify(const Sensor* sens){

    const VideoSensor* vSens = (dynamic_cast<const VideoSensor*>(sens));
    Image* newImage = vSens->getCurrentImage()->clone();

    if(!_isInit){
        IplImage* newIImage = newImage->getIplImage();
        _imgSize =       cvGetSize(newIImage);
        _image =         cvCreateImage( _imgSize, 8, 3 );
        _image->origin = newIImage->origin;
        _grey =          cvCreateImage( _imgSize, 8, 1 );
	_greyTmp =       cvCreateImage( _imgSize, 8, 1 );
        _prevGrey =      cvCreateImage( _imgSize, 8, 1 );
        _pyramid =       cvCreateImage( _imgSize, 8, 1 );
        _prevPyramid =   cvCreateImage( _imgSize, 8, 1 );
        _isInit = true;
    }

   if(Tracker::curImage){
        IplImage* oldIImage = Tracker::curImage->getIplImage();
        cvCvtColor( oldIImage, _prevGrey, CV_BGR2GRAY );

        delete Tracker::curImage;
        Tracker::curImage = 0;
    }
    Tracker::curImage = newImage;
    _isFreshImage = true;
}


// ============================================================================

void LKTracker::resetFeatures(FeatureDetector* d)
{

    if(Tracker::curImage == 0){
        throw std::string(
              "there is no image on wich to detect features");
    }

    // BEGIN: delete the features that may have been detected preveously
    std::vector<FeaturePtr>::const_iterator featuresItD =
                            Tracker::features.begin();
    Tracker::features.clear();
    // END: delete

    // detect features on the last known image
    Tracker::features = d->detect(Tracker::curImage);
    _count = Tracker::features.size();

    //free memory used for last feature set
    if(_status) delete[] _status;
    if(_error)  delete[] _error;
    if(_points[0]) cvFree(&_points[0]);
    if(_points[1]) cvFree(&_points[1]);

    //allocate new memoy gor new feature set
    _status = new char[_count];
    _error  = new float[_count];
    _points[0] = (CvPoint2D32f*)cvAlloc(_count*sizeof(_points[0][0]));
    _points[1] = (CvPoint2D32f*)cvAlloc(_count*sizeof(_points[0][0]));

    // initialize the points array for use with the opencv tracker algoritm
    std::vector<FeaturePtr>::iterator featuresIt = Tracker::features.begin();
    int i = 0;
    while(featuresIt != Tracker::features.end())
    {
        FeaturePtr feat = *(featuresIt++);
        _points[0][i].x = feat->getX();
        _points[0][i].y = _imgSize.height - feat->getY();
        i++;
    }

    Tracker::ready = true;
    Tracker::notifyResetListeners();
}
// ============================================================================

void LKTracker::step()
{
    if(!_isFreshImage){
        throw std::string(
        "You need to call step() on the sensor first");
    }
    _frame = Tracker::curImage->getIplImage();

    cvCopy( _frame, _image, 0 );
    cvCvtColor( _image, _grey, CV_BGR2GRAY );

    if(Tracker::ready && _count > 0)
    {	
        cvCalcOpticalFlowPyrLK( _prevGrey, _grey, _prevPyramid, _pyramid,
        _points[0], _points[1], _count, cvSize(_winSize,_winSize), _pyrLevels,
        _status, _error,
        cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), _flags
        );
       // _flags |= CV_LKFLOW_PYR_A_READY;

        //for opencv 0.9.9; the _points that are declared lost must be
        //removed for the next iteration
        int newCount = 0;
        for(int i=0;i<_count;i++)
        {
            if( (_points[1][i].x <= 0 || _points[1][i].x > _imgSize.width) ||
                (_points[1][i].y <= 0 || _points[1][i].y > _imgSize.height) )
                   _status[i] = 0;

            if(!_status[i]) continue;

            _points[1][newCount++] = _points[1][i];
        }
        _count = newCount;
        // done deleting

        int j=0;
        int i=0;
        std::vector<FeaturePtr>::iterator featuresIt;
        for(featuresIt = Tracker::features.begin();
            featuresIt !=Tracker::features.end(); featuresIt++)
        {
            FeaturePtr feat = *(featuresIt);

            if(!feat->isLost())
            {
                if(_status[j] == 1)
                {
                    feat->setLost(false);
                    feat->setError(_error[i]);
                    feat->setPos(_points[1][i].x, _imgSize.height - _points[1][i].y);
                    i++;
                }else
                {
                    feat->setLost(true);
                    feat->setError(-1);
                }
                j++;
            }
        }
    }

    CV_SWAP( _prevPyramid, _pyramid, _swapTemp );
    CV_SWAP( _points[0], _points[1], _swapPoints );	

    Tracker::notifyStepListeners();
    _isFreshImage = false;
}

void LKTracker::resetFeatures(std::vector<FeaturePtr> detectedFeatures){
    // BEGIN: delete the features that may have been detected preveously
    std::vector<FeaturePtr>::const_iterator featuresItD =
                            Tracker::features.begin();
    Tracker::features.clear();
    // END: delete

	std::vector<FeaturePtr>::iterator featuresIt = detectedFeatures.begin();
	int countTemp = 0;
	for(;featuresIt != detectedFeatures.end();featuresIt++)
    {
        if(!(*featuresIt)->isLost()){
			countTemp+=1;
		}
    }
    _count = countTemp;

    //free memory used for last feature set
    if(_status) delete[] _status;
    if(_error)  delete[] _error;
    if(_points[0]) cvFree(&_points[0]);
    if(_points[1]) cvFree(&_points[1]);

    //allocate new memoy gor new feature set
    _status = new char[_count];
    _error  = new float[_count];
    _points[0] = (CvPoint2D32f*)cvAlloc(_count*sizeof(_points[0][0]));
    _points[1] = (CvPoint2D32f*)cvAlloc(_count*sizeof(_points[0][0]));

    // initialize the points array for use with the opencv tracker algoritm
    std::vector<FeaturePtr>::iterator featuresIt2 = detectedFeatures.begin();
    int i = 0;
    for(;featuresIt2 != detectedFeatures.end();featuresIt2++)
    {
		if(!(*featuresIt2)->isLost()){
        FeaturePtr feat = *featuresIt2;
        _points[0][i].x = feat->getX();
        //_points[0][i].y = _imgSize.height - feat->getY();
		_points[0][i].y = feat->getY();
		this->features.push_back(feat);
        i++;
		}
    }
	//this->features = detectedFeatures;    //Doing it this way, the y-values are mirrored
    Tracker::ready = true;
    Tracker::notifyResetListeners();
}

void LKTracker::resetFeatures(std::vector<SURFFeaturePtr> detectedFeatures){
    // BEGIN: delete the features that may have been detected preveously
    std::vector<FeaturePtr>::const_iterator featuresItD =
                            Tracker::features.begin();
    Tracker::features.clear();
    // END: delete

	std::vector<SURFFeaturePtr>::iterator featuresIt = detectedFeatures.begin();
	int countTemp = 0;
	for(;featuresIt != detectedFeatures.end();featuresIt++)
    {
        if(!(*featuresIt)->isLost()){
			countTemp+=1;
		}
    }
    _count = countTemp;

    //free memory used for last feature set
    if(_status) delete[] _status;
    if(_error)  delete[] _error;
    if(_points[0]) cvFree(&_points[0]);
    if(_points[1]) cvFree(&_points[1]);

    //allocate new memoy gor new feature set
    _status = new char[_count];
    _error  = new float[_count];
    _points[0] = (CvPoint2D32f*)cvAlloc(_count*sizeof(_points[0][0]));
    _points[1] = (CvPoint2D32f*)cvAlloc(_count*sizeof(_points[0][0]));

    // initialize the points array for use with the opencv tracker algoritm
    std::vector<SURFFeaturePtr>::iterator featuresIt2 = detectedFeatures.begin();
    int i = 0;
    for(;featuresIt2 != detectedFeatures.end();featuresIt2++)
    {
		if(!(*featuresIt2)->isLost()){
        FeaturePtr feat = *featuresIt2;
        _points[0][i].x = feat->getX();
        //_points[0][i].y = _imgSize.height - feat->getY();
		_points[0][i].y = feat->getY();
		this->features.push_back(feat);
        i++;
		}
    }
    Tracker::ready = true;
    Tracker::notifyResetListeners();
}
/** ***************************************************************************
                                End of File
 ******************************************************************************/

