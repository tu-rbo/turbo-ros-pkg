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
 * HSTracker Model (implementation)
 ******************************************************************************
 * $Author: dubik $
 * $Date: 2009/03/10 15:01:03 $
 * $Revision: 1.1 $
 *****************************************************************************/

#include "hsTracker.h"
#include "feature.h"
#include "videoSensor.h"
#include "featureDetector.h"
#include "image.h"

#include "cv.h"
#include <iostream>
#include <string>

using namespace vision;

HSTracker::HSTracker() : Tracker()
{
    _frame = 0;
    _flags = 0;
    _isInit=false;
    _isFreshImage = false;
}
// ============================================================================

HSTracker::~HSTracker()
{
    std::vector<FeaturePtr>::const_iterator featuresIt =
                            Tracker::features.begin();

    features.clear();

    Tracker::notifyResetListeners();
}
// ============================================================================

void HSTracker::notify(const Sensor* sens){

    const VideoSensor* vSens = (dynamic_cast<const VideoSensor*>(sens));
    Image* newImage = vSens->getCurrentImage()->clone();

    if(!_isInit){
        IplImage* newIImage = newImage->getIplImage();
        _imgSize =       cvGetSize(newIImage);
        _image =         cvCreateImage( _imgSize, 8, 3 );
        _image->origin = newIImage->origin;
        _grey =          cvCreateImage( _imgSize, 8, 1 );
        _prevGrey =      cvCreateImage( _imgSize, 8, 1 );
        _xImg =          cvCreateImage( _imgSize, IPL_DEPTH_32F, 1 );
        _yImg =          cvCreateImage( _imgSize, IPL_DEPTH_32F, 1 );
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

void HSTracker::resetFeatures(FeatureDetector* d)
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

    Tracker::ready = true;
    Tracker::notifyResetListeners();
}
// ============================================================================

void HSTracker::step()
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
        cvCalcOpticalFlowHS( _prevGrey, _grey, 0,
                     _xImg, _yImg, 1,
                     cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));

        float x, y;
        std::vector<FeaturePtr>::iterator featuresIt;
        for(featuresIt = Tracker::features.begin();
            featuresIt !=Tracker::features.end(); featuresIt++)
        {
            FeaturePtr feat = *(featuresIt);
	    if(!feat->isLost())
	    {
                CvPoint2D32f ptf  = cvPoint2D32f(feat->getX(), feat->getY());
                CvPoint pt = cvPointFrom32f(ptf);

	       //std::cout << pt.x << " " << pt.y <<" " << _imgSize.width << " " << _imgSize.height << " b4\n";
                x = (float)((feat->getX() + .9*cvGetReal2D( _xImg, pt.y, pt.x )));
                y = (float)((feat->getY() + .9*cvGetReal2D( _yImg, pt.y, pt.x )));
                 // std::cout << "a4\n";
                feat->setError(0);
                feat->setPos(x, y);

                if( (x <= 0 || x > _imgSize.width) ||
                    (y <= 0 || y > _imgSize.height) )
                {
                    feat->setLost(true);
	        }
	    }

        }
    }

    Tracker::notifyStepListeners();
    _isFreshImage = false;
}
/** ***************************************************************************
                                End of File
 ******************************************************************************/

