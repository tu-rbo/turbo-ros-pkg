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
 * CorrelationTracker Model (implementation)
 ******************************************************************************
 * $Author: dubik $
 * $Date: 2009/03/10 15:01:00 $
 * $Revision: 1.1 $
 *****************************************************************************/

#include "correlationTracker.h"
#include "feature.h"
#include "videoSensor.h"
#include "featureDetector.h"
#include "image.h"

#include "highgui.h"
#include "cv.h"
#include <iostream>
#include <string>

using namespace vision;

CorrelationTracker::CorrelationTracker(int winSize) : Tracker()
{
    _winSize = winSize;
    _mask = 0;
}
// ============================================================================

CorrelationTracker::~CorrelationTracker()
{

    std::vector<FeaturePtr>::const_iterator featuresIt =
                            Tracker::features.begin();

    features.clear();

    if(Tracker::curImage){
        delete Tracker::curImage;
        Tracker::curImage = 0;
    }

    if(_mask) delete _mask;

    Tracker::notifyResetListeners();
}
// ============================================================================


void CorrelationTracker::notify(const Sensor* sens){

    const VideoSensor* vSens = (dynamic_cast<const VideoSensor*>(sens));
    Image* newImage = vSens->getCurrentImage()->clone();

   if(Tracker::curImage){
        delete Tracker::curImage;
        Tracker::curImage = 0;
    }
    Tracker::curImage = newImage;
    _isFreshImage = true;
}


// ============================================================================

void CorrelationTracker::resetFeatures(FeatureDetector* d)
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
    Tracker::features = d->detect(Tracker::curImage, _mask);
    _count = Tracker::features.size();

    Tracker::ready = true;
    Tracker::notifyResetListeners();
}
// ============================================================================

void CorrelationTracker::step()
{
    if(!_isFreshImage){
        throw std::string(
        "You need to call step() on the sensor first");
    }
    //TODO: Leak
    Image *grey = Tracker::curImage->clone();
    std::vector<FeaturePtr>::const_iterator featsIt = Tracker::features.begin();
    while(featsIt != Tracker::features.end())
    {
        float newX, newY;
        float minDiff = 10000.0;

        FeaturePtr ft = *(featsIt++);
        if(!ft->isLost()){
            for(int x = -_winSize; x < _winSize; x++)
            {
                for(int y = -_winSize; y < _winSize; y++)
                {
                    std::pair<float, float> pt(ft->getX() + x, ft->getY() + y);
                    if(ft->isFeature(pt, grey, 0)){
                        float diff = ft->compareTo(pt, grey, _mask);
                        if(diff < minDiff){
                            newX = ft->getX() + x;
                            newY = ft->getY() + y;
                            minDiff = diff;
                        }
                    }
                }
            }
        }
        if(minDiff < 1000)
        {
            ft->setX(newX);
            ft->setY(newY);
            ft->setLost(false);
        }else{
            ft->setLost(true);
        }
    }
    Tracker::notifyStepListeners();
    _isFreshImage = false;
}

void CorrelationTracker::setForegroundMask(const Image* mask)
{
    if(_mask) delete _mask;
    _mask = mask->clone();
}
/** ***************************************************************************
                                End of File
 ******************************************************************************/

