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
 * Display Video Sensor (implementation)
 ******************************************************************************
 * $Author: dubik $
 * $Date: 2009/03/10 15:01:14 $
 * $Revision: 1.1 $
 *****************************************************************************/
// #define DEBUG 1

#include <iostream>
#include <cv.h>
#include <highgui.h>
#include "displayVideoSensor.h"
#include "videoSensor.h"

using namespace std;
using namespace vision;

// ============================================================================
DisplayVideoSensor::DisplayVideoSensor() {
    #ifdef DEBUG
        cout<<"[Segmenter] - Constructor"<<endl;
    #endif
    _frame = 0;
}// ===========================================================================
DisplayVideoSensor::~DisplayVideoSensor() {
    #ifdef DEBUG
        cout<<"[Segmenter] - Destructor"<<endl;
    #endif
    delete _frame;
}// ===========================================================================
void DisplayVideoSensor::notify(const Sensor *pSensor){
   cout<<"In Display Video Sensor"<<endl;
    
    
    #ifdef DEBUG
        cout<<"[Segmenter] - Notify"<<endl;
    #endif
     
     if(_frame!=0)
        delete _frame;
    
    cout<<"After delete"<<endl;
    try{
        const VideoSensor* videoSensor = dynamic_cast<const VideoSensor*>(pSensor);
        _frame = videoSensor->getCurrentImage()->clone();
        _frame->display("Current Frame");
    }catch(string &s){
        _frame = 0;
        cout<<"[Segmenter::notify()] Throwing: "<<s<<endl;
    }
    
    cout<<"Leaving display video sensor"<<endl;
}// ===========================================================================
/*void DisplayVideoSensor::step() {
  
   _frame->display("Current Frame");
}*/
/** ***************************************************************************
                                End of File
******************************************************************************/







