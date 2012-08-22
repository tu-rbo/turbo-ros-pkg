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
#ifndef __DisplayVideoSensor_H__
#define __DisplayVideoSensor_H__

#include "image.h"
#include "sensorListener.h"

/** ***************************************************************************
 *\class    Segmenter
 *\brief    Abstact class for Segmentation classes, subscribes to sensors, etc.
 * $Author: dubik $
 * $Date: 2009/03/10 15:01:11 $
 * $Revision: 1.1 $
 *****************************************************************************/
namespace vision {
    class DisplayVideoSensor : public SensorListener {
     public:
        DisplayVideoSensor(); /**< Constructor */
        ~DisplayVideoSensor(); /**< Destructor */

        /**
        * Virtual function to be implemented by a derived class
        */
        //void step();

        /**
        * Recieve notifications from sensors we are subscribed to.
        * In our case this is a video sensor, giving us a new frame.
        * @param *pSen - the sensor who is notifying us of an update
        */
        void notify(const Sensor *pSensor);
        

     private:
        Image *_frame; /**< the last received frame */
    };
}
#endif

/** ***************************************************************************
                                End of File
******************************************************************************/






