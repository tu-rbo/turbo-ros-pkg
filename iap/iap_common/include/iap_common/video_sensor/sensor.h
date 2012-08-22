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
#ifndef SENSOR_H
#define SENSOR_H


#include <vector>
#include <string>

namespace vision{

 class SensorListener;

/**
 *\class    Sensor
 *\brief    Abstract class. Describes a sensor. Following the Publish-Subscribe
 *design pattern, this abstract class supports multiple SensorListeners that are
 *notified after sucessfull calls to step(). Classes that inherit from Sensor
 *implement step() by retreiving the sensor data and notifying its listeners,
 *making new sensor data available to them.
 *$Author: dubik $
 *$Date: 2009/03/10 15:01:12 $
 *$Revision: 1.1 $ 
  *****************************************************************************/
 class Sensor{
  public:
    /**
     * Default constructor
     */
    Sensor();

    /**
     * Default destructor
     */
    virtual ~Sensor();

    /**
     * Adds objects that implement the SensorListener interface wich requires a
     * function called notifyStep(..)
     * 
     * @param listener - The object that will be notified after the call to step
     */
    void addListener(SensorListener* listener);
	
    /**
     * Inheritaing class must retrieve the sensor data and assign it
     * to _sensorData then call notifyStepListeners()
     */
    virtual void step()   = 0;

    /**
     * Inheritaing class must call step() int times. The function must
     * return number of sucessfull calls to step()
     * 
     * @param count - the number of times to call step()
     */
    virtual int step(int count) = 0;

  protected:
    /** The vector of listeners*/	
    std::vector<SensorListener*> _listenerVector; 	
 };
};
#endif
