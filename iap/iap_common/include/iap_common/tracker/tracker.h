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
#ifndef TRACKER_H
#define TRACKER_H

#include <vector>
#include "sensorListener.h"
#include <boost/shared_ptr.hpp>

namespace vision{

 class Sensor;
 class TrackerListener;
 class Feature;
 typedef boost::shared_ptr<Feature> FeaturePtr;
 class Image;
 class FeatureDetector;
/**
 *\class    Tracker
 *\brief    Abstract class. Implements SensorListener. Tracks features on
 * viseoSensor data. Following the Publish-Subscribe design pattern, this
 * abstract class supports multiple TrackListeners that are notified after the
 * sucessfull calls to step and resetFeatures(). Additionally, this class
 * imoplements the SensorListener interface wich, when subscribed to a
 * VideoSensor gets notified whenever there is new video sensor data.
 * This data is tracked after calling step and all registered TrackerListeners
 * get notified.
 *$Author: dubik $
 *$Date: 2009/03/11 17:53:18 $
 *$Revision: 1.2 $
 */
 class Tracker : public SensorListener{
  public:

    /**
     * default constructor
     */
    Tracker();

    /**
     * default destructor
     */
    virtual ~Tracker();

    /**
     * adds to the list of listeners that get notified after calls to step and
     * reset features
     *
     * @param listener an object to be notified
     */
    void addListener(TrackerListener* listener);

    /**
     * detects a new set of features for tracking
     * 
     * @param d - the detector used for detecting new features
     */
    virtual void resetFeatures(FeatureDetector* d) = 0;

    /**
     * called whenever there is a new image available to the tracker
     *
     * @param sens - must be of type VideoSensor, the Sensor class that
     * this tracker is subscribed to.
     */
    virtual void notify(const Sensor* sens) = 0;

    /**
     * attempts to step through to the next iteration of tracking if there 
     * is there is a new image available
     */
    virtual void step() =0;

  protected:
    void notifyStepListeners();            // calls the notifyStep method of listeners
    void notifyResetListeners();/**< calls the notifyReset method of listeners*/

    /**
     * This is the list of objects that will be notified via
     * notifyStepListeners() and  notifyResetListeners();
     * method each time a frame is tracked.
     */
    std::vector<TrackerListener*> listeners;

    /**
     * This is the list of features that are found and tracked
     */
    std::vector<FeaturePtr> features;
    Image* curImage; /**< the last image optained from the sensor */
    bool ready;
 };
};
#endif
